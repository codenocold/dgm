#include "controller.h"
#include "encoder.h"
#include "hw_config.h"
#include "usr_config.h"
#include "arm_math.h"
#include "trapTraj.h"
#include "anticogging.h"
#include "util.h"

ControllerStruct Controller;

static volatile bool input_pos_updated = false;

static float mPosSetPoint;
static float mVelSetPoint;
static float mTorqueSetPoint;

static float input_filter_kp;
static float input_filter_ki;

static float vel_integrator_torque;

float CONTROLLER_get_integrator_current(void)
{
	return vel_integrator_torque;
}

void CONTROLLER_reset(ControllerStruct *controller)
{
	controller->input_pos = Encoder.position;
	controller->input_vel = 0;
	controller->input_torque = 0;
	
	mPosSetPoint = controller->input_pos;
	mVelSetPoint = 0;
	mTorqueSetPoint = 0;
	
	Traj.trajectory_done = true;
	
	vel_integrator_torque = 0;
	
	// update_filter_gains
	float bandwidth = MIN(UsrConfig.input_pos_filter_bandwidth, 0.25f * CURRENT_MEAS_HZ);
    input_filter_ki = 2.0f * bandwidth;
    input_filter_kp = 0.25f * (input_filter_ki * input_filter_ki);
}

void CONTROLLER_move_to_pos(float goal_point)
{
	if(UsrConfig.input_mode == INPUT_MODE_TRAP_TRAJ && UsrConfig.control_mode == CONTROL_MODE_POSITION_CONTROL){
		Controller.input_pos = goal_point;
		input_pos_updated = true;
	}
}

static void move_to_pos(float goal_point)
{
    TRAJ_plan(goal_point, mPosSetPoint, mVelSetPoint,
                                 UsrConfig.traj_vel,		// Velocity
                                 UsrConfig.traj_accel,		// Acceleration
                                 UsrConfig.traj_decel);		// Deceleration
	
    Traj.t = 0.0f;
    Traj.trajectory_done = false;
}

void CONTROLLER_update(ControllerStruct *controller, float *current_setpoint_output)
{
	// Update inputs
    switch (UsrConfig.input_mode) {
        case INPUT_MODE_PASSTHROUGH: {
            mPosSetPoint = Controller.input_pos;
            mVelSetPoint = Controller.input_vel;
            mTorqueSetPoint = Controller.input_torque; 
        } break;
		case INPUT_MODE_TORQUE_RAMP: {
            float max_step_size = fabs(DT * UsrConfig.torque_ramp_rate);
            float full_step = Controller.input_torque - mTorqueSetPoint;
            float step = CLAMP(full_step, -max_step_size, max_step_size);
            mTorqueSetPoint += step;
        } break;
        case INPUT_MODE_VEL_RAMP: {
            float max_step_size = fabs(DT * UsrConfig.vel_ramp_rate);
            float full_step = Controller.input_vel - mVelSetPoint;
            float step = CLAMP(full_step, -max_step_size, max_step_size);
            mVelSetPoint += step;
            mTorqueSetPoint = (step / DT) * UsrConfig.inertia;
        } break;
        case INPUT_MODE_POS_FILTER: {
            // 2nd order pos tracking filter
            float delta_pos = Controller.input_pos - mPosSetPoint; // Pos error
            float delta_vel = Controller.input_vel - mVelSetPoint; // Vel error
            float accel = input_filter_kp*delta_pos + input_filter_ki*delta_vel; // Feedback
            mTorqueSetPoint = accel * UsrConfig.inertia; // Accel
            mVelSetPoint += DT * accel; 		// delta vel
            mPosSetPoint += DT * mVelSetPoint; 	// Delta pos
        } break;
        case INPUT_MODE_TRAP_TRAJ: {
            if(input_pos_updated){
                move_to_pos(Controller.input_pos);
                input_pos_updated = false;
            }
			
            // Avoid updating uninitialized trajectory
            if (Traj.trajectory_done){
                break;
			}
            
            if (Traj.t > Traj.Tf_) {
                // Drop into position control mode when done to avoid problems on loop counter delta overflow
                UsrConfig.control_mode = CONTROL_MODE_POSITION_CONTROL;
                mPosSetPoint = Controller.input_pos;
                mVelSetPoint = 0.0f;
                mTorqueSetPoint = 0.0f;
                Traj.trajectory_done = true;
            } else {
				TRAJ_eval(Traj.t);
				mPosSetPoint = Traj.Y;
				mVelSetPoint = Traj.Yd;
				mTorqueSetPoint = Traj.Ydd * UsrConfig.inertia;
				Traj.t += DT;
            }
        } break;
        default: {
        } break;
    }
	
	// Position control
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = mVelSetPoint;
    if (UsrConfig.control_mode >= CONTROL_MODE_POSITION_CONTROL) {
        float pos_err;
		pos_err = mPosSetPoint - Encoder.position;
        vel_des += UsrConfig.pos_gain * pos_err;
        // V-shaped gain shedule based on position error
        float abs_pos_err = fabs(pos_err);
        if (UsrConfig.gain_scheduling_enable && abs_pos_err <= UsrConfig.gain_scheduling_width) {
            gain_scheduling_multiplier = abs_pos_err / UsrConfig.gain_scheduling_width;
        }
    }

    // Velocity limiting
    vel_des = CLAMP(vel_des, -UsrConfig.vel_limit, UsrConfig.vel_limit);

    // Velocity control
    float torque = mTorqueSetPoint;

    float v_err = 0.0f;
    if (UsrConfig.control_mode >= CONTROL_MODE_VELOCITY_CONTROL) {
        v_err = vel_des - Encoder.velocity;
        torque += (UsrConfig.vel_gain * gain_scheduling_multiplier) * v_err;

        // Velocity integral action before limiting
        torque += vel_integrator_torque;
    }
	
	float current = torque / UsrConfig.torque_constant;
	
	// Anticogging
	if(UsrConfig.anticogging_enable && AnticoggingValid){
		current += pCoggingMap->map[(COGGING_MAP_NUM-1)*Encoder.cnt/ENCODER_CPR];
	}

    // Current limit
    bool limited = false;
    if (current > UsrConfig.current_limit) {
        limited = true;
        current = UsrConfig.current_limit;
    }
    if (current < -UsrConfig.current_limit) {
        limited = true;
        current = -UsrConfig.current_limit;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (UsrConfig.control_mode < CONTROL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_torque = 0.0f;
    } else {
        if (limited) {
            vel_integrator_torque *= 0.99f;
        } else {
            vel_integrator_torque += ((UsrConfig.vel_integrator_gain * gain_scheduling_multiplier) * DT) * v_err;
        }
    }

    if (current_setpoint_output){
		*current_setpoint_output = current;
	}
}
