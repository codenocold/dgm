/*
    Copyright 2021 codenocold codenocold@qq.com
    Address : https://github.com/codenocold/dgm
    This file is part of the dgm firmware.
    The dgm firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    The dgm firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "controller.h"
#include "anticogging.h"
#include "encoder.h"
#include "foc.h"
#include "mc_task.h"
#include "pwm_curr.h"
#include "trapTraj.h"
#include "usr_config.h"
#include "util.h"

tController Controller;

int CONTROLLER_set_op_mode(tControlMode mode)
{
    __disable_irq();

    Controller.ctrl_mode = mode;
    CONTROLLER_reset();

    __enable_irq();

    return 0;
}

int CONTROLLER_set_home(void)
{
    int ret = 0;

    if (MCT_get_state() == IDLE) {
        Encoder.shadow_count = 0;
    } else if (MCT_get_state() == RUN) {
        if (ABS(Encoder.vel) < 0.5f && Traj.profile_done) {
            __disable_irq();
            Controller.input_position        = 0;
            Controller.input_position_buffer = 0;
            Controller.pos_setpoint          = 0;
            Encoder.shadow_count             = 0;
            Encoder.pos                      = 0;
            __enable_irq();
        } else {
            ret = -1;
        }
    } else {
        ret = -1;
    }

    return ret;
}

void CONTROLLER_sync_callback(void)
{
    if (MCT_get_state() != RUN) {
        return;
    }

    switch (Controller.ctrl_mode) {
    case CONTROL_MODE_CURRENT_RAMP:
        Controller.input_current = Controller.input_current_buffer;
        break;

    case CONTROL_MODE_VELOCITY_RAMP:
        Controller.input_velocity = Controller.input_velocity_buffer;
        break;

    case CONTROL_MODE_POSITION_FILTER:
        Controller.input_position = Controller.input_position_buffer;
        break;

    case CONTROL_MODE_POSITION_PROFILE:
        Controller.input_position = Controller.input_position_buffer;
        Controller.input_updated  = true;
        break;

    default:
        break;
    }

    StatuswordNew.status.target_reached = 0;
}

void CONTROLLER_init(void)
{
    Controller.ctrl_mode = (tControlMode) UsrConfig.default_op_mode;

    CONTROLLER_update_input_pos_filter_gain(UsrConfig.position_filter_bw);
}

void CONTROLLER_update_input_pos_filter_gain(float bw)
{
    float bandwidth                = bw * M_2PI;
    Controller.input_pos_filter_ki = 2.0f * bandwidth;
    Controller.input_pos_filter_kp = 0.25f * SQ(Controller.input_pos_filter_ki);
}

void CONTROLLER_reset(void)
{
    __disable_irq();

    float pos_meas = Encoder.pos;

    if (MCT_get_state() == ANTICOGGING) {
        pos_meas = (Encoder.count_in_cpr / ENCODER_CPR_F);
    }

    Controller.input_position = pos_meas;
    Controller.input_velocity = 0.0f;
    Controller.input_current  = 0.0f;

    Controller.input_position_buffer = pos_meas;
    Controller.input_velocity_buffer = 0.0f;
    Controller.input_current_buffer  = 0.0f;

    Controller.pos_setpoint = pos_meas;
    Controller.vel_setpoint = 0.0f;
    Controller.cur_setpoint = 0.0f;

    Controller.vel_integrator = 0;

    Controller.input_updated = false;
    Traj.profile_done        = true;

    Foc.current_ctrl_integral_d = 0;
    Foc.current_ctrl_integral_q = 0;

    __enable_irq();
}

void CONTROLLER_loop(void)
{
    float       vel_des;
    const float pos_meas       = Encoder.pos;
    const float vel_meas       = Encoder.vel;
    const float phase_meas     = Encoder.phase;
    const float phase_vel_meas = Encoder.phase_vel;

    if (MCT_get_state() == RUN) {
        switch (Controller.ctrl_mode) {
        // Current ramp
        case CONTROL_MODE_CURRENT_RAMP: {
            float max_step_size = ABS(CURRENT_MEASURE_PERIOD * UsrConfig.current_ramp_rate);
            float full_step     = Controller.input_current - Controller.cur_setpoint;
            float step          = CLAMP(full_step, -max_step_size, max_step_size);
            Controller.cur_setpoint += step;
        } break;

        // Velocity ramp
        case CONTROL_MODE_VELOCITY_RAMP: {
            float max_step_size = ABS(CURRENT_MEASURE_PERIOD * UsrConfig.velocity_ramp_rate);
            float full_step     = Controller.input_velocity - Controller.vel_setpoint;
            float step          = CLAMP(full_step, -max_step_size, max_step_size);
            Controller.vel_setpoint += step;
            Controller.cur_setpoint = UsrConfig.current_ff_gain * (step / CURRENT_MEASURE_PERIOD);

            // Target reached check
            if (StatuswordNew.status.target_reached == 0) {
                if (ABS(Controller.input_velocity - vel_meas) < UsrConfig.target_velcity_window) {
                    StatuswordNew.status.target_reached = 1;
                }
            }
        } break;

        // Position filter
        case CONTROL_MODE_POSITION_FILTER: {
            // 2nd order pos tracking filter
            float delta_pos = Controller.input_position - Controller.pos_setpoint;
            float delta_vel = Controller.input_velocity - Controller.vel_setpoint; // Vel error
            float accel     = Controller.input_pos_filter_kp * delta_pos
                          + Controller.input_pos_filter_ki * delta_vel;                  // Feedback
            Controller.cur_setpoint = UsrConfig.current_ff_gain * accel;                 // Accel
            Controller.vel_setpoint += CURRENT_MEASURE_PERIOD * accel;                   // Delta vel
            Controller.pos_setpoint += CURRENT_MEASURE_PERIOD * Controller.vel_setpoint; // Delta pos
        } break;

        // Position profile
        case CONTROL_MODE_POSITION_PROFILE: {
            if (Controller.input_updated) {
                Controller.input_updated = false;

                TRAJ_plan(Controller.input_position,
                          Controller.pos_setpoint,
                          Controller.vel_setpoint,
                          UsrConfig.profile_velocity, // Velocity
                          UsrConfig.profile_accel,    // Acceleration
                          UsrConfig.profile_decel);   // Deceleration
            }

            if (Traj.profile_done) {
                // Target reached check
                if (StatuswordNew.status.target_reached == 0) {
                    if (ABS(Controller.input_position - pos_meas) < UsrConfig.target_position_window) {
                        StatuswordNew.status.target_reached = 1;
                    }
                }
                break;
            }

            TRAJ_eval();
            Controller.pos_setpoint = Traj.Y;
            Controller.vel_setpoint = Traj.Yd;
            Controller.cur_setpoint = Traj.Ydd * UsrConfig.current_ff_gain;
        } break;

        default:
            break;
        }

        // Position control
        vel_des = Controller.vel_setpoint;
        if (Controller.ctrl_mode >= CONTROL_MODE_POSITION_FILTER) {
            float pos_err = Controller.pos_setpoint - pos_meas;
            vel_des += UsrConfig.pos_p_gain * pos_err;
        }
    } else {
        // Anticogging calib
        float pos_err = Controller.input_position - (Encoder.count_in_cpr / ENCODER_CPR_F);
        if (pos_err > +0.5f)
            pos_err -= 1.0f;
        if (pos_err < -0.5f)
            pos_err += 1.0f;
        vel_des = UsrConfig.pos_p_gain * pos_err;
    }

    // Velocity limiting
    vel_des = CLAMP(vel_des, -UsrConfig.velocity_limit, +UsrConfig.velocity_limit);

    // Velocity control
    float iq_set = Controller.cur_setpoint;

    float v_err = 0.0f;
    if (Controller.ctrl_mode >= CONTROL_MODE_VELOCITY_RAMP) {
        v_err = vel_des - vel_meas;
        iq_set += UsrConfig.vel_p_gain * v_err;

        // Velocity integral action before limiting
        iq_set += Controller.vel_integrator;
    }

    // Velocity limiting in current mode
    if (Controller.ctrl_mode < CONTROL_MODE_VELOCITY_RAMP) {
        float Imax = (+UsrConfig.velocity_limit - vel_meas) * UsrConfig.vel_p_gain;
        float Imin = (-UsrConfig.velocity_limit - vel_meas) * UsrConfig.vel_p_gain;
        iq_set     = CLAMP(iq_set, Imin, Imax);
    }

    // Anticogging
    if (UsrConfig.anticogging_enable && AnticoggingValid) {
        int16_t index = nearbyintf(COGGING_MAP_NUM * Encoder.count_in_cpr / ENCODER_CPR_F);
        if (index >= COGGING_MAP_NUM) {
            index = 0;
        }
        iq_set += pCoggingMap->map[index] / 5000.0f;
    }

    // Current limit
    bool limited = false;
    if (iq_set > +UsrConfig.current_limit) {
        limited = true;
        iq_set  = +UsrConfig.current_limit;
    }
    if (iq_set < -UsrConfig.current_limit) {
        limited = true;
        iq_set  = -UsrConfig.current_limit;
    }

    FOC_current(0, iq_set, phase_meas, phase_vel_meas);

    // Velocity integrator
    if (Controller.ctrl_mode < CONTROL_MODE_VELOCITY_RAMP) {
        // reset integral if not in use
        Controller.vel_integrator = 0.0f;
    } else {
        if (limited) {
            Controller.vel_integrator *= 0.99f;
        } else {
            Controller.vel_integrator += (UsrConfig.vel_i_gain * CURRENT_MEASURE_PERIOD) * v_err;
        }
    }
}
