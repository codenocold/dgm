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

    switch (UsrConfig.control_mode) {
    case CONTROL_MODE_TORQUE_RAMP:
        Controller.input_torque = Controller.input_torque_buffer;
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
    StatuswordOld.status.target_reached = 0;
}

void CONTROLLER_init(void)
{
    CONTROLLER_update_input_pos_filter_gain(UsrConfig.position_filter_bw);
}

void CONTROLLER_update_input_pos_filter_gain(float bw)
{
    float bandwidth                = MIN(bw, 0.25f * PWM_FREQUENCY);
    Controller.input_pos_filter_ki = 2.0f * bandwidth;
    Controller.input_pos_filter_kp = 0.25f * SQ(Controller.input_pos_filter_ki);
}

void CONTROLLER_reset(void)
{
    float pos_meas = Encoder.pos;

    if (MCT_get_state() == ANTICOGGING) {
        pos_meas = (Encoder.count_in_cpr / ENCODER_CPR_F);
    }

    Controller.input_position = pos_meas;
    Controller.input_velocity = 0.0f;
    Controller.input_torque   = 0.0f;

    Controller.input_position_buffer = pos_meas;
    Controller.input_velocity_buffer = 0.0f;
    Controller.input_torque_buffer   = 0.0f;

    Controller.pos_setpoint    = pos_meas;
    Controller.vel_setpoint    = 0.0f;
    Controller.torque_setpoint = 0.0f;

    Controller.vel_integrator_torque = 0;

    Controller.input_updated = false;
    Traj.profile_done        = true;
}

void CONTROLLER_loop(void)
{
    float       vel_des;
    const float pos_meas       = Encoder.pos;
    const float vel_meas       = Encoder.vel;
    const float phase_meas     = Encoder.phase;
    const float phase_vel_meas = Encoder.phase_vel;

    if (MCT_get_state() == RUN) {
        switch (UsrConfig.control_mode) {
        // Torque ramp
        case CONTROL_MODE_TORQUE_RAMP: {
            float max_step_size = ABS(CURRENT_MEASURE_PERIOD * UsrConfig.torque_ramp_rate);
            float full_step     = Controller.input_torque - Controller.torque_setpoint;
            float step          = CLAMP(full_step, -max_step_size, max_step_size);
            Controller.torque_setpoint += step;
        } break;

        // Velocity ramp
        case CONTROL_MODE_VELOCITY_RAMP: {
            float max_step_size = ABS(CURRENT_MEASURE_PERIOD * UsrConfig.velocity_ramp_rate);
            float full_step     = Controller.input_velocity - Controller.vel_setpoint;
            float step          = CLAMP(full_step, -max_step_size, max_step_size);
            Controller.vel_setpoint += step;
            Controller.torque_setpoint = (step / CURRENT_MEASURE_PERIOD) * UsrConfig.inertia;

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
            Controller.torque_setpoint = accel * UsrConfig.inertia;                      // Accel
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
            Controller.pos_setpoint    = Traj.Y;
            Controller.vel_setpoint    = Traj.Yd;
            Controller.torque_setpoint = Traj.Ydd * UsrConfig.inertia;
        } break;

        default:
            break;
        }

        // Position control
        vel_des = Controller.vel_setpoint;
        if (UsrConfig.control_mode >= CONTROL_MODE_POSITION_FILTER) {
            float pos_err = Controller.pos_setpoint - pos_meas;
            vel_des += UsrConfig.pos_gain * pos_err;
        }
    } else {
        // Anticogging calib
        float pos_err = Controller.input_position - (Encoder.count_in_cpr / ENCODER_CPR_F);
        if (pos_err > +0.5f)
            pos_err -= 1.0f;
        if (pos_err < -0.5f)
            pos_err += 1.0f;
        vel_des = UsrConfig.pos_gain * pos_err;
    }

    // Velocity limiting
    vel_des = CLAMP(vel_des, -UsrConfig.velocity_limit, +UsrConfig.velocity_limit);

    // Velocity control
    float torque = Controller.torque_setpoint;

    float v_err = 0.0f;
    if (UsrConfig.control_mode >= CONTROL_MODE_VELOCITY_RAMP) {
        v_err = vel_des - vel_meas;
        torque += UsrConfig.vel_gain * v_err;

        // Velocity integral action before limiting
        torque += Controller.vel_integrator_torque;
    }

    // Velocity limiting in torque mode
    if (UsrConfig.control_mode < CONTROL_MODE_VELOCITY_RAMP) {
        float Tmax = (+UsrConfig.velocity_limit - vel_meas) * UsrConfig.vel_gain;
        float Tmin = (-UsrConfig.velocity_limit - vel_meas) * UsrConfig.vel_gain;
        torque     = CLAMP(torque, Tmin, Tmax);
    }

    // Anticogging
    if (UsrConfig.anticogging_enable && AnticoggingValid) {
        int16_t index = nearbyintf(COGGING_MAP_NUM * Encoder.count_in_cpr / ENCODER_CPR_F);
        if (index >= COGGING_MAP_NUM) {
            index = 0;
        }
        torque += pCoggingMap->map[index] / 5000.0f;
    }

    // Torque limiting
    bool  limited    = false;
    float max_torque = UsrConfig.current_limit * UsrConfig.torque_constant;
    if (torque > +max_torque) {
        limited = true;
        torque  = +max_torque;
    }
    if (torque < -max_torque) {
        limited = true;
        torque  = -max_torque;
    }

    if (limited) {
        StatuswordNew.status.current_limit_active = 1;
    } else {
        if (ABS(torque) < 0.9f * max_torque) {
            StatuswordNew.status.current_limit_active = 0;
        }
    }

    // Velocity integrator
    if (UsrConfig.control_mode < CONTROL_MODE_VELOCITY_RAMP) {
        // reset integral if not in use
        Controller.vel_integrator_torque = 0.0f;
    } else {
        if (limited) {
            Controller.vel_integrator_torque *= 0.99f;
        } else {
            Controller.vel_integrator_torque += (UsrConfig.vel_integrator_gain * CURRENT_MEASURE_PERIOD) * v_err;
        }
    }

    float iq_set = torque / UsrConfig.torque_constant;
    FOC_current(0, iq_set, phase_meas, phase_vel_meas);
}
