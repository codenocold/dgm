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

#include "foc.h"
#include "pwm_curr.h"
#include "usr_config.h"
#include "util.h"
#include <math.h>

tFOC Foc;

void FOC_init(void)
{
    Foc.v_bus      = 0;
    Foc.v_bus_filt = 0;
    Foc.i_q        = 0;
    Foc.i_q_filt   = 0;
    Foc.i_bus      = 0;
    Foc.i_bus_filt = 0;
    Foc.power_filt = 0;

    Foc.is_armed = false;
    FOC_update_current_ctrl_gain(UsrConfig.current_ctrl_bw);
}

void FOC_update_current_ctrl_gain(float bw)
{
    float bandwidth         = bw * M_2PI;
    Foc.current_ctrl_p_gain = UsrConfig.motor_phase_inductance * bandwidth;
    Foc.current_ctrl_i_gain = UsrConfig.motor_phase_resistance / UsrConfig.motor_phase_inductance;
}

void FOC_arm(void)
{
    if (Foc.is_armed) {
        return;
    }

    __disable_irq();

    Foc.i_q        = 0;
    Foc.i_q_filt   = 0;
    Foc.i_bus      = 0;
    Foc.i_bus_filt = 0;
    Foc.power_filt = 0;

    Foc.current_ctrl_integral_d = 0;
    Foc.current_ctrl_integral_q = 0;

    PWMC_TurnOnLowSides();

    Foc.is_armed = true;

    __enable_irq();
}

void FOC_disarm(void)
{
    if (!Foc.is_armed) {
        return;
    }

    __disable_irq();

    Foc.i_q        = 0;
    Foc.i_q_filt   = 0;
    Foc.i_bus      = 0;
    Foc.i_bus_filt = 0;
    Foc.power_filt = 0;

    PWMC_SwitchOffPWM();

    Foc.is_armed = false;

    __enable_irq();
}

void FOC_voltage(float Vd_set, float Vq_set, float phase)
{
    float v_to_mod = 1.5f / Foc.v_bus_filt; // = 1.0f / (Foc.v_bus_filt * 2.0f / 3.0f);
    float mod_vd   = Vd_set * v_to_mod;
    float mod_vq   = Vq_set * v_to_mod;

    // Vector modulation saturation, lock integrator if saturated
    float factor = 0.9f * SQRT3_BY_2 / sqrtf(SQ(mod_vd) + SQ(mod_vq));
    if (factor < 1.0f) {
        mod_vd *= factor;
        mod_vq *= factor;
    }

    // Inverse park transform
    float alpha, beta;
    float pwm_phase = phase;
    inverse_park(mod_vd, mod_vq, pwm_phase, &alpha, &beta);

    // SVM
    if (0 == svm(alpha, beta, &Foc.dtc_a, &Foc.dtc_b, &Foc.dtc_c)) {
        set_a_duty((uint16_t) (Foc.dtc_a * (float) HALF_PWM_PERIOD_CYCLES));
        set_b_duty((uint16_t) (Foc.dtc_b * (float) HALF_PWM_PERIOD_CYCLES));
        set_c_duty((uint16_t) (Foc.dtc_c * (float) HALF_PWM_PERIOD_CYCLES));
    }
}

void FOC_current(float Id_set, float Iq_set, float phase, float phase_vel)
{
    // Clarke transform
    float i_alpha, i_beta;
    clarke_transform(Foc.i_a, Foc.i_b, Foc.i_c, &i_alpha, &i_beta);

    // Park transform
    float i_d, i_q;
    park_transform(i_alpha, i_beta, phase, &i_d, &i_q);

    // Current PI control
    float i_d_err = Id_set - i_d;
    float i_q_err = Iq_set - i_q;
    float v_d     = i_d_err * Foc.current_ctrl_p_gain + Foc.current_ctrl_integral_d;
    float v_q     = i_q_err * Foc.current_ctrl_p_gain + Foc.current_ctrl_integral_q;

    // voltage normalize = 1/(2/3*v_bus)
    float v_to_mod = 1.5f / Foc.v_bus_filt;
    float mod_vd   = v_d * v_to_mod;
    float mod_vq   = v_q * v_to_mod;

    // Vector modulation saturation, lock integrator if saturated
    float factor = 0.9f * SQRT3_BY_2 / sqrtf(SQ(mod_vd) + SQ(mod_vq));
    if (factor < 1.0f) {
        mod_vd *= factor;
        mod_vq *= factor;
        Foc.current_ctrl_integral_d *= 0.99f;
        Foc.current_ctrl_integral_q *= 0.99f;
    } else {
        Foc.current_ctrl_integral_d += i_d_err * (Foc.current_ctrl_i_gain * CURRENT_MEASURE_PERIOD);
        Foc.current_ctrl_integral_q += i_q_err * (Foc.current_ctrl_i_gain * CURRENT_MEASURE_PERIOD);
    }

    // Inverse park transform
    float alpha, beta;
    float pwm_phase = phase + phase_vel * CURRENT_MEASURE_PERIOD;
    inverse_park(mod_vd, mod_vq, pwm_phase, &alpha, &beta);

    // SVM
    if (0 == svm(alpha, beta, &Foc.dtc_a, &Foc.dtc_b, &Foc.dtc_c)) {
        set_a_duty((uint16_t) (Foc.dtc_a * (float) HALF_PWM_PERIOD_CYCLES));
        set_b_duty((uint16_t) (Foc.dtc_b * (float) HALF_PWM_PERIOD_CYCLES));
        set_c_duty((uint16_t) (Foc.dtc_c * (float) HALF_PWM_PERIOD_CYCLES));
    }

    // used for report
    Foc.i_q = i_q;
    UTILS_LP_FAST(Foc.i_q_filt, Foc.i_q, 0.01f);
    Foc.i_d = i_d;
    UTILS_LP_FAST(Foc.i_d_filt, Foc.i_d, 0.01f);
    Foc.i_bus = (mod_vd * i_d + mod_vq * i_q);
    UTILS_LP_FAST(Foc.i_bus_filt, Foc.i_bus, 0.01f);
    Foc.power_filt = Foc.v_bus_filt * Foc.i_bus_filt;
}
