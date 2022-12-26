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
#include <math.h>
#include "util.h"
#include "usr_config.h"
#include "pwm_curr.h"

tFOC Foc;

void FOC_init(void)
{
    Foc.v_bus = 0;
    Foc.v_bus_filt = 0;
    Foc.i_q = 0;
    Foc.i_q_filt = 0;
    Foc.i_bus = 0;
    Foc.i_bus_filt = 0;
    Foc.power_filt = 0;
    
    Foc.is_armed = false;
    FOC_update_current_ctrl_gain(UsrConfig.current_ctrl_bw);
}

void FOC_update_current_ctrl_gain(float bw)
{
    float bandwidth = MIN(bw, 0.25f * PWM_FREQUENCY);
    Foc.current_ctrl_p_gain = UsrConfig.motor_phase_inductance * bandwidth;
    Foc.current_ctrl_i_gain = UsrConfig.motor_phase_resistance * bandwidth;
}

void FOC_arm(void)
{
    if(Foc.is_armed){
        return;
    }
    
    __disable_irq();
    
    Foc.i_q = 0;
    Foc.i_q_filt = 0;
    Foc.i_bus = 0;
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
    if(!Foc.is_armed){
        return;
    }
    
    __disable_irq();
    
    Foc.i_q = 0;
    Foc.i_q_filt = 0;
    Foc.i_bus = 0;
    Foc.i_bus_filt = 0;
    Foc.power_filt = 0;
    
    PWMC_SwitchOffPWM();
    
    Foc.is_armed = false;
    
    __enable_irq();
}

void FOC_voltage(float Vd_set, float Vq_set, float phase)
{
    // Clarke transform
    float i_alpha, i_beta;
    clarke_transform(Foc.i_a, Foc.i_b, Foc.i_c, &i_alpha, &i_beta);
    
    // Park transform
    float i_d, i_q;
    park_transform(i_alpha, i_beta, phase, &i_d, &i_q);
    
    // Used for report
    Foc.i_q = i_q;
    UTILS_LP_FAST(Foc.i_q_filt, Foc.i_q, 0.01f);
    Foc.i_d = i_d;
    UTILS_LP_FAST(Foc.i_d_filt, Foc.i_d, 0.01f);
    
    // Modulation
    float V_to_mod = 1.0f / (Foc.v_bus_filt * 2.0f / 3.0f);
    float mod_d = V_to_mod * Vd_set;
    float mod_q = V_to_mod * Vq_set;
    
    // Vector modulation saturation, lock integrator if saturated
    float mod_scalefactor = 0.9f * SQRT3_BY_2 / sqrtf(SQ(mod_d) + SQ(mod_q));
    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
    }
    
    // Inverse park transform
    float mod_alpha;
    float mod_beta;
    inverse_park(mod_d, mod_q, phase, &mod_alpha, &mod_beta);

    // SVM
    if(0 == svm(mod_alpha, mod_beta, &Foc.dtc_a, &Foc.dtc_b, &Foc.dtc_c)){
        set_a_duty((uint16_t)(Foc.dtc_a * (float)HALF_PWM_PERIOD_CYCLES));
        set_b_duty((uint16_t)(Foc.dtc_b * (float)HALF_PWM_PERIOD_CYCLES));
        set_c_duty((uint16_t)(Foc.dtc_c * (float)HALF_PWM_PERIOD_CYCLES));
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
    
    float mod_to_V = Foc.v_bus_filt * 2.0f / 3.0f;
    float V_to_mod = 1.0f / mod_to_V;
    
    // Apply PI control
    float Ierr_d = Id_set - i_d;
    float Ierr_q = Iq_set - i_q;
    float mod_d = V_to_mod * (Foc.current_ctrl_integral_d + Ierr_d * Foc.current_ctrl_p_gain);
    float mod_q = V_to_mod * (Foc.current_ctrl_integral_q + Ierr_q * Foc.current_ctrl_p_gain);

    // Vector modulation saturation, lock integrator if saturated
    float mod_scalefactor = 0.9f * SQRT3_BY_2 / sqrtf(SQ(mod_d) + SQ(mod_q));
    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        Foc.current_ctrl_integral_d *= 0.99f;
        Foc.current_ctrl_integral_q *= 0.99f;
    } else {
        Foc.current_ctrl_integral_d += Ierr_d * (Foc.current_ctrl_i_gain * CURRENT_MEASURE_PERIOD);
        Foc.current_ctrl_integral_q += Ierr_q * (Foc.current_ctrl_i_gain * CURRENT_MEASURE_PERIOD);
    }

    // Inverse park transform
    float mod_alpha, mod_beta;
    float pwm_phase = phase + phase_vel * CURRENT_MEASURE_PERIOD;
    inverse_park(mod_d, mod_q, pwm_phase, &mod_alpha, &mod_beta);
    
    // Used for report
    Foc.i_q = i_q;
    UTILS_LP_FAST(Foc.i_q_filt, Foc.i_q, 0.01f);
    Foc.i_d = i_d;
    UTILS_LP_FAST(Foc.i_d_filt, Foc.i_d, 0.01f);
    Foc.i_bus = (mod_d * i_d + mod_q * i_q);
    UTILS_LP_FAST(Foc.i_bus_filt, Foc.i_bus, 0.01f);
    Foc.power_filt = Foc.v_bus_filt * Foc.i_bus_filt;

    // SVM
    if(0 == svm(mod_alpha, mod_beta, &Foc.dtc_a, &Foc.dtc_b, &Foc.dtc_c)){
        set_a_duty((uint16_t)(Foc.dtc_a * (float)HALF_PWM_PERIOD_CYCLES));
        set_b_duty((uint16_t)(Foc.dtc_b * (float)HALF_PWM_PERIOD_CYCLES));
        set_c_duty((uint16_t)(Foc.dtc_c * (float)HALF_PWM_PERIOD_CYCLES));
    }
}
