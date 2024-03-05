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

#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include <stdbool.h>

typedef struct sFOC
{
    bool is_armed;

    float v_bus, v_bus_filt, i_a, i_b, i_c;

    float i_q, i_q_filt, i_d, i_d_filt, i_bus, i_bus_filt, power_filt;
    float dtc_a, dtc_b, dtc_c;

    float current_ctrl_p_gain, current_ctrl_i_gain;
    float current_ctrl_integral_d, current_ctrl_integral_q;
} tFOC;

extern tFOC Foc;

void FOC_init(void);
void FOC_update_current_ctrl_gain(float bw);
void FOC_arm(void);
void FOC_disarm(void);
void FOC_voltage(float Vd_set, float Vq_set, float phase);
void FOC_current(float Id_set, float Iq_set, float phase, float phase_vel);

#endif
