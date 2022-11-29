/*
	Copyright 2021 codenocold 1107795287@qq.com
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

typedef struct {
	uint16_t adc_vbus;
    uint16_t adc_phase_a, adc_phase_b, adc_phase_c;
	float v_bus;                                            // DC link voltage
    float i_a, i_b, i_c;                                    // Phase currents
	float dtc_a, dtc_b, dtc_c;
	float i_d_filt, i_q_filt, i_bus_filt;                   // D/Q currents
	
	float current_ctrl_integral_d, current_ctrl_integral_q;	// Current error integrals
	int adc_phase_a_offset;
	int adc_phase_b_offset;
	int adc_phase_c_offset;
} FOCStruct;

extern FOCStruct Foc;

void FOC_zero_current(FOCStruct *foc);
void FOC_update_current_gain(void);
void FOC_arm(void);
void FOC_disarm(void);
bool FOC_is_armed(void);
void FOC_reset(FOCStruct *foc);
void apply_voltage_timings(float vbus, float v_d, float v_q, float pwm_phase);
void FOC_current(FOCStruct *foc, float Id_des, float Iq_des, float I_phase, float pwm_phase);

#endif
