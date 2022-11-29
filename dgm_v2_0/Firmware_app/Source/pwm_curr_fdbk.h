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

#ifndef __PWM_CURR_FDBK_H__
#define __PWM_CURR_FDBK_H__

#include "main.h"

#define SHUNT_RESISTENCE	0.001f								// Resistance of phase current sampling resistor
#define V_SCALE (19.0f * 3.3f / 4095.0f)     					// Bus volts per A/D Count (0.015311 V)
#define I_SCALE ((3.3f / 4095.0f) / SHUNT_RESISTENCE / 30.482f)	// Amps per A/D Count

#define PWM_ARR					3000       // 20KHz
#define DT						(1.0f/20000.0f)
#define CURRENT_MEASURE_HZ		20000

extern uint16_t adc_value[1];

void PWMC_init(void);
void PWMC_switch_on_pwm(void);
void PWMC_switch_off_pwm(void);

#endif
