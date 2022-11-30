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

#ifndef __PWM_CURR_H__
#define __PWM_CURR_H__

#include "main.h"

#define PWM_FREQUENCY   			20000
#define CURRENT_MEASURE_HZ          PWM_FREQUENCY
#define CURRENT_MEASURE_PERIOD      (float)(1.0f/(float)CURRENT_MEASURE_HZ)

#define TIMER0_CLK_MHz  			120
#define PWM_PERIOD_CYCLES        	(uint16_t)((TIMER0_CLK_MHz*(uint32_t)1000000u/((uint32_t)(PWM_FREQUENCY)))&0xFFFE)
#define HALF_PWM_PERIOD_CYCLES   	(uint16_t)(PWM_PERIOD_CYCLES/2U)

#define SHUNT_RESISTENCE			(0.001f)
#define V_SCALE 					((float)(19.0f * 3.3f / 4095.0f))
#define I_SCALE 					((float)((3.3f / 4095.0f) / SHUNT_RESISTENCE / 30.482f))

#define READ_IPHASE_A_ADC()			((uint16_t)(ADC_IDATA0(ADC0)))
#define READ_IPHASE_B_ADC()			((uint16_t)(ADC_IDATA0(ADC1)))
#define READ_IPHASE_C_ADC()			((uint16_t)(ADC_IDATA1(ADC1)))

extern uint16_t adc_vbus[1];
extern int16_t phase_a_adc_offset;
extern int16_t phase_b_adc_offset;
extern int16_t phase_c_adc_offset;

static inline float read_vbus(void) {
	return (float)(adc_vbus[0]) * V_SCALE;
}

static inline float read_iphase_a(void) {
	return (float)(READ_IPHASE_A_ADC() - phase_a_adc_offset) * I_SCALE;
}

static inline float read_iphase_b(void) {
	return (float)(READ_IPHASE_B_ADC() - phase_b_adc_offset) * I_SCALE;
}

static inline float read_iphase_c(void) {
	return (float)(READ_IPHASE_C_ADC() - phase_c_adc_offset) * I_SCALE;
}

static inline void set_a_duty(uint32_t duty)            { TIMER_CH0CV(TIMER0) = duty; }
static inline void set_b_duty(uint32_t duty)            { TIMER_CH1CV(TIMER0) = duty; }
static inline void set_c_duty(uint32_t duty)            { TIMER_CH2CV(TIMER0) = duty; }

void PWMC_init(void);
void PWMC_SwitchOnPWM(void);
void PWMC_SwitchOffPWM(void);
void PWMC_TurnOnLowSides(void);
int PWMC_CurrentReadingPolarization(void);

#endif
