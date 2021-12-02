#ifndef __PWM_CURR_FDBK_H__
#define __PWM_CURR_FDBK_H__

#include "gd32e10x.h"

#define SHUNT_RESISTENCE	0.001f								// Resistance of phase current sampling resistor
#define V_SCALE (19.0f * 3.3f / 4095.0f)     					// Bus volts per A/D Count (0.015311 V)
#define I_SCALE (3.3f / 4095.0f) / SHUNT_RESISTENCE / 40.0f		// Amps per A/D Count (0.020147 A)

#define PWM_ARR					3000       // 20KHz
#define DT						(1.0f/20000.0f)
#define CURRENT_MEASURE_HZ		20000

extern uint16_t adc_value[1];

void PWMC_init(void);
void PWMC_switch_on_pwm(void);
void PWMC_switch_off_pwm(void);

#endif
