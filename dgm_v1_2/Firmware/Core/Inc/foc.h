#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include <stdbool.h>

typedef struct {
    uint16_t adc_phase_a, adc_phase_b, adc_vbus;            // Raw ADC Values
	float v_bus;                                            // DC link voltage
    float i_a, i_b, i_c;                                    // Phase currents
	float i_d_filt, i_q_filt, i_bus_filt;                   // D/Q currents
	float current_ctrl_integral_d, current_ctrl_integral_q;	// Current error integrals
	int adc_phase_a_offset, adc_phase_b_offset;            	// ADC offsets
} FOCStruct;

extern FOCStruct Foc;

void FOC_zero_current(FOCStruct *foc);
void FOC_arm(void);
void FOC_disarm(void);
bool FOC_is_armed(void);
void FOC_reset(FOCStruct *foc);
void FOC_current(FOCStruct *foc, float Id_des, float Iq_des, float I_phase, float pwm_phase);

#endif
