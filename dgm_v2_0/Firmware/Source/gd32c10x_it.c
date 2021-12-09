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

#include "gd32c10x_it.h"
#include "main.h"
#include "can.h"
#include "foc.h"
#include "fsm.h"
#include "encoder.h"
#include "systick.h"
#include "pwm_curr_fdbk.h"

void NMI_Handler(void)
{
	Error_Handler();
}

void HardFault_Handler(void)
{
    Error_Handler();
}

void MemManage_Handler(void)
{
    Error_Handler();
}

void BusFault_Handler(void)
{
    Error_Handler();
}

void UsageFault_Handler(void)
{
    Error_Handler();
}

void SVC_Handler(void)
{
	Error_Handler();
}

void DebugMon_Handler(void)
{
	Error_Handler();
}

void PendSV_Handler(void)
{
	Error_Handler();
}

void SysTick_Handler(void)
{
	SysTickCnt++;
}

void CAN0_RX0_IRQHandler(void)
{
	CanFrame frame;
	while(CAN_receive(&frame)){
		CAN_rx_callback(&frame);
	}
}

void USART0_IRQHandler(void)
{
    if(usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){
		FSM_input(usart_data_receive(USART0));
    }
}

void ADC0_1_IRQHandler(void)
{	
	// Clear EOIC
	ADC_STAT(ADC0) &= ~((uint32_t)ADC_INT_FLAG_EOIC);
	
	ENCODER_sample();
	
	Foc.adc_vbus = adc_value[0];
	Foc.adc_phase_a = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
	Foc.adc_phase_b = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_1);
	Foc.adc_phase_c = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0);
	Foc.v_bus = 0.98f*Foc.v_bus + 0.02f*Foc.adc_vbus*V_SCALE;
	Foc.i_a = I_SCALE * (float)(Foc.adc_phase_a - Foc.adc_phase_a_offset);
	Foc.i_b = I_SCALE * (float)(Foc.adc_phase_b - Foc.adc_phase_b_offset);
	Foc.i_c = I_SCALE * (float)(Foc.adc_phase_c - Foc.adc_phase_c_offset);
	
	FSM_loop();
}
