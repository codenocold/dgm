#include "gd32e10x_it.h"
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
	
	ENCODER_sample(DT);
	
	Foc.adc_vbus = adc_value[0];
	Foc.adc_phase_a = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
	Foc.adc_phase_b = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0);
	Foc.i_a = I_SCALE * (float)(Foc.adc_phase_a_offset - Foc.adc_phase_a);
	Foc.i_b = I_SCALE * (float)(Foc.adc_phase_b_offset - Foc.adc_phase_b);
	Foc.i_c = -Foc.i_a - Foc.i_b;
	Foc.v_bus = 0.98f*Foc.v_bus + 0.02f*Foc.adc_vbus*V_SCALE;
	
	FSM_loop();
}
