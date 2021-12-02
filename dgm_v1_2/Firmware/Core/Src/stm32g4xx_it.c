#include "stm32g4xx_it.h"
#include "main.h"
#include "systick.h"
#include "fdcan.h"
#include "fsm.h"
#include "foc.h"
#include "hw_config.h"
#include "encoder.h"

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
	Error_Handler();
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
	Error_Handler();
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
	Error_Handler();
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
	Error_Handler();
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
	Error_Handler();
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{

}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
	
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{

}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
	SysTickCnt ++;
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_RXNE_RXFNE(USART2)){
		FSM_input(LL_USART_ReceiveData8(USART2));
	}
}

void FDCAN1_IT0_IRQHandler(void)
{
	#define FDCAN_RX_FIFO0_MASK (FDCAN_IR_RF0L | FDCAN_IR_RF0F | FDCAN_IR_RF0N)
	
	uint32_t RxFifo0ITs;
	RxFifo0ITs = hfdcan1.Instance->IR & FDCAN_RX_FIFO0_MASK;
	RxFifo0ITs &= hfdcan1.Instance->IE;
	
	if (RxFifo0ITs != 0U){
		/* Clear the Rx FIFO 0 flags */
		__HAL_FDCAN_CLEAR_FLAG(&hfdcan1, RxFifo0ITs);
		
		while(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0)){
			FDCAN_rx_callback();
		}
	}
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
	// Start VBUS Sample ADC
	LL_ADC_REG_StartConversion(ADC2);
	
	ENCODER_sample(DT);		// sample position sensor
	
	Foc.adc_vbus = LL_ADC_REG_ReadConversionData12(ADC2);
	Foc.adc_phase_a = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
	Foc.adc_phase_b = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
	Foc.i_a = I_SCALE * (float)(Foc.adc_phase_a_offset - Foc.adc_phase_a);
	Foc.i_b = I_SCALE * (float)(Foc.adc_phase_b_offset - Foc.adc_phase_b);
	Foc.i_c = -Foc.i_a - Foc.i_b;
	Foc.v_bus = 0.9f*Foc.v_bus + 0.1f*Foc.adc_vbus*V_SCALE;		// filter the dc link voltage measurement
	
	FSM_loop();
	
	LL_ADC_ClearFlag_JEOS(ADC1);
}
