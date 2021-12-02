#include "led.h"
#include "fsm.h"
#include "systick.h"

void LED_init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void LED_loop(void)
{
	static uint32_t tick = 0;
	static uint16_t cnt = 0;
	
	// 1000Hz
	if(SYSTICK_get_ms_since(tick) < 1){
		return;
	}
	tick = SYSTICK_get_tick();
	
	switch(FSM_get_stat()){
		case FS_MENU_MODE:
			if(cnt == 0){
				LED_set(1);
			}else if(cnt == 100){
				LED_set(0);
			}else if(cnt >= 1500){
				cnt = 0xFFFF;
			}
			break;
		
		case FS_MOTOR_MODE:
			if(cnt == 0){
				LED_set(1);
			}else if(cnt == 100){
				LED_set(0);
			}else if(cnt == 200){
				LED_set(1);
			}else if(cnt == 300){
				LED_set(0);
			}else if(cnt >= 1500){
				cnt = 0xFFFF;
			}
			break;
		
		case FS_CALIBRATION_MODE:
			if(cnt == 0){
				LED_set(1);
			}else if(cnt == 100){
				LED_set(0);
			}else if(cnt == 200){
				LED_set(1);
			}else if(cnt == 300){
				LED_set(0);
			}else if(cnt == 400){
				LED_set(1);
			}else if(cnt == 500){
				LED_set(0);
			}else if(cnt >= 1500){
				cnt = 0xFFFF;
			}
			break;
		
		case FS_UART_SETUP:
		case FS_UART_ENCODER:
			LED_set(1);
			break;
		
		default:
			break;
	}
	
	cnt ++;
}
