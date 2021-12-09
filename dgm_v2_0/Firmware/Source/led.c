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

#include "led.h"
#include "fsm.h"
#include "systick.h"

void LED_init(void)
{
	// LED ACT init
    rcu_periph_clock_enable(RCU_GPIOC);
	gpio_bit_reset(GPIOC, GPIO_PIN_13);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_13);
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
			LED_set(1);
			break;
		
		default:
			break;
	}
	
	cnt ++;
}
