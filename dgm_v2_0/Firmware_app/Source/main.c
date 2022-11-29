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

#include "systick.h"
#include "led.h"
#include "can.h"
#include "foc.h"
#include "uart.h"
#include "fsm.h"
#include "usr_config.h"
#include "encoder.h"
#include "pwm_curr_fdbk.h"
#include <math.h>
#include "util.h"
#include "anticogging.h"

#ifdef __DEBUG__
static void IO_init(void)
{
	// PA15 out pp for debug
    rcu_periph_clock_enable(RCU_GPIOA);
	gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
	gpio_bit_reset(GPIOA, GPIO_PIN_15);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
}
#endif

int main(void)
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
	
	SYSTICK_init();
	LED_init();
	CAN_init();
#ifdef __DEBUG__
	IO_init();
	UART_init(115200);
#endif
	ENCODER_init();
	PWMC_init();
	
	DEBUG("\n\r");
	if(0 == USR_CONFIG_read_config()){
		DEBUG("Config loaded\n\r");
	}else{
		USR_CONFIG_set_default_config();
		DEBUG("Config loaded faile set to default\n\r");
	}
	
	if(0 == USR_CONFIG_read_cogging_map()){
		AnticoggingValid = true;
		DEBUG("Cogging map loaded\n\r\n\r");
	}else{
		USR_CONFIG_set_default_cogging_map();
		DEBUG("Cogging map loaded faile set to default\n\r\n\r");
	}

	FOC_zero_current(&Foc);
	
	FSM_input(CMD_MENU);
	
    while(1){
		LED_loop();
		CAN_timeout_check_loop();
		ANTICOGGING_loop(&Controller);
    }
}

void Error_Handler(void)
{
	__disable_irq();
	
	PWMC_switch_off_pwm();
	
	while(1){}
}
