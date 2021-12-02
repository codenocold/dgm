#include "systick.h"
#include "led.h"
#include "can.h"
#include "foc.h"
#include "uart.h"
#include "fsm.h"
#include "usr_config.h"
#include "drv8323.h"
#include "encoder.h"
#include "pwm_curr_fdbk.h"
#include <math.h>
#include "util.h"

#ifdef __DEBUG__
static void IO_init(void)
{
	// PA2 out pp for debug
    rcu_periph_clock_enable(RCU_GPIOA);
	gpio_bit_reset(GPIOA, GPIO_PIN_2);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
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
	
	if(0 != DRV8323_init()){
		DEBUG("\n\rDRV8323 init fail!\n\r");
		while(1);
	}
	
	if(0 == USR_CONFIG_read_config()){
		DEBUG("\n\rConfig loaded ok\n\r");
	}else{
		USR_CONFIG_set_default_config();
		DEBUG("\n\rConfig loaded faile set to default\n\r");
	}

	FOC_zero_current(&Foc);
	
	FSM_input(CMD_MENU);
	
    while(1){
		LED_loop();
		CAN_timeout_check_loop();
    }
}

void Error_Handler(void)
{
	__disable_irq();
	
	PWMC_switch_off_pwm();
	
	while(1){}
}
