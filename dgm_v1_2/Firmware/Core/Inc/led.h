#ifndef __LED_H__
#define __LED_H__

#include "main.h"

void LED_init(void);
void LED_loop(void);

static inline void LED_set(uint32_t state){
	if(state){
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
	}else{
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
	}
}

static inline uint32_t LED_get(void){
	return LL_GPIO_IsOutputPinSet(GPIOB, LL_GPIO_PIN_6);
}

static inline void LED_toggle(void){
	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_6);
}

#endif
