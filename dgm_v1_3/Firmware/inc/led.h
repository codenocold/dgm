#ifndef __LED_H__
#define __LED_H__

#include "gd32e10x.h"

void LED_init(void);

static inline void LED_set(uint32_t state){
	if(state){
		GPIO_BOP(GPIOB) = (uint32_t)GPIO_PIN_8;
	}else{
		GPIO_BC(GPIOB) = (uint32_t)GPIO_PIN_8;
	}
}

static inline uint32_t LED_get(void){
	return (GPIO_OCTL(GPIOB)&(GPIO_PIN_8));
}

static inline void LED_toggle(void){
	if(LED_get()){
		LED_set(0);
	}else{
		LED_set(1);
	}
}

void LED_loop(void);

#endif
