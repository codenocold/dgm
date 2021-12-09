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

#ifndef __LED_H__
#define __LED_H__

#include "main.h"

void LED_init(void);

static inline void LED_set(uint32_t state){
	if(state){
		GPIO_BOP(GPIOC) = (uint32_t)GPIO_PIN_13;
	}else{
		GPIO_BC(GPIOC) = (uint32_t)GPIO_PIN_13;
	}
}

static inline uint32_t LED_get(void){
	return (GPIO_OCTL(GPIOC)&(GPIO_PIN_13));
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
