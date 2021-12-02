/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include "gd32e10x.h"

//#define __DEBUG__

#ifdef __DEBUG__
	#define DEBUG(...)		printf(__VA_ARGS__);
#else
	#define DEBUG(...)
#endif

#define IO_SET()		GPIO_BOP(GPIOA) = (uint32_t)GPIO_PIN_2
#define IO_RESET()		GPIO_BC(GPIOA) = (uint32_t)GPIO_PIN_2
#define IO_GET()		(GPIO_OCTL(GPIOA)&(GPIO_PIN_2))
static inline void IO_TOGGLE(void){if(IO_GET()){IO_RESET();}else{IO_SET();}}

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
