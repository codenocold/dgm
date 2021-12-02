#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include "gd32e10x.h"

#define TIM_US_COUNT	(uint32_t)TIMER_CNT(TIMER1)

extern volatile uint32_t SysTickCnt;

void SYSTICK_init(void);
uint32_t SYSTICK_get_tick(void);
uint32_t SYSTICK_get_ms_since(uint32_t tick);
void SYSTICK_delay_us(uint32_t uS);
void SYSTICK_delay_ms(uint32_t mS);

#endif
