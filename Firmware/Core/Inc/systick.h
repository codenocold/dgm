#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include "main.h"

extern __IO uint32_t SysTickCnt;

void SYSTICK_init(void);
uint32_t SYSTICK_get_tick(void);
uint32_t SYSTICK_get_ms_since(uint32_t tick);
void SYSTICK_delay_us(uint32_t uS);
void SYSTICK_delay_ms(uint32_t mS);

#endif
