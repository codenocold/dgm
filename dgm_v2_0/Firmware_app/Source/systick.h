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

#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include "main.h"

#define TIM_US_COUNT	(uint32_t)TIMER_CNT(TIMER1)

extern volatile uint32_t SysTickCnt;

void SYSTICK_init(void);
uint32_t SYSTICK_get_tick(void);
uint32_t SYSTICK_get_ms_since(uint32_t tick);
void SYSTICK_delay_us(uint32_t uS);
void SYSTICK_delay_ms(uint32_t mS);

#endif
