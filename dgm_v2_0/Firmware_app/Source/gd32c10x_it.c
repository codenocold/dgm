/*
	Copyright 2021 codenocold codenocold@qq.com
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

#include "gd32c10x_it.h"
#include "can.h"
#include "mc_task.h"

void NMI_Handler(void)
{
	Error_Handler();
}

void HardFault_Handler(void)
{
    Error_Handler();
}

void MemManage_Handler(void)
{
    Error_Handler();
}

void BusFault_Handler(void)
{
    Error_Handler();
}

void UsageFault_Handler(void)
{
    Error_Handler();
}

void SVC_Handler(void)
{
	Error_Handler();
}

void DebugMon_Handler(void)
{
	Error_Handler();
}

void PendSV_Handler(void)
{
	Error_Handler();
}

void ADC0_1_IRQHandler(void)
{	
	// Clear EOIC
	ADC_STAT(ADC0) &= ~((uint32_t)ADC_INT_FLAG_EOIC);

	MCT_high_frequency_task();
}

void TIMER1_IRQHandler(void)
{
	/* clear update interrupt bit */
	TIMER_INTF(TIMER1) = (~(uint32_t)TIMER_INT_FLAG_UP);
	
	MCT_safety_task();
	
	SystickCount ++;
}

void CAN0_RX0_IRQHandler(void)
{
	CAN_receive_callback();
}
