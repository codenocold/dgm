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

void NMI_Handler(void)
{
	while(1){};
}

void HardFault_Handler(void)
{
    while(1){};
}

void MemManage_Handler(void)
{
    while(1){};
}

void BusFault_Handler(void)
{
    while(1){};
}

void UsageFault_Handler(void)
{
    while(1){};
}

void SVC_Handler(void)
{
	while(1){};
}

void DebugMon_Handler(void)
{
	while(1){};
}

void PendSV_Handler(void)
{
	while(1){};
}
