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

#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32c10x.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

extern volatile uint32_t SystickCount;

// LED ACT
#define LED_ACT_SET()   GPIO_BOP(GPIOC) = (uint32_t) GPIO_PIN_13;
#define LED_ACT_RESET() GPIO_BC(GPIOC) = (uint32_t) GPIO_PIN_13;
#define LED_ACT_GET()   (GPIO_OCTL(GPIOC) & (GPIO_PIN_13))

static inline void LED_ACT_TOGGLE(void)
{
    if (LED_ACT_GET()) {
        LED_ACT_RESET();
    } else {
        LED_ACT_SET();
    }
}

// SPI NCS
#define NCS_SET()   GPIO_BOP(GPIOA) = (uint32_t) GPIO_PIN_4;
#define NCS_RESET() GPIO_BC(GPIOA) = (uint32_t) GPIO_PIN_4;

// IO
#define IO_SET()    GPIO_BOP(GPIOA) = (uint32_t) GPIO_PIN_15
#define IO_RESET()  GPIO_BC(GPIOA) = (uint32_t) GPIO_PIN_15
#define IO_GET()    (GPIO_OCTL(GPIOA) & (GPIO_PIN_15))

static inline void IO_TOGGLE(void)
{
    if (IO_GET()) {
        IO_RESET();
    } else {
        IO_SET();
    }
}

/* FLASH MAP ---------------------------------------------*/
#define PAGE_SIZE            ((uint32_t) 0x400U) // 1KB

#define APP_MAIN_ADDR        ((uint32_t) (0x8000000 + 0 * PAGE_SIZE))  // Page 0
#define APP_BACK_ADDR        ((uint32_t) (0x8000000 + 50 * PAGE_SIZE)) // Page 50
#define APP_MAX_SIZE         ((uint32_t) (50 * PAGE_SIZE))             // 50KB

#define BOOTLOADER_ADDR      ((uint32_t) (0x8000000 + 100 * PAGE_SIZE)) // Page 100
#define BOOTLOADER_MAX_SIZE  ((uint32_t) (10 * PAGE_SIZE))              // 10KB

#define USR_CONFIG_ADDR      ((uint32_t) (0x8000000 + 110 * PAGE_SIZE)) // Page 110
#define USR_CONFIG_MAX_SIZE  ((uint32_t) (5 * PAGE_SIZE))               // 5KB

#define COGGING_MAP_ADDR     ((uint32_t) (0x8000000 + 115 * PAGE_SIZE)) // Page 115
#define COGGING_MAP_MAX_SIZE ((uint32_t) (10 * PAGE_SIZE))              // 10KB

/* Exported functions prototypes ---------------------------------------------*/
static inline void watch_dog_feed(void)
{
    FWDGT_CTL = FWDGT_KEY_RELOAD;
}

static inline uint32_t get_ms_since(uint32_t tick)
{
    return (uint32_t) ((SystickCount - tick) / 2U);
}

void Error_Handler(void);
void delay_ms(const uint16_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
