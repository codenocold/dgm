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

#ifndef __PWM_CURR_H__
#define __PWM_CURR_H__

#include "main.h"
#include <math.h>

#define PWM_FREQUENCY          20000
#define CURRENT_MEASURE_HZ     PWM_FREQUENCY
#define CURRENT_MEASURE_PERIOD (float) (1.0f / (float) CURRENT_MEASURE_HZ)

#define TIMER0_CLK_MHz         120
#define PWM_PERIOD_CYCLES      (uint16_t) ((TIMER0_CLK_MHz * (uint32_t) 1000000u / ((uint32_t) (PWM_FREQUENCY))) & 0xFFFE)
#define HALF_PWM_PERIOD_CYCLES (uint16_t) (PWM_PERIOD_CYCLES / 2U)

#define SHUNT_RESISTENCE       (0.01f)
#define V_SCALE                ((float) (16.0f * 3.3f / 4095.0f))
#define I_SCALE                ((float) ((3.3f / 4095.0f) / SHUNT_RESISTENCE / 15.0f))

#define READ_IPHASE_A_ADC()    ((uint16_t) (ADC_IDATA0(ADC0)))
#define READ_IPHASE_B_ADC()    ((uint16_t) (ADC_IDATA0(ADC1)))

extern uint16_t adc_buff[3];
extern int16_t  phase_a_adc_offset;
extern int16_t  phase_b_adc_offset;

static const int16_t temp_table[]
    = {277, 277, 227, 201, 184, 172, 162, 154, 148, 142, 137, 132, 128, 125, 121, 118, 115, 113, 110, 108, 105, 103,
       101, 99,  98,  96,  94,  93,  91,  90,  88,  87,  86,  84,  83,  82,  81,  80,  79,  77,  76,  75,  74,  73,
       72,  72,  71,  70,  69,  68,  67,  66,  66,  65,  64,  63,  63,  62,  61,  60,  60,  59,  58,  58,  57,  56,
       56,  55,  54,  54,  53,  53,  52,  51,  51,  50,  50,  49,  48,  48,  47,  47,  46,  46,  45,  45,  44,  44,
       43,  43,  42,  42,  41,  41,  40,  40,  39,  39,  38,  38,  37,  37,  36,  36,  35,  35,  35,  34,  34,  33,
       33,  32,  32,  31,  31,  31,  30,  30,  29,  29,  28,  28,  28,  27,  27,  26,  26,  25,  25,  25,  24,  24,
       23,  23,  22,  22,  22,  21,  21,  20,  20,  20,  19,  19,  18,  18,  18,  17,  17,  16,  16,  16,  15,  15,
       14,  14,  13,  13,  13,  12,  12,  11,  11,  11,  10,  10,  9,   9,   8,   8,   8,   7,   7,   6,   6,   5,
       5,   5,   4,   4,   3,   3,   2,   2,   1,   1,   1,   0,   -0,  -1,  -1,  -2,  -2,  -3,  -3,  -4,  -4,  -5,
       -5,  -6,  -6,  -7,  -7,  -8,  -8,  -9,  -9,  -10, -11, -11, -12, -12, -13, -13, -14, -15, -15, -16, -17, -17,
       -18, -19, -19, -20, -21, -21, -22, -23, -24, -25, -26, -26, -27, -28, -29, -30, -31, -32, -33, -35, -36, -37,
       -38, -40, -41, -43, -45, -47, -49, -51, -54, -57, -61, -65, -72, -82};

static inline float read_vbus(void)
{
    return (float) (adc_buff[0]) * V_SCALE;
}

static inline int read_drv_temp(void)
{
    return temp_table[adc_buff[1] >> 4];
}

static inline int read_ntc_temp(void)
{
    return temp_table[adc_buff[2] >> 4];
}

static inline float read_iphase_a(void)
{
    return (float) (READ_IPHASE_A_ADC() - phase_a_adc_offset) * I_SCALE;
}

static inline float read_iphase_b(void)
{
    return (float) (READ_IPHASE_B_ADC() - phase_b_adc_offset) * I_SCALE;
}

static inline void set_a_duty(uint32_t duty)
{
    TIMER_CH2CV(TIMER0) = duty;
}

static inline void set_b_duty(uint32_t duty)
{
    TIMER_CH1CV(TIMER0) = duty;
}

static inline void set_c_duty(uint32_t duty)
{
    TIMER_CH0CV(TIMER0) = duty;
}

void PWMC_init(void);
void PWMC_SwitchOnPWM(void);
void PWMC_SwitchOffPWM(void);
void PWMC_TurnOnLowSides(void);
int  PWMC_CurrentReadingPolarization(void);

#endif
