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

#include "pwm_curr.h"

uint16_t adc_vbus[1];
int16_t phase_a_adc_offset = 0;
int16_t phase_b_adc_offset = 0;
int16_t phase_c_adc_offset = 0;

void PWMC_init(void)
{
    /* Disable ADC interrupt */
    adc_interrupt_disable(ADC0, ADC_INT_EOIC);
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);

    /* enable ADC0 */
    adc_enable(ADC0);
    /* Wait ADC0 startup */
    delay_ms(1);
    /* ADC0 calibration */
    adc_calibration_enable(ADC0);

    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
    
    /* ADC0 inject convert complete interrupt */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);

    /* enable ADC1 */
    adc_enable(ADC1);
    /* Wait ADC1 startup */
    delay_ms(1);
    /* ADC1 calibration */
    adc_calibration_enable(ADC1);
    
    /* Hold TIMER0 counter when core is halted */
    dbg_periph_enable(DBG_TIMER0_HOLD);
    
    /* Enable TIMER0 counter */
    timer_enable(TIMER0);
    
    timer_repetition_value_config(TIMER0, 1);
    
    /* Set all duty to 50% */
    set_a_duty(((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2));
    set_b_duty(((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2));
    set_c_duty(((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2));
    
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_DISABLE);
    
    /* Main PWM Output Enable */
    timer_primary_output_config(TIMER0, ENABLE);
}

void PWMC_SwitchOnPWM(void)
{
    /* Set all duty to 50% */
    set_a_duty(((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2));
    set_b_duty(((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2));
    set_c_duty(((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2));

    /* wait for a new PWM period */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    while(RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)){};
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);

    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_ENABLE);
}

void PWMC_SwitchOffPWM(void)
{
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_DISABLE);
    
    /* wait for a new PWM period */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    while(RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)){};
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
}

void PWMC_TurnOnLowSides(void)
{
    /* Set all duty to 0% */
    set_a_duty(0);
    set_b_duty(0);
    set_c_duty(0);

    /* wait for a new PWM period */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    while(RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)){};
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);

    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_ENABLE);
}

int PWMC_CurrentReadingPolarization(void)
{
    int i = 0;
    int adc_sum_a = 0;
    int adc_sum_b = 0;
    int adc_sum_c = 0;
    
    /* Clear Update Flag */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    /* Wait until next update */
    while(RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)){};
    /* Clear Update Flag */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    
    while(i < 64) {
        if(timer_flag_get(TIMER0, TIMER_FLAG_UP) == SET){
            timer_flag_clear(TIMER0, TIMER_FLAG_UP);
            
            i ++;
            adc_sum_a += READ_IPHASE_A_ADC();
            adc_sum_b += READ_IPHASE_B_ADC();
            adc_sum_c += READ_IPHASE_C_ADC();
        }
    }
    
    phase_a_adc_offset = adc_sum_a/i;
    phase_b_adc_offset = adc_sum_b/i;
    phase_c_adc_offset = adc_sum_c/i;
    
    // offset check
    i = 0;
    const int Vout = 2122;
    const int check_threshold = 200;
    if(phase_a_adc_offset > (Vout + check_threshold) || phase_a_adc_offset < (Vout - check_threshold)){
        i = -1;
    }
    if(phase_b_adc_offset > (Vout + check_threshold) || phase_b_adc_offset < (Vout - check_threshold)){
        i = -1;
    }
    if(phase_c_adc_offset > (Vout + check_threshold) || phase_c_adc_offset < (Vout - check_threshold)){
        i = -1;
    }
    
    return i;
}
