#include "pwm_curr_fdbk.h"
#include "systick.h"
#include "drv8323.h"

uint16_t adc_value[1];

static void dma_config(void);
static void adc0_config(void);
static void adc1_config(void);

void PWMC_init(void)
{
	timer_parameter_struct timer_initpara;
	timer_oc_parameter_struct timer_ocinitpara;
	
	rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
	rcu_periph_clock_enable(RCU_TIMER0);

    /*configure PA8/PA9/PA10(TIMER0/CH0/CH1/CH2) as alternate function*/
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    timer_deinit(TIMER0);
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_UP;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = PWM_ARR - 1;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0, &timer_initpara);

    timer_channel_output_struct_para_init(&timer_ocinitpara);
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocinitpara);
    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocinitpara);
    timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocinitpara);
	timer_channel_output_config(TIMER0, TIMER_CH_3, &timer_ocinitpara);

    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, (PWM_ARR>>1));
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);

    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, (PWM_ARR>>1));
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);

    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, (PWM_ARR>>1));
    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);
	
	timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, (PWM_ARR-20));
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);
    
	timer_auto_reload_shadow_enable(TIMER0);

	dma_config();
	adc0_config();
	adc1_config();
	
	SYSTICK_delay_ms(10);
	
	timer_enable(TIMER0);
	timer_primary_output_config(TIMER0, ENABLE);
}

static void dma_config(void)
{
	/* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
	
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;
    
    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);
    
    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr = (uint32_t)(adc_value);
    dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;  
    dma_data_parameter.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number = 1;
    dma_data_parameter.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA0, DMA_CH0);
  
    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
}

static void adc0_config(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_ADC0);
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV2);
	
	gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_1);
	gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_1);
	
    adc_deinit(ADC0);
    adc_mode_config(ADC_DAUL_INSERTED_PARALLEL);

    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    
	/* ADC regular channel config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_9, ADC_SAMPLETIME_7POINT5);
	
	/* ADC inserted channel config */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 1);
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_1, ADC_SAMPLETIME_7POINT5);
	
    /* ADC regular trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_NONE); 
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
	
	/* ADC inserted trigger config */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_CH3); 
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
	
	/* ADC interrupt */
	nvic_irq_enable(ADC0_1_IRQn, 0, 0);
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);
    
    /* enable ADC interface */
    adc_enable(ADC0);
    SYSTICK_delay_ms(1);
	
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);

    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC0);
    
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
}

static void adc1_config(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_ADC1);
	
	gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0);
	
    adc_deinit(ADC1);

    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    
	/* ADC inserted channel config */
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL, 1);
    adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_7POINT5);
	
	/* ADC inserted trigger config */
    adc_external_trigger_source_config(ADC1, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_NONE); 
    adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, ENABLE);
    
    /* enable ADC interface */
    adc_enable(ADC1);
    SYSTICK_delay_ms(1);
	
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC1);
    
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC1, ADC_INSERTED_CHANNEL);
}

void PWMC_switch_on_pwm(void)
{
	/* Set all duty to 50% */
	TIMER_CH2CV(TIMER0) = (PWM_ARR>>1);
	TIMER_CH1CV(TIMER0) = (PWM_ARR>>1);
	TIMER_CH0CV(TIMER0) = (PWM_ARR>>1);
	
	/* wait for a new PWM period */
	timer_flag_clear(TIMER0, TIMER_FLAG_UP);
	while(RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)){};
	timer_flag_clear(TIMER0, TIMER_FLAG_UP);
	
	DRV8323_enable_gd();
}

void PWMC_switch_off_pwm(void)
{
	DRV8323_disable_gd();
	
	/* Set all duty to 50% */
	TIMER_CH2CV(TIMER0) = (PWM_ARR>>1);
	TIMER_CH1CV(TIMER0) = (PWM_ARR>>1);
	TIMER_CH0CV(TIMER0) = (PWM_ARR>>1);
	
	/* wait for a new PWM period */
	timer_flag_clear(TIMER0, TIMER_FLAG_UP);
	while(RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)){};
	timer_flag_clear(TIMER0, TIMER_FLAG_UP);
}
