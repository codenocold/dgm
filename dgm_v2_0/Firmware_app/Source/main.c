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

#include "main.h"
#include "can.h"
#include "foc.h"
#include "mc_task.h"
#include "encoder.h"
#include "pwm_curr.h"
#include "controller.h"
#include "usr_config.h"
#include "anticogging.h"

volatile uint32_t SystickCount = 0;

static void RCU_init(void)
{
    rcu_periph_clock_enable(RCU_AF);
    
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    
    /* enable TIMER clock */
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_TIMER1);
    
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    // ADC clock = 30M (APB2 = 120M, ADC clock MAX 40M)
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
    
    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
    
    /* enable CAN0 clock */
    rcu_periph_clock_enable(RCU_CAN0);
    
    /* enable SPI0 clock */
    rcu_periph_clock_enable(RCU_SPI0);
    
    /* enable uart0 clock */
    rcu_periph_clock_enable(RCU_USART0);
}

static void GPIO_init(void)
{
    // LED ACT PC13
    gpio_bit_reset(GPIOC, GPIO_PIN_13);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_13);
    
    // IO PA15
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
    gpio_bit_reset(GPIOA, GPIO_PIN_15);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
    
    // ADC
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0);        // IA   - PA0 - ADC01_IN0
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_1);        // IB   - PA1 - ADC01_IN1
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_2);        // IC   - PA2 - ADC01_IN2
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_3);        // VBUS - PA3 - ADC01_IN3
    
    // FOC PWM
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);    // AH PA8
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);    // BH PA9
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);    // CH PA10
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);    // AL PB13
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);    // BL PB14
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);    // CL PB13
    
    /* configure CAN0 GPIO */
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_11);    // CAN_RX PA11
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);    // CAN_TX PA12
    
    /* SPI0 */
    // ENC_nCS PA4
    gpio_bit_set(GPIOA, GPIO_PIN_4);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    // SCK/PA5, MISO/PA6, MOSI/PA7
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    
    /* UART0 */
    gpio_pin_remap_config(GPIO_USART0_REMAP, ENABLE);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);           // UART Tx PB6
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_7);     // UART Rx PB7
}

static void SPI0_init(void)
{
    spi_parameter_struct spi_init_struct;
    
    /* deinitilize SPI and the parameters */
    spi_i2s_deinit(SPI0);
    spi_struct_para_init(&spi_init_struct);

    /* SPI1 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_8;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct);
    
    spi_enable(SPI0);
}

static void DMA0_init(void)
{
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;
    
    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);
    
    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr = (uint32_t)(adc_vbus);
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

static void ADC0_init(void)
{
    adc_deinit(ADC0);
    
    adc_mode_config(ADC_DAUL_INSERTED_PARALLEL);
    
    adc_special_function_config(ADC0, ADC_SCAN_MODE, DISABLE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
    adc_special_function_config(ADC0, ADC_INSERTED_CHANNEL_AUTO, DISABLE);
    adc_resolution_config(ADC0, ADC_RESOLUTION_12B);
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    
    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC0);
    
    /* ADC inserted channel config */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 1);
    // IA - PA0 - ADC01_IN0
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_7POINT5);
    
    /* ADC inserted trigger config */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_CH3); 
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    
    /* ADC regular channel config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);
    // VBUS - PA3 - ADC01_IN3
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_3, ADC_SAMPLETIME_7POINT5);
    
    /* ADC regular trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_NONE); 
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
}

static void ADC1_init(void)
{
    adc_deinit(ADC1);

    adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, DISABLE);
    adc_special_function_config(ADC1, ADC_INSERTED_CHANNEL_AUTO, DISABLE);
    adc_resolution_config(ADC1, ADC_RESOLUTION_12B);
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    
    /* ADC inserted channel config */
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL, 2);
    // IB - PA1 - ADC01_IN1
    adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_1, ADC_SAMPLETIME_7POINT5);
    // IC - PA2 - ADC01_IN2
    adc_inserted_channel_config(ADC1, 1, ADC_CHANNEL_2, ADC_SAMPLETIME_7POINT5);
    
    /* ADC inserted trigger config */
    adc_external_trigger_source_config(ADC1, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_NONE);
    adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, ENABLE);
}

static void TIMER0_init(void)
{
    timer_parameter_struct timer_initpara;
    timer_oc_parameter_struct timer_ocinitpara;
    timer_break_parameter_struct timer_breakpara;

    timer_deinit(TIMER0);
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_UP;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = HALF_PWM_PERIOD_CYCLES;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0, &timer_initpara);

    timer_channel_output_struct_para_init(&timer_ocinitpara);
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_ENABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocinitpara);
    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocinitpara);
    timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocinitpara);
    timer_channel_output_config(TIMER0, TIMER_CH_3, &timer_ocinitpara);

    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);

    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);

    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, 0);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);
    
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, (HALF_PWM_PERIOD_CYCLES-5));
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);
    
    /* configure TIMER break function */
    timer_break_struct_para_init(&timer_breakpara);
    /* automatic output enable, break, dead time and lock configuration*/
    timer_breakpara.runoffstate      = TIMER_ROS_STATE_DISABLE;
    timer_breakpara.ideloffstate     = TIMER_IOS_STATE_DISABLE;
    timer_breakpara.deadtime         = 0;    // use FD6288Q hardware deadtime 200ns
    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_LOW;
    timer_breakpara.outputautostate  = TIMER_OUTAUTO_DISABLE;
    timer_breakpara.protectmode      = TIMER_CCHP_PROT_OFF;
    timer_breakpara.breakstate       = TIMER_BREAK_DISABLE;
    timer_break_config(TIMER0, &timer_breakpara);
}

static void TIMER1_init(void)
{
    // TIMER1CLK = SystemCoreClock/60000 = 2KHz, the period is 500 us
    timer_parameter_struct timer_initpara;

    timer_deinit(TIMER1);
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 60000-1;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);
    
    /* enable the TIMER interrupt */
    timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER1, TIMER_INT_UP);
    
    timer_enable(TIMER1);
}

void UART0_init(uint32_t baudrate)
{
    /* UART0 configure */
    usart_deinit(USART0);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_baudrate_set(USART0, baudrate);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}

static void LOCK_pins(void)
{
    // LED ACT PC13
    gpio_pin_lock(GPIOC, GPIO_PIN_13);
    
    // IO PA15
    gpio_pin_lock(GPIOA, GPIO_PIN_15);
    
    // ADC
    gpio_pin_lock(GPIOA, GPIO_PIN_0);
    gpio_pin_lock(GPIOA, GPIO_PIN_1);
    gpio_pin_lock(GPIOA, GPIO_PIN_2);
    gpio_pin_lock(GPIOA, GPIO_PIN_3);
    
    // PWM
    gpio_pin_lock(GPIOA, GPIO_PIN_8);
    gpio_pin_lock(GPIOA, GPIO_PIN_9);
    gpio_pin_lock(GPIOA, GPIO_PIN_10);
    gpio_pin_lock(GPIOB, GPIO_PIN_13);
    gpio_pin_lock(GPIOB, GPIO_PIN_14);
    gpio_pin_lock(GPIOB, GPIO_PIN_15);
    
    // CAN0
    gpio_pin_lock(GPIOA, GPIO_PIN_11);
    gpio_pin_lock(GPIOA, GPIO_PIN_12);
    
    // SPI0
    gpio_pin_lock(GPIOA, GPIO_PIN_4);
    gpio_pin_lock(GPIOA, GPIO_PIN_5);
    gpio_pin_lock(GPIOA, GPIO_PIN_6);
    gpio_pin_lock(GPIOA, GPIO_PIN_7);
    
    // UART0
    gpio_pin_lock(GPIOB, GPIO_PIN_6);
    gpio_pin_lock(GPIOB, GPIO_PIN_7);
}

static void NVIC_init(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    
    // ADC01 insert convert complete interrupt NVIC
    NVIC_SetPriority(ADC0_1_IRQn, 0);
    NVIC_EnableIRQ(ADC0_1_IRQn);
    
    // TIM1 interrupt NVIC
    NVIC_SetPriority(TIMER1_IRQn, 1);
    NVIC_EnableIRQ(TIMER1_IRQn);
    
    // CAN0 Rx interrupt NVIC
    NVIC_SetPriority(CAN0_RX0_IRQn, 2);
    NVIC_EnableIRQ(CAN0_RX0_IRQn);
}

static void WATCH_DOG_init(void)
{
    /* enable IRC40K */
    rcu_osci_on(RCU_IRC40K);
    
    /* wait till IRC40K is ready */
    while(SUCCESS != rcu_osci_stab_wait(RCU_IRC40K)){}

    /* confiure FWDGT counter clock: 40KHz(IRC40K) / 32 = 5000 Hz */
    fwdgt_config(50, FWDGT_PSC_DIV8);    // 10 ms
}

static void CAN0_init(int baudrate)
{
    can_parameter_struct can_parameter;

    can_interrupt_disable(CAN0, CAN_INTEN_RFNEIE0);
    can_deinit(CAN0);
    
    // Reset CAN peripheral
    CAN_CTL(CAN0) |= CAN_CTL_SWRST;
    while((CAN_CTL(CAN0) & CAN_CTL_SWRST) != 0);        // reset bit is set to zero after reset
    while((CAN_STAT(CAN0) & CAN_STAT_SLPWS) == 0);        // should be in sleep mode after reset
    
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = ENABLE;
    can_parameter.time_triggered = DISABLE;
    
    int brp, seg1, seg2, sjw;
    
    // Set timing
    switch((tCanBaudrate)baudrate){
        case CAN_BAUDRATE_250K:
            // 87.5% when BaudRate <= 500K
            brp  = 15;
            seg1 = 13;
            seg2 = 2;
            sjw  = 1;
            break;
        
        case CAN_BAUDRATE_500K:
            // 87.5% when BaudRate <= 500K
            brp  = 15;
            seg1 = 6;
            seg2 = 1;
            sjw  = 1;
            break;
        
        case CAN_BAUDRATE_800K:
            // 80% when BaudRate > 500K
            brp  = 15;
            seg1 = 3;
            seg2 = 1;
            sjw  = 1;
            break;
        
        case CAN_BAUDRATE_1000K:
            // 75% when BaudRate > 800K
            brp  = 15;
            seg1 = 2;
            seg2 = 1;
            sjw  = 1;
            break;
        
        default:
            // 500K
            // 87.5% when BaudRate <= 500K
            brp  = 15;
            seg1 = 6;
            seg2 = 1;
            sjw  = 1;
            break;
    }

    can_parameter.prescaler = brp;
    can_parameter.resync_jump_width = sjw - 1;
    can_parameter.time_segment_1 = seg1 - 1;
    can_parameter.time_segment_2 = seg2 - 1;
    can_init(CAN0, &can_parameter);

    /* initialize filter */
    can1_filter_start_bank(14);
    can_filter_mask_mode_init(0, 0, CAN_EXTENDED_FIFO0, 0);
    can_filter_mask_mode_init(0, 0, CAN_STANDARD_FIFO0, 0);

    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
}

int main(void)
{
    __disable_irq();

    RCU_init();
    GPIO_init();
    SPI0_init();
    DMA0_init();
    ADC0_init();
    ADC1_init();
    TIMER0_init();
    TIMER1_init();
    UART0_init(115200);
    LOCK_pins();
    NVIC_init();
    WATCH_DOG_init();
    
    if(0 == USR_CONFIG_read_config()){
        printf("Config loaded\n\r");
    }else{
        USR_CONFIG_set_default_config();
        printf("Config loaded faile set to default\n\r");
    }
    
    if(0 == USR_CONFIG_read_cogging_map()){
        AnticoggingValid = true;
        printf("Cogging map loaded\n\r\n\r");
    }else{
        USR_CONFIG_set_default_cogging_map();
        printf("Cogging map loaded faile set to default\n\r\n\r");
    }
    
    CAN_set_node_id(UsrConfig.node_id);
    CAN0_init(UsrConfig.can_baudrate);
    
    MCT_init();
    FOC_init();
    PWMC_init();
    ENCODER_init();
    CONTROLLER_init();
    
    fwdgt_enable();
    __enable_irq();

    // wait voltage stable
    for(uint8_t i=0, j=0; i<250; i++){
        if(Foc.v_bus_filt > 20){
            if(++j > 20){
                break;
            }
        }
        delay_ms(2);
    }
    
    if(PWMC_CurrentReadingPolarization() != 0){
        StatuswordNew.errors.adc_selftest_fatal = 1;
    }
    
    MCT_set_state(IDLE);

    while(1) {
        MCT_low_priority_task();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    
    /* Main PWM Output Disable */
    timer_primary_output_config(TIMER0, DISABLE);
    
    while(1){}
}

void delay_ms(const uint16_t ms)
{
    volatile uint32_t i = ms * 17200;
    while (i-- > 0) {
        __NOP();
    }
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}
