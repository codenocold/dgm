#include "uart.h"

void UART_init(uint32_t baudrate)
{
	rcu_periph_clock_enable(RCU_AF);
	rcu_periph_clock_enable(RCU_GPIOB);
	gpio_pin_remap_config(GPIO_USART0_REMAP, ENABLE);
	rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

    /* USART configure */
    usart_deinit(USART0);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_baudrate_set(USART0, baudrate);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
	
	nvic_irq_enable(USART0_IRQn, 3, 0);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
	
    usart_enable(USART0);
}

int fputc(int ch, FILE *f)
{
	usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
	return ch;
}
