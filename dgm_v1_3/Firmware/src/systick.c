#include "systick.h"

#define SYSTICK_US_TICK		(SystemCoreClock / 1000000U)
#define SYSTICK_MS_TICK		(SystemCoreClock / 1000U)

volatile uint32_t SysTickCnt = 0;

void SYSTICK_init(void)
{
	/* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SYSTICK_MS_TICK)){
        /* capture error */
        while (1){}
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x02U);
	
	/* ----------------------------------------------------------------------------
    TIMER1 Configuration: 
    TIMER1CLK = SystemCoreClock/120 = 1MHz
    ---------------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;
    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 120-1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xFFFF;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);
    
    timer_enable(TIMER1);
}

uint32_t SYSTICK_get_tick(void)
{
	return SysTickCnt;
}

uint32_t SYSTICK_get_ms_since(uint32_t tick)
{
	return (uint32_t)(SysTickCnt - tick);
}

void SYSTICK_delay_us(uint32_t uS)
{
    uint32_t elapsed    = 0;
    uint32_t last_count = SysTick->VAL;

    for (;;) {
        uint32_t current_count = SysTick->VAL;
        uint32_t elapsed_uS;

        /* measure the time elapsed since the last time we checked */
		if(last_count > current_count){
			elapsed += last_count - current_count;
		}else if(last_count < current_count){
			elapsed += last_count + (SYSTICK_MS_TICK - current_count);
		}
        
        last_count = current_count;

        /* convert to microseconds */
        elapsed_uS = elapsed / SYSTICK_US_TICK;
        if (elapsed_uS >= uS) {
            break;
        }

        /* reduce the delay by the elapsed time */
        uS -= elapsed_uS;

        /* keep fractional microseconds for the next iteration */
        elapsed %= SYSTICK_US_TICK;
    }
}

void SYSTICK_delay_ms(uint32_t mS)
{
    while (mS--) {
        SYSTICK_delay_us(1000);
    }
}
