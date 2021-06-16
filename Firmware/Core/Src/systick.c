#include "systick.h"

#define SYSTICK_US_TICK		170
#define SYSTICK_MS_TICK		170000

__IO uint32_t SysTickCnt = 0;

void SYSTICK_init(void)
{
	uint32_t prioritygroup;
	
	SysTick->LOAD  = (uint32_t)(SYSTICK_MS_TICK - 1UL);               /* set reload register */
	
	/* set Priority for Systick Interrupt */
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(prioritygroup, 2, 0));

	SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
				     SysTick_CTRL_TICKINT_Msk   |
				     SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */
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
