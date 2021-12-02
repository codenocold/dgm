#include "pwm_curr_fdbk.h"
#include "hw_config.h"
#include "systick.h"

#define TIMxCCER_MASK_CH123     ((uint16_t)(LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3))

static void ADC1_init(void);
static void ADC2_init(void);

void PWMC_init(void)
{
	LL_TIM_InitTypeDef TIM_InitStruct = {0};
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
	
	TIM_InitStruct.Prescaler = 0;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP;
	TIM_InitStruct.Autoreload = PWM_ARR - 1;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	TIM_InitStruct.RepetitionCounter = 1;
	LL_TIM_Init(TIM1, &TIM_InitStruct);
	LL_TIM_EnableARRPreload(TIM1);

	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = (PWM_ARR>>1);
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
	TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
	
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);

	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);

	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);

	TIM_OC_InitStruct.CompareValue = PWM_ARR - 10;
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
	
	// Triger ADC set
	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);
	LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM1);
	
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	/**TIM1 GPIO Configuration
	PA8     ------> TIM1_CH1
	PA9     ------> TIM1_CH2
	PA10     ------> TIM1_CH3
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	ADC1_init();
	ADC2_init();
	
	// Wait ADC stable
	SYSTICK_delay_ms(10);
	
	// ADC1 calibration
	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED );
	while ( LL_ADC_IsCalibrationOnGoing(ADC1) == 1u){}

	// ADC2 calibration
	LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED );
	while ( LL_ADC_IsCalibrationOnGoing(ADC2) == 1u){}

	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);
	
	/* ADC interrupt Init */
	NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
	NVIC_EnableIRQ(ADC1_2_IRQn);
	LL_ADC_EnableIT_JEOS(ADC1);
	
	LL_ADC_INJ_StartConversion(ADC1);
	LL_ADC_INJ_StartConversion(ADC2);
		
	LL_TIM_EnableCounter(TIM1);
}

static void ADC1_init(void)
{
	LL_ADC_InitTypeDef ADC_InitStruct = {0};
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
	LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
	LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	/**ADC1 GPIO Configuration
	PB1   ------> ADC1_IN12
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/** Common config
	*/
	ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
	LL_ADC_Init(ADC1, &ADC_InitStruct);
	ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
	ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
	LL_ADC_SetGainCompensation(ADC1, 0);
	LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
	ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV4;
	ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
	LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
	ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
	ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_DISABLE;
	ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
	ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
	LL_ADC_INJ_Init(ADC1, &ADC_INJ_InitStruct);
	LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_DISABLE);
	LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
	LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

	/* Disable ADC deep power down (enabled by default after reset state) */
	LL_ADC_DisableDeepPowerDown(ADC1);
	/* Enable ADC internal voltage regulator */
	LL_ADC_EnableInternalRegulator(ADC1);
	/* Delay for ADC internal voltage regulator stabilization. */
	/* Compute number of CPU cycles to wait for, from delay in us. */
	/* Note: Variable divided by 2 to compensate partially */
	/* CPU processing cycles (depends on compilation optimization). */
	/* Note: If system core clock frequency is below 200kHz, wait time */
	/* is only a few CPU processing cycles. */
	uint32_t wait_loop_index;
	wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
	while(wait_loop_index != 0){
		wait_loop_index--;
	}
	
	/** Configure Injected Channel
	*/
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_6CYCLES_5);
	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SINGLE_ENDED);
	LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_12);
}

static void ADC2_init(void)
{
	LL_ADC_InitTypeDef ADC_InitStruct = {0};
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
	LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	/**ADC2 GPIO Configuration
	PC5   ------> ADC2_IN11
	PB2   ------> ADC2_IN12
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/** Common config
	*/
	ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
	LL_ADC_Init(ADC2, &ADC_InitStruct);
	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
	ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
	LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
	LL_ADC_SetGainCompensation(ADC2, 0);
	LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
	ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
	ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_DISABLE;
	ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
	ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
	LL_ADC_INJ_Init(ADC2, &ADC_INJ_InitStruct);
	LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_DISABLE);
	LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
	LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

	/* Disable ADC deep power down (enabled by default after reset state) */
	LL_ADC_DisableDeepPowerDown(ADC2);
	/* Enable ADC internal voltage regulator */
	LL_ADC_EnableInternalRegulator(ADC2);
	/* Delay for ADC internal voltage regulator stabilization. */
	/* Compute number of CPU cycles to wait for, from delay in us. */
	/* Note: Variable divided by 2 to compensate partially */
	/* CPU processing cycles (depends on compilation optimization). */
	/* Note: If system core clock frequency is below 200kHz, wait time */
	/* is only a few CPU processing cycles. */
	uint32_t wait_loop_index;
	wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
	while(wait_loop_index != 0){
		wait_loop_index--;
	}
	
	/** Configure Regular Channel
	*/
	LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_11);
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_6CYCLES_5);
	LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_11, LL_ADC_SINGLE_ENDED);

	/** Configure Injected Channel
	*/
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_6CYCLES_5);
	LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_12, LL_ADC_SINGLE_ENDED);
	LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_12);
}

void PWMC_switch_on_pwm(void)
{
	/* Set all duty to 50% */
	TIM1->CCR3 = (PWM_ARR>>1);
	TIM1->CCR2 = (PWM_ARR>>1);
	TIM1->CCR1 = (PWM_ARR>>1);
	
	/* wait for a new PWM period */
	LL_TIM_ClearFlag_UPDATE(TIM1);
	while ( LL_TIM_IsActiveFlag_UPDATE(TIM1) == 0u )
	{}
	LL_TIM_ClearFlag_UPDATE(TIM1);

	/* Main PWM Output Enable */
	LL_TIM_CC_EnableChannel(TIM1, TIMxCCER_MASK_CH123);
	LL_TIM_EnableAllOutputs(TIM1);
}

void PWMC_switch_off_pwm(void)
{
	/* Main PWM Output Disable */
	LL_TIM_DisableAllOutputs(TIM1);
	
	/* wait for a new PWM period */
	LL_TIM_ClearFlag_UPDATE(TIM1);
	while ( LL_TIM_IsActiveFlag_UPDATE(TIM1) == 0u )
	{}
	LL_TIM_ClearFlag_UPDATE(TIM1);
}
