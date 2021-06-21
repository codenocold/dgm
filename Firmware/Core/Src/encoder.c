#include "encoder.h"
#include "systick.h"
#include "util.h"
#include "usr_config.h"
#include "arm_math.h"

tEncoder Encoder;

static void SPI2_init(void);

void ENCODER_init(void)
{
	SPI2_init();
	
	Encoder.turns = 0;
}

static void SPI2_init(void)
{
	LL_SPI_InitTypeDef SPI_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	
	// PB12   ------> ENC_nCS
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
	GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/**SPI2 GPIO Configuration
	PB13   ------> SPI2_SCK
	PB14   ------> SPI2_MISO
	PB15   ------> SPI2_MOSI
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 7;
	LL_SPI_Init(SPI2, &SPI_InitStruct);
	LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
	LL_SPI_DisableNSSPulseMgt(SPI2);
	
	/* Enable SPI peripheral */
	LL_SPI_Enable(SPI2);
}

static int spi_transmit_receive(uint16_t data_in, uint16_t *data_out)
{
	int state = 0;
	*data_out = 0;
	uint32_t timeout_cnt;
	static const uint32_t timeout_cnt_num = 10000;
	
	/* Wait until TXE flag is set to send data */
	timeout_cnt = 0;
	while(!LL_SPI_IsActiveFlag_TXE(SPI2)){
		timeout_cnt ++;
		if(timeout_cnt > timeout_cnt_num){
			state = -1;
			break;
		}
	}
	
	/* Transmit data in 16 Bit mode */
	LL_SPI_TransmitData16(SPI2, data_in);
	
	/* Check BSY flag */
	timeout_cnt = 0;
	while(LL_SPI_IsActiveFlag_BSY(SPI2)){
		timeout_cnt ++;
		if(timeout_cnt > timeout_cnt_num){
			state = -2;
			break;
		}
	}
	
	/* Check RXNE flag */
	timeout_cnt = 0;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI2)){
		timeout_cnt ++;
		if(timeout_cnt > timeout_cnt_num){
			state = -3;
			break;
		}
	}
	
	// Read 16-Bits in the data register
	*data_out = LL_SPI_ReceiveData16(SPI2);
	
	return state;
}

/*
static uint8_t MA730_read_reg(uint8_t addr)
{
	uint16_t txData;
	uint16_t rxData;
	
	txData = (0x40 | addr) << 8;
	rxData = 0;

	// CS
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);
	
	spi_transmit_receive(txData, &rxData);
	
	// NCS
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
	
	SYSTICK_delay_ms(20);
	
	// CS
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);
	
	txData = 0;
	rxData = 0;
	spi_transmit_receive(txData, &rxData);
	
	// NCS
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
	
	SYSTICK_delay_ms(10);

	return (uint8_t)(rxData>>8);
}

static uint8_t MA730_write_reg(uint8_t addr, uint8_t data)
{
	uint16_t txData;
	uint16_t rxData;
	
	txData = ((0x80 | addr) << 8) | data;
	rxData = 0;

	// CS
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);
	
	spi_transmit_receive(txData, &rxData);
	
	// NCS
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
	
	SYSTICK_delay_ms(20);
	
	// CS
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);
	
	txData = 0;
	rxData = 0;
	spi_transmit_receive(txData, &rxData);
	
	// NCS
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
	
	SYSTICK_delay_ms(10);

	return (uint8_t)(rxData>>8);
}
*/

static inline uint16_t MA730_read_raw(void)
{
	uint16_t txData = 0;
	uint16_t rxData;

	// CS
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);
	
	spi_transmit_receive(txData, &rxData);
	
	// NCS
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);

	return (rxData>>2);
}

void ENCODER_sample(float dt)
{
	if(UsrConfig.calib_valid){
		// dir swap
		if(UsrConfig.encoder_dir_rev){
			Encoder.raw = ENCODER_CPR - MA730_read_raw();
		}else{
			Encoder.raw = MA730_read_raw();
		}
		
		// offset compensation
		int off_1 = UsrConfig.offset_lut[Encoder.raw>>7];
		int off_2 = UsrConfig.offset_lut[((Encoder.raw>>7)+1)%128];
		int off_interp = off_1 + ((off_2 - off_1) * (Encoder.raw - ((Encoder.raw>>7)<<7)) >> 7);        // Interpolate between lookup table entries
		int cnt = Encoder.raw - off_interp;                                     					// Correct for nonlinearity with lookup table from calibration
		if(cnt > ENCODER_CPR){
			cnt -= ENCODER_CPR;
		}else if(cnt < 0){
			cnt += ENCODER_CPR;
		}
		Encoder.cnt = cnt;
	}else{
		// dir swap
		if(UsrConfig.encoder_dir_rev){
			Encoder.raw = ENCODER_CPR - MA730_read_raw();
		}else{
			Encoder.raw = MA730_read_raw();
		}
		Encoder.cnt = Encoder.raw;
	}
	
	// Turns
    if(Encoder.cnt - Encoder.cnt_last > ENCODER_CPR/2){
        Encoder.turns -= 1;
    }else if (Encoder.cnt - Encoder.cnt_last < -ENCODER_CPR/2){
        Encoder.turns += 1;
    }
    Encoder.cnt_last = Encoder.cnt;
	
	// Position
	Encoder.position = Encoder.turns + (float)Encoder.cnt / (float)ENCODER_CPR;
	
	// Elec angle
	Encoder.elec_angle = (UsrConfig.pole_pairs*(float)(Encoder.cnt-UsrConfig.encoder_offset))/((float)ENCODER_CPR);
	Encoder.elec_angle = 2.0f*PI*(Encoder.elec_angle - (int)Encoder.elec_angle);
	Encoder.elec_angle = Encoder.elec_angle<0 ? Encoder.elec_angle + 2.0f*PI : Encoder.elec_angle;

    float vel = (Encoder.position - Encoder.position_last) / dt;
	Encoder.position_last = Encoder.position;
    
    float sum = vel;
    for (int i = 1; i < VEL_VEC_SIZE; i++){
        Encoder.vel_vec[VEL_VEC_SIZE - i] = Encoder.vel_vec[VEL_VEC_SIZE-i-1];
        sum += Encoder.vel_vec[VEL_VEC_SIZE-i];
    }
    Encoder.vel_vec[0] = vel;
	
    Encoder.velocity =  sum / ((float)VEL_VEC_SIZE);
    Encoder.velocity_elec = 2.0f*PI * Encoder.velocity * UsrConfig.pole_pairs;
}
