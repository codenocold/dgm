#include "encoder.h"
#include "systick.h"
#include "util.h"
#include "usr_config.h"

tEncoder Encoder;

#define NCS_SET()		GPIO_BOP(GPIOB) = (uint32_t)GPIO_PIN_12;
#define NCS_RESET()		GPIO_BC(GPIOB) = (uint32_t)GPIO_PIN_12;

void ENCODER_init(void)
{
	spi_parameter_struct spi_init_struct;
	
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_SPI1);
	
	// PB12   ------> ENC_nCS
	gpio_bit_set(GPIOB, GPIO_PIN_12);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
	
	/* SPI1 GPIO config: SCK/PB13, MISO/PB14, MOSI/PB15 */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_15);
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
	
	/* deinitilize SPI and the parameters */
    spi_i2s_deinit(SPI1);
    spi_struct_para_init(&spi_init_struct);

    /* SPI1 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_16BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_2;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI1, &spi_init_struct);
	
	spi_enable(SPI1);
	
	SYSTICK_delay_ms(10);
	
	Encoder.turns = 0;
}

static int spi_transmit_receive(uint16_t data_in, uint16_t *data_out){
	int state = 0;
	*data_out = 0;
	uint32_t timeout_cnt;
	static const uint32_t timeout_cnt_num = 10000;

	/* Wait until TXE flag is set to send data */
	timeout_cnt = 0;
	while(RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_TBE)){
		timeout_cnt ++;
		if(timeout_cnt > timeout_cnt_num){
			state = -1;
			break;
		}
	}
	
	/* Transmit data in 16 Bit mode */
	spi_i2s_data_transmit(SPI1, data_in);
	
	/* Check BSY flag */
	timeout_cnt = 0;
	while(spi_i2s_flag_get(SPI1, SPI_FLAG_TRANS)){
		timeout_cnt ++;
		if(timeout_cnt > timeout_cnt_num){
			state = -1;
			break;
		}
	}
	
	/* Check RXNE flag */
	timeout_cnt = 0;
	while(RESET == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE)){
		timeout_cnt ++;
		if(timeout_cnt > timeout_cnt_num){
			state = -1;
			break;
		}
	}
	
	// Read 16-Bits in the data register
	*data_out = spi_i2s_data_receive(SPI1);
	
	return state;
}

static inline uint16_t MA730_read_raw(void)
{
	uint16_t txData = 0;
	uint16_t rxData;

	// CS
	NCS_RESET();
	
	spi_transmit_receive(txData, &rxData);
	
	// NCS
	NCS_SET();

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
	Encoder.elec_angle = (UsrConfig.motor_pole_pairs*(float)(Encoder.cnt-UsrConfig.encoder_offset))/((float)ENCODER_CPR);
	Encoder.elec_angle = 2.0f*M_PI*(Encoder.elec_angle - (int)Encoder.elec_angle);
	Encoder.elec_angle = Encoder.elec_angle<0 ? Encoder.elec_angle + 2.0f*M_PI : Encoder.elec_angle;
	
    float vel = (Encoder.position - Encoder.position_last) / dt;
	Encoder.position_last = Encoder.position;
    
    float sum = vel;
    for (int i = 1; i < VEL_VEC_SIZE; i++){
        Encoder.vel_vec[VEL_VEC_SIZE - i] = Encoder.vel_vec[VEL_VEC_SIZE-i-1];
        sum += Encoder.vel_vec[VEL_VEC_SIZE-i];
    }
    Encoder.vel_vec[0] = vel;

    Encoder.velocity =  sum / ((float)VEL_VEC_SIZE);
    Encoder.velocity_elec = 2.0f*M_PI * Encoder.velocity * UsrConfig.motor_pole_pairs;
}
