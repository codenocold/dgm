#include "drv8323.h"
#include "systick.h "

#define NCS_SET()		gpio_bit_set(GPIOA, GPIO_PIN_4)
#define NCS_RESET()		gpio_bit_reset(GPIOA, GPIO_PIN_4)
#define DRV_ENABLE()	gpio_bit_set(GPIOA, GPIO_PIN_3)	
#define DRV_DISABLE()	gpio_bit_reset(GPIOA, GPIO_PIN_3)


int DRV8323_init(void)
{
	spi_parameter_struct spi_init_struct;

	/* Peripheral clock enable */
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_SPI0);
	
	// PB0   ------> DRV_nFAULT
	gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
	
	// PA3   ------> DRV_ENABLE
	gpio_bit_reset(GPIOA, GPIO_PIN_3);
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
	
	// PA4   ------> DRV_nCS
	gpio_bit_set(GPIOA, GPIO_PIN_4);
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);

	/* SPI0 GPIO config: SCK/PA5, MISO/PA6, MOSI/PA7 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
	
	/* deinitilize SPI and the parameters */
    spi_i2s_deinit(SPI0);
    spi_struct_para_init(&spi_init_struct);

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_16BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_64;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct);
	
	spi_enable(SPI0);
	
	return DRV8323_reset();
}

int DRV8323_reset(void)
{
	// DRV8323 config
	DRV8323_set_enable(false);
	SYSTICK_delay_us(20);
	DRV8323_set_enable(true);
	
	DRV8323_calibrate();
	SYSTICK_delay_us(100);
	DRV8323_write_DCR(0x0, 0x0, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x1, 0x0, 0x1);
    SYSTICK_delay_us(100);
	DRV8323_write_HSR(LOCK_OFF, IDRIVEP_HS_80MA, IDRIVEN_HS_660MA);
	SYSTICK_delay_us(100);
	DRV8323_write_LSR(1, TDRIVE_1000NS, IDRIVEP_LS_80MA, IDRIVEN_LS_660MA);
	SYSTICK_delay_us(100);
    DRV8323_write_CSACR(0x0, VREF_DIV_2, 0x0, CSA_GAIN_40, 0x0, 0x0, 0x0, 0x0, SEN_LVL_1_0);
    SYSTICK_delay_us(100);
    DRV8323_write_OCPCR(TRETRY_4MS, DEADTIME_100NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_1_88);
	SYSTICK_delay_us(100);
	
	if(0x0024 != DRV8323_read_reg(DRV8323_DRIVER_CONTROL)){
		return -1;
	}
	if(0x0339 != DRV8323_read_reg(DRV8323_GATE_DRIVE_HS)){
		return -1;
	}
	if(0x0539 != DRV8323_read_reg(DRV8323_GATE_DRIVE_LS)){
		return -1;
	}
	if(0x02C3 != DRV8323_read_reg(DRV8323_CSA_CONTROL)){
		return -1;
	}
	if(0x017F != DRV8323_read_reg(DRV8323_OCP_CONTROL)){
		return -1;
	}
	
	return 0;
}

static int spi_transmit_receive(uint16_t data_in, uint16_t *data_out){
	int state = 0;
	*data_out = 0;
	uint32_t timeout_cnt;
	static const uint32_t timeout_cnt_num = 10000;

	/* Wait until TXE flag is set to send data */
	timeout_cnt = 0;
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)){
		timeout_cnt ++;
		if(timeout_cnt > timeout_cnt_num){
			state = -1;
			break;
		}
	}
	
	/* Transmit data in 16 Bit mode */
	spi_i2s_data_transmit(SPI0, data_in);
	
	/* Check BSY flag */
	timeout_cnt = 0;
	while(spi_i2s_flag_get(SPI0, SPI_FLAG_TRANS)){
		timeout_cnt ++;
		if(timeout_cnt > timeout_cnt_num){
			state = -1;
			break;
		}
	}
	
	/* Check RXNE flag */
	timeout_cnt = 0;
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)){
		timeout_cnt ++;
		if(timeout_cnt > timeout_cnt_num){
			state = -1;
			break;
		}
	}
	
	// Read 16-Bits in the data register
	*data_out = spi_i2s_data_receive(SPI0);
	
	return state;
}

uint16_t DRV8323_read_reg(const DRV8323_Reg reg)
{
	uint16_t controlword = 0x8000 | reg; // MSbit =1 for read, address is 3 bits (MSbit is always 0), data is 11 bits
	uint16_t recbuff = 0xbeef;

	// CS
	NCS_RESET();
	
	spi_transmit_receive(controlword, &recbuff);
	
	// NCS
	NCS_SET();

	return (0x7ff&recbuff);
}

void DRV8323_write_reg(const DRV8323_Reg reg, uint16_t regVal)
{
	uint16_t controlword = (reg | (regVal & 0x7ff)); // MSbit =0 for write, address is 3 bits (MSbit is always 0), data is 11 bits
	uint16_t recbuff = 0xbeef;
	
	// CS
	NCS_RESET();
	
	spi_transmit_receive(controlword, &recbuff);
	
	// NCS
	NCS_SET();
}

uint32_t DRV8323_getFault(void)
{
	uint16_t val1 = DRV8323_read_FSR1();
	SYSTICK_delay_us(10);
	uint16_t val2 = DRV8323_read_FSR2();
	SYSTICK_delay_us(10);
	
	val1 &= 0x07FF;
	val2 &= 0x07FF;
	
	return (uint32_t)(val1<<11 & val2);
}

void DRV8323_set_enable(bool enable)
{
	if(enable){
		// Enable driver
		DRV_ENABLE();
	}else{
		// Disable driver
		DRV_DISABLE();
	}
	
	//Wait for driver to come online
	SYSTICK_delay_ms(2);
}

uint16_t DRV8323_read_FSR1(void)
{
    return DRV8323_read_reg(DRV8323_FAULT_STATUS_1);
}

uint16_t DRV8323_read_FSR2(void)
{
    return DRV8323_read_reg(DRV8323_FAULT_STATUS_2);
}

void DRV8323_write_DCR(int DIS_CPUV, int DIS_GDF, int OTW_REP, int PWM_MODE, int PWM_COM, int PWM_DIR, int COAST, int BRAKE, int CLR_FLT)
{
	uint16_t val = (DIS_CPUV<<9) | (DIS_GDF<<8) | (OTW_REP<<7) | (PWM_MODE<<5) | (PWM_COM<<4) | (PWM_DIR<<3) | (COAST<<2) | (BRAKE<<1) | CLR_FLT;
	DRV8323_write_reg(DRV8323_DRIVER_CONTROL, val);
}

void DRV8323_write_HSR(int LOCK, int IDRIVEP_HS, int IDRIVEN_HS)
{
    uint16_t val = (LOCK<<8) | (IDRIVEP_HS<<4) | IDRIVEN_HS;
    DRV8323_write_reg(DRV8323_GATE_DRIVE_HS, val);
}

void DRV8323_write_LSR(int CBC, int TDRIVE, int IDRIVEP_LS, int IDRIVEN_LS)
{
    uint16_t val = (CBC<<10) | (TDRIVE<<8) | (IDRIVEP_LS<<4) | IDRIVEN_LS;
    DRV8323_write_reg(DRV8323_GATE_DRIVE_LS, val);
}

void DRV8323_write_OCPCR(int TRETRY, int DEAD_TIME, int OCP_MODE, int OCP_DEG, int VDS_LVL)
{
    uint16_t val = (TRETRY<<10) | (DEAD_TIME<<8) | (OCP_MODE<<6) | (OCP_DEG<<4) | VDS_LVL;
    DRV8323_write_reg(DRV8323_OCP_CONTROL, val);
}

void DRV8323_write_CSACR(int CSA_FET, int VREF_DIV, int LS_REF, int CSA_GAIN, int DIS_SEN, int CSA_CAL_A, int CSA_CAL_B, int CSA_CAL_C, int SEN_LVL)
{
    uint16_t val = (CSA_FET<<10) | (VREF_DIV<<9) | (LS_REF<<8) | (CSA_GAIN<<6) | (DIS_SEN<<5) | (CSA_CAL_A<<4) | (CSA_CAL_B<<3) | (CSA_CAL_C<<2) | SEN_LVL;
    DRV8323_write_reg(DRV8323_CSA_CONTROL, val);
}

void DRV8323_enable_gd(void)
{
    uint16_t val = (DRV8323_read_reg(DRV8323_DRIVER_CONTROL)) & (~(0x1<<2));
    DRV8323_write_reg(DRV8323_DRIVER_CONTROL, val);
}
    
void DRV8323_disable_gd(void)
{
    uint16_t val = (DRV8323_read_reg(DRV8323_DRIVER_CONTROL)) | (0x1<<2);    
    DRV8323_write_reg(DRV8323_DRIVER_CONTROL, val);
}

void DRV8323_calibrate(void)
{
    uint16_t val = 0x1<<4 | 0x1<<3 | 0x1<<2;
    DRV8323_write_reg(DRV8323_CSA_CONTROL, val);
}
