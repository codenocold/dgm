/*
	Copyright 2021 codenocold 1107795287@qq.com
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

#include "encoder.h"
#include "systick.h"
#include "util.h"
#include "usr_config.h"
#include "pwm_curr_fdbk.h"

tEncoder Encoder;

#define NCS_SET()		GPIO_BOP(GPIOA) = (uint32_t)GPIO_PIN_4;
#define NCS_RESET()		GPIO_BC(GPIOA) = (uint32_t)GPIO_PIN_4;

void ENCODER_init(void)
{
	spi_parameter_struct spi_init_struct;
	
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_SPI0);
	
	// PA4   ------> ENC_nCS
	gpio_bit_set(GPIOA, GPIO_PIN_4);
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
	
	/* SPI1 GPIO config: SCK/PA5, MISO/PA6, MOSI/PA7 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
	
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
	
	SYSTICK_delay_ms(10);
	
	// Init
	Encoder.count_in_cpr_ = 0;
	Encoder.shadow_count_ = 0;
	Encoder.pos_estimate_counts_ = 0.0f;
	Encoder.vel_estimate_counts_ = 0.0f;
	Encoder.pos_cpr_counts_ = 0.0f;
	
	Encoder.pos_estimate_ = 0.0f;
	Encoder.vel_estimate_ = 0.0f;
	Encoder.pos_cpr_ = 0.0f;
	
	Encoder.phase_ = 0.0f;
	Encoder.interpolation_ = 0.0f;
	
	int encoder_pll_bandwidth = 2000;
	Encoder.pll_kp_ = 2.0f * encoder_pll_bandwidth;  				// basic conversion to discrete time
    Encoder.pll_ki_ = 0.25f * (Encoder.pll_kp_ * Encoder.pll_kp_); 	// Critically damped
}

uint32_t ENCODER_read_raw(void)
{
	uint32_t count = 0;
	uint8_t data[3];
	
	// CS
	NCS_RESET();

	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
	spi_i2s_data_transmit(SPI0, 0x83);
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
	spi_i2s_data_transmit(SPI0, 0xFF);
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE));
	spi_i2s_data_receive(SPI0);
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
	spi_i2s_data_transmit(SPI0, 0xFF);
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE));
	data[0] = spi_i2s_data_receive(SPI0);
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
	spi_i2s_data_transmit(SPI0, 0xFF);
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE));
	data[1] = spi_i2s_data_receive(SPI0);
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE));
	data[2] = spi_i2s_data_receive(SPI0);
	
	data[1] >>= 2;
	data[2] >>= 4;
	count = data[0]<<10 | data[1]<<4 | data[2];
	
	// NCS
	NCS_SET();
	
	return count;
}

void ENCODER_sample(void)
{
	int32_t delta_enc = 0;
    Encoder.raw = ENCODER_read_raw();
	
	if(UsrConfig.calib_valid){
		// offset compensation
		int off_1 = UsrConfig.offset_lut[Encoder.raw>>11];
		int off_2 = UsrConfig.offset_lut[((Encoder.raw>>11)+1)%128];
		int off_interp = off_1 + ((off_2 - off_1) * (Encoder.raw - ((Encoder.raw>>11)<<11)) >> 11);		// Interpolate between lookup table entries
		int cnt = Encoder.raw - off_interp;                                     						// Correct for nonlinearity with lookup table from calibration
		if(cnt > ENCODER_CPR){
			cnt -= ENCODER_CPR;
		}else if(cnt < 0){
			cnt += ENCODER_CPR;
		}
		Encoder.cnt = cnt;
	}else{
		Encoder.cnt = Encoder.raw;
	}
	
	delta_enc = Encoder.cnt - Encoder.count_in_cpr_;
	delta_enc = mod(delta_enc, ENCODER_CPR);
	if (delta_enc > ENCODER_CPR_DIV_2) {
		delta_enc -= ENCODER_CPR;
	}
	
	Encoder.shadow_count_ += delta_enc;
    Encoder.count_in_cpr_ += delta_enc;
    Encoder.count_in_cpr_ = mod(Encoder.count_in_cpr_, ENCODER_CPR);
	
	Encoder.count_in_cpr_ = Encoder.cnt;
	
	//// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    Encoder.pos_estimate_counts_ += DT * Encoder.vel_estimate_counts_;
    Encoder.pos_cpr_counts_      += DT * Encoder.vel_estimate_counts_;
    // discrete phase detector
    float delta_pos_counts = (float)(Encoder.shadow_count_ - (int32_t)floor(Encoder.pos_estimate_counts_));
    float delta_pos_cpr_counts = (float)(Encoder.count_in_cpr_ - (int32_t)floor(Encoder.pos_cpr_counts_));
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, ENCODER_CPR_DIV_2);
    // pll feedback
    Encoder.pos_estimate_counts_ += DT * Encoder.pll_kp_ * delta_pos_counts;
    Encoder.pos_cpr_counts_ += DT * Encoder.pll_kp_ * delta_pos_cpr_counts;
    Encoder.pos_cpr_counts_ = fmodf_pos(Encoder.pos_cpr_counts_, (float)(ENCODER_CPR));
    Encoder.vel_estimate_counts_ += DT * Encoder.pll_ki_ * delta_pos_cpr_counts;
    bool snap_to_zero_vel = false;
    if (ABS(Encoder.vel_estimate_counts_) < 0.5f * DT * Encoder.pll_ki_) {
        Encoder.vel_estimate_counts_ = 0.0f;  // align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }
	
	// Outputs from Encoder for Controller
    float pos_cpr_last = Encoder.pos_cpr_;
    Encoder.pos_estimate_ = Encoder.pos_estimate_counts_ / (float)ENCODER_CPR;
    Encoder.vel_estimate_ = Encoder.vel_estimate_counts_ / (float)ENCODER_CPR;
    Encoder.pos_cpr_= Encoder.pos_cpr_counts_ / (float)ENCODER_CPR;
    float delta_pos_cpr = wrap_pm(Encoder.pos_cpr_ - pos_cpr_last, 0.5f);

	//// run encoder count interpolation
    int32_t corrected_enc = Encoder.count_in_cpr_ - UsrConfig.encoder_offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel) {
        Encoder.interpolation_ = 0.5f;
    // reset interpolation if encoder edge comes
    // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
    } else if (delta_enc > 0) {
        Encoder.interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        Encoder.interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        Encoder.interpolation_ += DT * Encoder.vel_estimate_counts_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (Encoder.interpolation_ > 1.0f) Encoder.interpolation_ = 1.0f;
        if (Encoder.interpolation_ < 0.0f) Encoder.interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + Encoder.interpolation_;
	
	//// compute electrical phase
    float elec_rad_per_enc = UsrConfig.motor_pole_pairs * M_2PI * (1.0f / (float)ENCODER_CPR);
    float ph = elec_rad_per_enc * interpolated_enc;
    Encoder.phase_ = wrap_pm_pi(ph);
}
