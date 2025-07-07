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

#include "encoder.h"
#include "pwm_curr.h"
#include "usr_config.h"
#include "util.h"

tEncoder Encoder;

void ENCODER_init(void)
{
    // Init
    Encoder.need_init    = 20;
    Encoder.shadow_count = 0;
    Encoder.pll_pos      = 0;
    Encoder.pll_vel      = 0;

    int encoder_pll_bw     = 100 * M_2PI;
    Encoder.pll_kp         = 2.0f * encoder_pll_bw;      // basic conversion to discrete time
    Encoder.pll_ki         = 0.25f * SQ(Encoder.pll_kp); // Critically damped
    Encoder.snap_threshold = 0.5f * CURRENT_MEASURE_PERIOD * Encoder.pll_ki;
}

static inline void delay_100ns(void)
{
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}

int32_t ENCODER_read(void)
{
    uint16_t data[2];
    uint16_t sample_data;

    ENC_NCS_RESET();
    spi_i2s_data_transmit(SPI2, 0x8300);
    while (RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE))
        ;
    data[0] = spi_i2s_data_receive(SPI2);
    ENC_NCS_SET();

    delay_100ns();

    ENC_NCS_RESET();
    spi_i2s_data_transmit(SPI2, 0x8400);
    while (RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE))
        ;
    data[1] = spi_i2s_data_receive(SPI2);
    ENC_NCS_SET();

    sample_data = ((data[0] & 0x00FF) << 8) | (data[1] & 0x00FF);

    return (sample_data >> 2);
}

void ENCODER_loop(void)
{
    Encoder.raw = ENCODER_read();
    if (UsrConfig.encoder_dir == -1) {
        Encoder.raw = ENCODER_CPR - 1 - Encoder.raw;
    }

    /* Linearization */
    int off_1      = UsrConfig.offset_lut[(Encoder.raw) >> 7];
    int off_2      = UsrConfig.offset_lut[((Encoder.raw >> 7) + 1) % 128];
    int off_interp = off_1 + ((off_2 - off_1) * (Encoder.raw - ((Encoder.raw >> 7) << 7)) >> 7);

    int count = Encoder.raw - off_interp - UsrConfig.encoder_offset;

    /*  Wrap in ENCODER_CPR */
    while (count > ENCODER_CPR)
        count -= ENCODER_CPR;
    while (count < 0)
        count += ENCODER_CPR;

    Encoder.count_in_cpr = count;

    if (Encoder.need_init) {
        Encoder.need_init--;
        Encoder.count_in_cpr_prev = Encoder.count_in_cpr;
        return;
    }

    /* Delta count */
    int delta_count           = Encoder.count_in_cpr - Encoder.count_in_cpr_prev;
    Encoder.count_in_cpr_prev = Encoder.count_in_cpr;
    while (delta_count > +ENCODER_CPR_DIV)
        delta_count -= ENCODER_CPR;
    while (delta_count < -ENCODER_CPR_DIV)
        delta_count += ENCODER_CPR;

    // Run pll (for now pll is in units of encoder counts)
    // Predict current pos
    Encoder.pll_pos += CURRENT_MEASURE_PERIOD * Encoder.pll_vel;
    // Discrete phase detector
    float delta_pos = Encoder.count_in_cpr - floorf(Encoder.pll_pos);
    while (delta_pos > +ENCODER_CPR_DIV)
        delta_pos -= ENCODER_CPR_F;
    while (delta_pos < -ENCODER_CPR_DIV)
        delta_pos += ENCODER_CPR_F;
    // PLL feedback
    Encoder.pll_pos += CURRENT_MEASURE_PERIOD * Encoder.pll_kp * delta_pos;
    while (Encoder.pll_pos > ENCODER_CPR)
        Encoder.pll_pos -= ENCODER_CPR_F;
    while (Encoder.pll_pos < 0)
        Encoder.pll_pos += ENCODER_CPR_F;
    Encoder.pll_vel += CURRENT_MEASURE_PERIOD * Encoder.pll_ki * delta_pos;

    // Align delta-sigma on zero to prevent jitter
    if (ABS(Encoder.pll_vel) < Encoder.snap_threshold) {
        Encoder.pll_vel = 0.0f;
    }

    /* Outputs from Encoder for Controller */
    Encoder.shadow_count += delta_count;
    Encoder.pos       = Encoder.shadow_count / ENCODER_CPR_F;
    Encoder.vel       = Encoder.pll_vel / ENCODER_CPR_F;
    Encoder.phase     = (M_2PI * UsrConfig.motor_pole_pairs) * Encoder.count_in_cpr / ENCODER_CPR_F;
    Encoder.phase_vel = (M_2PI * UsrConfig.motor_pole_pairs) * Encoder.vel;
}
