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

tMT6825  MT6825;
tEncoder Encoder;

void ENCODER_init(void)
{
    // Init
    Encoder.raw                 = 0;
    Encoder.count_in_cpr        = 0;
    Encoder.count_in_cpr_prev   = 0;
    Encoder.shadow_count        = 0;
    Encoder.pos_cpr_counts      = 0;
    Encoder.vel_estimate_counts = 0;
    Encoder.pos                 = 0;
    Encoder.vel                 = 0;
    Encoder.phase               = 0;
    Encoder.phase_vel           = 0;
    Encoder.interpolation       = 0;

    int   encoder_pll_bw   = 2000;
    float bandwidth        = MIN(encoder_pll_bw, 0.25f * PWM_FREQUENCY);
    Encoder.pll_kp         = 2.0f * bandwidth;           // basic conversion to discrete time
    Encoder.pll_ki         = 0.25f * SQ(Encoder.pll_kp); // Critically damped
    Encoder.snap_threshold = 0.5f * CURRENT_MEASURE_PERIOD * Encoder.pll_ki;
}

bool ENCODER_sample(void)
{
    uint16_t              pc1;
    uint8_t               pc2;
    uint8_t               data[3];
    uint32_t              timeout_cnt;
    static const uint32_t timeout_cnt_max = 10;

    // CS
    NCS_RESET();

    timeout_cnt = 0;
    while (0 == (SPI_STAT(SPI0) & SPI_FLAG_TBE)) {
        if (timeout_cnt++ > timeout_cnt_max) {
            goto TIMEOUT;
        }
    }
    SPI_DATA(SPI0) = 0x83;

    timeout_cnt = 0;
    while (0 == (SPI_STAT(SPI0) & SPI_FLAG_RBNE)) {
        if (timeout_cnt++ > timeout_cnt_max) {
            goto TIMEOUT;
        }
    }
    data[0] = ((uint16_t) SPI_DATA(SPI0));

    timeout_cnt = 0;
    while (0 == (SPI_STAT(SPI0) & SPI_FLAG_TBE)) {
        if (timeout_cnt++ > timeout_cnt_max) {
            goto TIMEOUT;
        }
    }
    SPI_DATA(SPI0) = 0xFF;

    timeout_cnt = 0;
    while (0 == (SPI_STAT(SPI0) & SPI_FLAG_RBNE)) {
        if (timeout_cnt++ > timeout_cnt_max) {
            goto TIMEOUT;
        }
    }
    data[0] = ((uint16_t) SPI_DATA(SPI0));

    timeout_cnt = 0;
    while (0 == (SPI_STAT(SPI0) & SPI_FLAG_TBE)) {
        if (timeout_cnt++ > timeout_cnt_max) {
            goto TIMEOUT;
        }
    }
    SPI_DATA(SPI0) = 0xFF;

    timeout_cnt = 0;
    while (0 == (SPI_STAT(SPI0) & SPI_FLAG_RBNE)) {
        if (timeout_cnt++ > timeout_cnt_max) {
            goto TIMEOUT;
        }
    }
    data[1] = ((uint16_t) SPI_DATA(SPI0));

    timeout_cnt = 0;
    while (0 == (SPI_STAT(SPI0) & SPI_FLAG_TBE)) {
        if (timeout_cnt++ > timeout_cnt_max) {
            goto TIMEOUT;
        }
    }
    SPI_DATA(SPI0) = 0xFF;

    timeout_cnt = 0;
    while (0 == (SPI_STAT(SPI0) & SPI_FLAG_RBNE)) {
        if (timeout_cnt++ > timeout_cnt_max) {
            goto TIMEOUT;
        }
    }
    data[2] = ((uint16_t) SPI_DATA(SPI0));

    // NCS
    NCS_SET();

    if (MT6825.rx_err_count) {
        MT6825.rx_err_count--;
    }

    // check pc1
    pc1 = (data[0] << 8) | data[1];
    pc1 ^= pc1 >> 8;
    pc1 ^= pc1 >> 4;
    pc1 ^= pc1 >> 2;
    pc1 ^= pc1 >> 1;
    if (pc1 & 1) {
        goto CHECK_ERR;
    }

    // check pc2
    pc2 = data[2] & 0xFC;
    pc2 ^= pc2 >> 4;
    pc2 ^= pc2 >> 2;
    pc2 ^= pc2 >> 1;
    if (pc2 & 1) {
        goto CHECK_ERR;
    }

    MT6825.no_mag     = data[1] & 0x02;
    MT6825.over_speed = data[2] & 0x08;
    data[1] >>= 2;
    data[2] >>= 4;
    MT6825.angle = data[0] << 10 | data[1] << 4 | data[2];

    if (MT6825.check_err_count) {
        MT6825.check_err_count--;
    }

    return true;

CHECK_ERR:
    if (MT6825.check_err_count < 0xFF) {
        MT6825.check_err_count++;
    }

    return false;

TIMEOUT:
    // NCS
    NCS_SET();

    if (MT6825.rx_err_count < 0xFF) {
        MT6825.rx_err_count++;
    }

    return false;
}

void ENCODER_loop(void)
{
    if (ENCODER_sample()) {
        if (UsrConfig.encoder_dir == +1) {
            Encoder.raw = MT6825.angle;
        } else {
            Encoder.raw = (ENCODER_CPR - MT6825.angle);
        }
    }

    /* Linearization */
    int off_1      = UsrConfig.offset_lut[(Encoder.raw) >> 11];             // lookup table lower entry
    int off_2      = UsrConfig.offset_lut[((Encoder.raw >> 11) + 1) % 128]; // lookup table higher entry
    int off_interp = off_1
                     + ((off_2 - off_1) * (Encoder.raw - ((Encoder.raw >> 11) << 11))
                        >> 11); // Interpolate between lookup table entries
    int count = Encoder.raw - off_interp;

    /*  Wrap in ENCODER_CPR */
    while (count > ENCODER_CPR)
        count -= ENCODER_CPR;
    while (count < 0)
        count += ENCODER_CPR;
    Encoder.count_in_cpr = count;

    /* Delta count */
    int delta_count           = Encoder.count_in_cpr - Encoder.count_in_cpr_prev;
    Encoder.count_in_cpr_prev = Encoder.count_in_cpr;
    while (delta_count > +ENCODER_CPR_DIV)
        delta_count -= ENCODER_CPR;
    while (delta_count < -ENCODER_CPR_DIV)
        delta_count += ENCODER_CPR;

    /* Add measured delta to encoder count */
    Encoder.shadow_count += delta_count;

    /* Run vel PLL */
    Encoder.pos_cpr_counts += CURRENT_MEASURE_PERIOD * Encoder.vel_estimate_counts;
    float delta_pos_cpr_counts = (float) (Encoder.count_in_cpr - (int) Encoder.pos_cpr_counts);
    while (delta_pos_cpr_counts > +ENCODER_CPR_DIV)
        delta_pos_cpr_counts -= ENCODER_CPR_F;
    while (delta_pos_cpr_counts < -ENCODER_CPR_DIV)
        delta_pos_cpr_counts += ENCODER_CPR_F;
    Encoder.pos_cpr_counts += CURRENT_MEASURE_PERIOD * Encoder.pll_kp * delta_pos_cpr_counts;
    while (Encoder.pos_cpr_counts > ENCODER_CPR)
        Encoder.pos_cpr_counts -= ENCODER_CPR_F;
    while (Encoder.pos_cpr_counts < 0)
        Encoder.pos_cpr_counts += ENCODER_CPR_F;
    Encoder.vel_estimate_counts += CURRENT_MEASURE_PERIOD * Encoder.pll_ki * delta_pos_cpr_counts;

    // align delta-sigma on zero to prevent jitter
    bool snap_to_zero_vel = false;
    if (ABS(Encoder.vel_estimate_counts) < Encoder.snap_threshold) {
        Encoder.vel_estimate_counts = 0.0f;
        snap_to_zero_vel            = true;
    }

    // run encoder count interpolation
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel) {
        Encoder.interpolation = 0.5f;
        // reset interpolation if encoder edge comes
    } else if (delta_count > 0) {
        Encoder.interpolation = 0.0f;
    } else if (delta_count < 0) {
        Encoder.interpolation = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        Encoder.interpolation += CURRENT_MEASURE_PERIOD * Encoder.vel_estimate_counts;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (Encoder.interpolation > 1.0f)
            Encoder.interpolation = 1.0f;
        if (Encoder.interpolation < 0.0f)
            Encoder.interpolation = 0.0f;
    }
    float interpolated_enc = Encoder.count_in_cpr - UsrConfig.encoder_offset + Encoder.interpolation;
    while (interpolated_enc > ENCODER_CPR)
        interpolated_enc -= ENCODER_CPR;
    while (interpolated_enc < 0)
        interpolated_enc += ENCODER_CPR;

    float shadow_count_f = Encoder.shadow_count;
    float turns          = shadow_count_f / ENCODER_CPR_F;
    float residual       = shadow_count_f - turns * ENCODER_CPR_F;

    /* Outputs from Encoder for Controller */
    Encoder.pos = turns + residual / ENCODER_CPR_F;
    UTILS_LP_MOVING_AVG_APPROX(Encoder.vel, (Encoder.vel_estimate_counts / ENCODER_CPR_F), 5);
    Encoder.phase     = (interpolated_enc * M_2PI * UsrConfig.motor_pole_pairs) / ENCODER_CPR_F;
    Encoder.phase_vel = Encoder.vel * M_2PI * UsrConfig.motor_pole_pairs;
}
