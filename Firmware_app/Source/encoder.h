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

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

#define ENCODER_CPR     (int) 262144
#define ENCODER_CPR_F   (262144.0f)
#define ENCODER_CPR_DIV (ENCODER_CPR >> 1)

typedef struct sMT6825
{
    bool     no_mag;
    bool     over_speed;
    uint32_t angle;

    uint8_t rx_err_count;
    uint8_t check_err_count;
} tMT6825;

typedef struct sEncoder
{
    int raw;
    int count_in_cpr;
    int count_in_cpr_prev;

    int64_t shadow_count;

    // pll use
    float pos_cpr_counts;
    float vel_estimate_counts;

    float pos;
    float vel;

    float phase;
    float phase_vel;

    float pll_kp;
    float pll_ki;
    float interpolation;
    float snap_threshold;
} tEncoder;

extern tMT6825  MT6825;
extern tEncoder Encoder;

void ENCODER_init(void);
bool ENCODER_sample(void);
void ENCODER_loop(void);

#endif
