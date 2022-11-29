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

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

#define ENCODER_CPR			 262144
#define ENCODER_CPR_DIV_2	(ENCODER_CPR>>1)

typedef struct sEncoder {
	int raw;
	int cnt;
	int pos_abs_;
	int count_in_cpr_;
	int shadow_count_;
	float pos_estimate_counts_;
	float vel_estimate_counts_;
	float pos_cpr_counts_;
	
	float pos_estimate_;
    float vel_estimate_;
	float pos_cpr_;
	
	float phase_;
	float interpolation_;

	float pll_kp_;
	float pll_ki_;
} tEncoder;

extern tEncoder Encoder;

void ENCODER_init(void);
uint32_t ENCODER_read_raw(void);
void ENCODER_sample(void);

#endif
