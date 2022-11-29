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

#ifndef __UTIL_H__
#define __UTIL_H__

#include "main.h"
#include <math.h>
#include <stdbool.h>

#define M_PI					(3.14159265358f)
#define M_2PI					(6.28318530716f)
#define SQRT3 					(1.73205080757f)
#define SQRT3_BY_2				(0.86602540378f)
#define ONE_BY_SQRT3			(0.57735026919f)
#define TWO_BY_SQRT3			(1.15470053838f)

#define SQ(x) 					((x) * (x))
#define ABS(x) 					( (x)>0?(x):-(x) ) 
#define MIN(a,b) 				(((a)<(b))?(a):(b))
#define MAX(a,b) 				(((a)>(b))?(a):(b))
#define CLAMP(x, lower, upper) 	(MIN(upper, MAX(x, lower)))

// Return the sign of the argument. -1 if negative, 1 if zero or positive.
#define SIGN(x)				((x < 0) ? -1 : 1)

static inline int mod(int dividend, int divisor)
{
    int r = dividend % divisor;
    return (r < 0) ? (r + divisor) : r;
}

uint32_t crc32(const unsigned char *buf, int len, unsigned int init);

static inline void int_to_data(int val, uint8_t *data)
{
	data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}

static inline int data_to_int(uint8_t *data)
{
	int tmp_int;
    *(((uint8_t*)(&tmp_int)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int)) + 1) = data[1];
    *(((uint8_t*)(&tmp_int)) + 2) = data[2];
    *(((uint8_t*)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}

static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}

static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t*)(&tmp_float)) + 0) = data[0];
    *(((uint8_t*)(&tmp_float)) + 1) = data[1];
    *(((uint8_t*)(&tmp_float)) + 2) = data[2];
    *(((uint8_t*)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}

static inline uint32_t cpu_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void cpu_exit_critical(uint32_t priority_mask)
{
    __set_PRIMASK(priority_mask);
}

static inline float fmodf_pos(float x, float y)
{
    float out = fmodf(x, y);
    if (out < 0.0f){
        out += y;
	}
    return out;
}

static inline float wrap_pm(float x, float pm_range)
{
    return fmodf_pos(x + pm_range, 2.0f * pm_range) - pm_range;
}

static inline float wrap_pm_pi(float theta)
{
    return wrap_pm(theta, M_PI);
}

static inline void fast_sincos(float angle, float *sin, float *cos)
{
	//always wrap input angle to -PI..PI
	angle = wrap_pm_pi(angle);

	//compute sine
	if (angle < 0.0f) {
		*sin = 1.27323954f * angle + 0.405284735f * angle * angle;

		if (*sin < 0.0f) {
			*sin = 0.225f * (*sin * -*sin - *sin) + *sin;
		} else {
			*sin = 0.225f * (*sin * *sin - *sin) + *sin;
		}
	} else {
		*sin = 1.27323954f * angle - 0.405284735f * angle * angle;

		if (*sin < 0.0f) {
			*sin = 0.225f * (*sin * -*sin - *sin) + *sin;
		} else {
			*sin = 0.225f * (*sin * *sin - *sin) + *sin;
		}
	}

	// compute cosine: sin(x + PI/2) = cos(x)
	angle += 0.5f * M_PI;
	if (angle >  M_PI) {
		angle -= 2.0f * M_PI;
	}

	if (angle < 0.0f) {
		*cos = 1.27323954f * angle + 0.405284735f * angle * angle;

		if (*cos < 0.0f) {
			*cos = 0.225f * (*cos * -*cos - *cos) + *cos;
		} else {
			*cos = 0.225f * (*cos * *cos - *cos) + *cos;
		}
	} else {
		*cos = 1.27323954f * angle - 0.405284735f * angle * angle;

		if (*cos < 0.0f) {
			*cos = 0.225f * (*cos * -*cos - *cos) + *cos;
		} else {
			*cos = 0.225f * (*cos * *cos - *cos) + *cos;
		}
	}
}

static inline void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
	*Ialpha = Ia;
	*Ibeta  = (Ib - Ic) * ONE_BY_SQRT3;
}

static inline void park_transform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq)
{
	// Id =  Ialpha * cos(Theta) + Ibeta * sin(Theta)
	// Iq = -Ialpha * sin(Theta) + Ibeta * cos(Theta)
	
	float s, c;
	fast_sincos(Theta, &s, &c);
	*Id =   Ialpha * c + Ibeta * s;
    *Iq = - Ialpha * s + Ibeta * c;
}

static inline void inverse_park(float mod_d, float mod_q, float Theta, float *mod_alpha, float *mod_beta)
{
	// mod_alpha = mod_d * Cos(Theta) - mod_q * Sin(Theta)
	// mod_beta  = mod_d * Sin(Theta) + mod_q * Cos(Theta)
	
	float s, c;
	fast_sincos(Theta, &s, &c);
    *mod_alpha = mod_d * c - mod_q * s;
    *mod_beta  = mod_d * s + mod_q * c;
}

static inline int svm(float alpha, float beta, float* tA, float* tB, float* tC)
{
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (ONE_BY_SQRT3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2

        } else {
            //quadrant II
            if (-ONE_BY_SQRT3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-ONE_BY_SQRT3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (ONE_BY_SQRT3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - ONE_BY_SQRT3 * beta;
            float t2 = TWO_BY_SQRT3 * beta;

            // PWM timings
            *tA = (1.0f - t1 - t2) * 0.5f;
            *tB = *tA + t1;
            *tC = *tB + t2;
        } break;

        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + ONE_BY_SQRT3 * beta;
            float t3 = -alpha + ONE_BY_SQRT3 * beta;

            // PWM timings
            *tB = (1.0f - t2 - t3) * 0.5f;
            *tA = *tB + t3;
            *tC = *tA + t2;
        } break;

        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = TWO_BY_SQRT3 * beta;
            float t4 = -alpha - ONE_BY_SQRT3 * beta;

            // PWM timings
            *tB = (1.0f - t3 - t4) * 0.5f;
            *tC = *tB + t3;
            *tA = *tC + t4;
        } break;

        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + ONE_BY_SQRT3 * beta;
            float t5 = -TWO_BY_SQRT3 * beta;

            // PWM timings
            *tC = (1.0f - t4 - t5) * 0.5f;
            *tB = *tC + t5;
            *tA = *tB + t4;
        } break;

        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - ONE_BY_SQRT3 * beta;
            float t6 = alpha - ONE_BY_SQRT3 * beta;

            // PWM timings
            *tC = (1.0f - t5 - t6) * 0.5f;
            *tA = *tC + t5;
            *tB = *tA + t6;
        } break;

        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -TWO_BY_SQRT3 * beta;
            float t1 = alpha + ONE_BY_SQRT3 * beta;

            // PWM timings
            *tA = (1.0f - t6 - t1) * 0.5f;
            *tC = *tA + t1;
            *tB = *tC + t6;
        } break;
    }

    // if any of the results becomes NaN, result_valid will evaluate to false
    int result_valid =
            *tA >= 0.0f && *tA <= 1.0f
         && *tB >= 0.0f && *tB <= 1.0f
         && *tC >= 0.0f && *tC <= 1.0f;
	
    return result_valid ? 0 : -1;
}

#endif
