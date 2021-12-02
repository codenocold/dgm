#ifndef __UTIL_H__
#define __UTIL_H__

#include "main.h"

#define SQ(x) ((x) * (x))
#define SQRT3 1.73205080757f
static const float ONE_BY_SQRT3 = 0.57735026919f;
static const float TWO_BY_SQRT3 = 1.15470053838f;
static const float SQRT3_BY_2 = 0.86602540378f;

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define CLAMP(x, lower, upper) (MIN(upper, MAX(x, lower)))

// Return the sign of the argument. -1 if negative, 1 if zero or positive.
#define SIGN(x)				((x < 0) ? -1 : 1)

unsigned int crc32(const unsigned char *buf, int len, unsigned int init);
int svm(float alpha, float beta, float* tA, float* tB, float* tC);
float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
void limit_norm(float *x, float *y, float limit);
float our_arm_sin_f32(float x);
float our_arm_cos_f32(float x);

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

static inline void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
	// Ialpha = Ia
	// Ibeta = -(2*Ib+Ia)/sqrt(3)
	
	*Ialpha = Ia;
	*Ibeta  = (Ib - Ic) * ONE_BY_SQRT3;		// Ic = -Ia - Ib
}

static inline void park_transform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq)
{
	// Id =  Ialpha * cos(Theta) + Ibeta * sin(Theta)
	// Iq = -Ialpha * sin(Theta) + Ibeta * cos(Theta)
	
	float c = our_arm_cos_f32(Theta);
    float s = our_arm_sin_f32(Theta);
	*Id =   Ialpha * c + Ibeta * s;
    *Iq = - Ialpha * s + Ibeta * c;
}

static inline void inverse_park(float mod_d, float mod_q, float Theta, float *mod_alpha, float *mod_beta)
{
	// mod_alpha = mod_d * Cos(Theta) - mod_q * Sin(Theta)
	// mod_beta  = mod_d * Sin(Theta) + mod_q * Cos(Theta)
	
	float c = our_arm_cos_f32(Theta);
    float s = our_arm_sin_f32(Theta);
    *mod_alpha = mod_d * c - mod_q * s;
    *mod_beta  = mod_d * s + mod_q * c;
}

#endif
