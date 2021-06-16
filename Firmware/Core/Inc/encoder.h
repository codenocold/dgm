#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

#define ENCODER_CPR		16383
#define VEL_VEC_SIZE	40

typedef struct sEncoder {
	uint16_t raw;
	int cnt;
	int cnt_last;
	int turns;
	float position;
	float position_last;
	float elec_angle;
	float velocity;
	float velocity_elec;
	
	float vel_vec[VEL_VEC_SIZE];
} tEncoder;

extern tEncoder Encoder;

void ENCODER_init(void);
void ENCODER_sample(float dt);

#endif
