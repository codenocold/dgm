#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "main.h"

typedef struct {
	float input_position;
	float input_velocity;
	float input_current;
} ControllerStruct;

extern ControllerStruct Controller;

void CONTROLLER_move_to_pos(float goal_point);

void CONTROLLER_reset(ControllerStruct *controller);
float CONTROLLER_loop(ControllerStruct *controller, float velocity, float position);

#endif
