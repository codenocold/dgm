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

float CONTROLLER_get_integrator_current(void);
void CONTROLLER_reset(ControllerStruct *controller);
float CONTROLLER_loop(ControllerStruct *controller, int control_mode, float velocity, float position);

#endif
