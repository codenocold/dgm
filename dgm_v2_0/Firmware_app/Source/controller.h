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

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "main.h"

typedef enum {
	CONTROL_MODE_TORQUE_RAMP		= 0,
	CONTROL_MODE_VELOCITY_RAMP		= 1,
	CONTROL_MODE_POSITION_FILTER	= 2,
    CONTROL_MODE_POSITION_PROFILE   = 3,
} tControlMode;

typedef struct sController{
	float input_position;
	float input_velocity;
	float input_torque;
	
	float input_position_buffer;
	float input_velocity_buffer;
	float input_torque_buffer;
	
	float pos_setpoint;
	float vel_setpoint;
	float torque_setpoint;

	bool input_updated;
	float input_pos_filter_kp;
	float input_pos_filter_ki;
	float vel_integrator_torque;
} tController;

extern tController Controller;

int CONTROLLER_set_home(void);
void CONTROLLER_sync_callback(void);

void CONTROLLER_init(void);
void CONTROLLER_update_input_pos_filter_gain(float bw);
void CONTROLLER_reset(void);
void CONTROLLER_loop(void);

#endif
