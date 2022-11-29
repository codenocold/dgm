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

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "main.h"
#include <stdbool.h>

#define FW_VERSION_MAJOR	2
#define FW_VERSION_MINOR	1

#define OFFSET_LUT_NUM		128
#define COGGING_MAP_NUM		3000

typedef enum {
	CONTROL_MODE_CURRENT		= 0,
	CONTROL_MODE_CURRENT_RAMP	= 1,
	CONTROL_MODE_VELOCITY		= 2,
	CONTROL_MODE_VELOCITY_RAMP	= 3,
	CONTROL_MODE_POSITION		= 4,
	CONTROL_MODE_POSITION_TRAP	= 5,
} tControlMode;

typedef struct sUsrConfig{
	// Motor
	int motor_pole_pairs;			// (Auto)
	float motor_phase_resistance;	// (Auto)
	float motor_phase_inductance;	// (Auto)
	float inertia;					// [A/(turn/s^2)]
	
	// Encoder
	int encoder_dir;				// (Auto)
	int encoder_offset;				// (Auto)
	int offset_lut[OFFSET_LUT_NUM];	// (Auto)
	
	// Calib
	int calib_valid;				// (Auto)
	float calib_current;			// [A]
	float calib_max_voltage;		// [V]
	
	// Anticogging
	int anticogging_enable;
	float anticogging_pos_threshold;
	float anticogging_vel_threshold;
	
	// Control
	int control_mode;
	float current_ramp_rate;		// [A/sec]
	float vel_ramp_rate;			// [(turn/s)/s]
	float traj_vel;					// [turn/s]
	float traj_accel;				// [(turn/s)/s]
	float traj_decel;				// [(turn/s)/s]
	float pos_gain;					// [(turn/s)/turn]
	float vel_gain;					// [A/(turn/s)]
	float vel_integrator_gain;		// [A/((turn/s)*s)]
	float vel_limit;				// [turn/s]
	float current_limit;			// [A]
	float current_ctrl_p_gain;		// (Auto)
	float current_ctrl_i_gain;		// (Auto)
	int current_ctrl_bandwidth; 	// [rad/s] Current loop bandwidth 100~2000
	
	// Protect
	float protect_under_voltage;	// [V]
	float protect_over_voltage;		// [V]
	float protect_over_speed;		// [turn/s]
	
	// CAN
	int can_id;						// CAN bus ID
	int can_timeout_ms;				// CAN bus timeout in ms : 0 Disable
	int can_sync_target_enable;		// 0 Disable : 1 Enable
	
	uint32_t crc;
} tUsrConfig;

typedef struct sCoggingMap{
	float map[COGGING_MAP_NUM];
	
	uint32_t crc;
} tCoggingMap;

extern tUsrConfig UsrConfig;
extern tCoggingMap *pCoggingMap;

void USR_CONFIG_set_default_config(void);
void USR_CONFIG_set_default_cogging_map(void);
int USR_CONFIG_read_config(void);
int USR_CONFIG_save_config(void);
int USR_CONFIG_read_cogging_map(void);
int USR_CONFIG_save_cogging_map(void);

#endif
