#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "main.h"
#include <stdbool.h>

#define FW_VERSION_MAJOR	0
#define FW_VERSION_MINOR	3

#define OFFSET_LUT_NUM		128

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
	int motor_pole_pairs;				// (Auto)
	float motor_phase_resistance;		// (Auto)
	float motor_phase_inductance;		// (Auto)
	float inertia;						// [Nm/(turn/s^2)]
	
	// Encoder
	int encoder_dir_rev;					// (Auto)
	int encoder_offset;						// (Auto)
	int16_t offset_lut[OFFSET_LUT_NUM];		// (Auto)
	
	// Calib
	int calib_valid;				// (Auto)
	float calib_current;			// [A]
	float calib_max_voltage;		// [V]
	
	// Control
	int control_mode;
	float current_ramp_rate;		// [A/sec]
	float vel_ramp_rate;			// [(turn/s)/s]
	float traj_vel;					// [turn/s]
	float traj_accel;				// [(turn/s)/s]
	float traj_decel;				// [(turn/s)/s]
	float pos_gain;
	float vel_gain;
	float vel_integrator_gain;
	float vel_limit;				// [turn/s]
	float current_limit;			// [A]
	float current_ctrl_p_gain;		// (Auto)
	float current_ctrl_i_gain;		// (Auto)
	int current_ctrl_bandwidth; 	// Current loop bandwidth 100~2000
	
	// Protect
	float protect_under_voltage;	// [V]
	float protect_over_voltage;		// [V]
	float protect_over_speed;		// [turn/s]
	
	// CAN
	int can_id;						// CAN bus ID
	int can_timeout_ms;				// CAN bus timeout in ms 0 Disable
	int can_sync_target_enable;		// 0 Disable : else Enable
	
	uint32_t crc;
} tUsrConfig;

extern tUsrConfig UsrConfig;

void USR_CONFIG_set_default_config(void);
int USR_CONFIG_read_config(void);
int USR_CONFIG_save_config(void);

#endif
