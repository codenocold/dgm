#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "main.h"
#include <stdbool.h>

#define FW_VERSION_MAJOR	0
#define FW_VERSION_MINOR	2

#define OFFSET_LUT_NUM		128
#define COGGING_MAP_NUM		2048

typedef struct sUsrConfig{
	// Motor
	int pole_pairs;				// (Auto)
	float phase_resistance;		// (Auto)
	float phase_inductance;		// (Auto)
	float inertia;				// [Nm/(turn/s^2)]
	float torque_constant;		// [Nm/A]
	
	// Encoder
	int encoder_dir_rev;					// (Auto)
	int encoder_offset;						// (Auto)
	int16_t offset_lut[OFFSET_LUT_NUM];		// (Auto)
	
	// Calib
	int calib_valid;
	float calib_current;
	float calib_max_voltage;
	
	// Anti cogging
	int anticogging_enable;
	float anticogging_pos_threshold;
	float anticogging_vel_threshold;
	
	// Control
	int input_mode;
	int control_mode;
	float torque_ramp_rate;		  	// Nm/sec
	float vel_ramp_rate;		  	// [(turn/s)/s]
	int input_pos_filter_bandwidth; // [1/s]
	float traj_vel;			// [turn/s]
	float traj_accel;		// [(turn/s)/s]
	float traj_decel;		// [(turn/s)/s]
	int gain_scheduling_enable;
	float gain_scheduling_width;
	float pos_gain;
	float vel_gain;
	float vel_integrator_gain;
	float vel_limit;
	float current_limit;
	float current_ctrl_p_gain;	// (Auto)
	float current_ctrl_i_gain;	// (Auto)
	int current_ctrl_bandwidth; // Current loop bandwidth 100~2000
	
	// Protect
	float protect_under_voltage;
	float protect_over_voltage;
	float protect_over_speed;
	
	// CAN
	int can_id;				// CAN bus ID
	int can_timeout_ms;		// CAN bus timeout
	int can_sync_target_enable;
	
	// Pad to 64 bit
//	int _pad_0;
	
	uint32_t crc;
} tUsrConfig;

typedef struct sCoggingMap{
	float map[COGGING_MAP_NUM];
	
	// Pad to 64 bit
	uint32_t _pad_0;
	
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
