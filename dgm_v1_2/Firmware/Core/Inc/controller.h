#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "main.h"

typedef struct {
	float input_pos;
	float input_vel;
	float input_torque;
} ControllerStruct;

typedef enum {
	INPUT_MODE_PASSTHROUGH        = 0,
	INPUT_MODE_TORQUE_RAMP        = 1,
	INPUT_MODE_VEL_RAMP           = 2,
	INPUT_MODE_POS_FILTER         = 3,
	INPUT_MODE_TRAP_TRAJ          = 4,
} tInputMode;

typedef enum {
	CONTROL_MODE_TORQUE_CONTROL   = 0,
	CONTROL_MODE_VELOCITY_CONTROL = 1,
	CONTROL_MODE_POSITION_CONTROL = 2,
} tControlMode;

extern ControllerStruct Controller;
	
void CONTROLLER_reset(ControllerStruct *controller);
void CONTROLLER_move_to_pos(float goal_point);
float CONTROLLER_get_integrator_current(void);
void CONTROLLER_update(ControllerStruct *controller, float *current_output);

#endif
