#ifndef _TRAP_TRAJ_H
#define _TRAP_TRAJ_H

#include "main.h"
#include <stdbool.h>

typedef struct sTraj{
	// Step
	float Y;
	float Yd;
	float Ydd;
	
	float Tf_;
	
	float t;
    bool trajectory_done;
} tTraj;

extern tTraj Traj;

void TRAJ_plan(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax);
void TRAJ_eval(float t);

#endif
