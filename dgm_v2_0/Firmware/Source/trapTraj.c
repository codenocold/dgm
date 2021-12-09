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

#include "trapTraj.h"
#include <math.h>
#include "util.h"

tTraj Traj;

static float Xi_;
static float Xf_;
static float Vi_;

static float Ar_;
static float Vr_;
static float Dr_;

static float Ta_;
static float Tv_;
static float Td_;
//static float Tf_;

static float yAccel_;

// A sign function where input 0 has positive sign (not 0)
float sign_hard(float val) {
    return (signbit(val)) ? -1.0f : 1.0f;
}

// Symbol                     Description
// Ta, Tv and Td              Duration of the stages of the AL profile
// Xi and Vi                  Adapted initial conditions for the AL profile
// Xf                         Position set-point
// s                          Direction (sign) of the trajectory
// Vmax, Amax, Dmax and jmax  Kinematic bounds
// Ar, Dr and Vr              Reached values of acceleration and velocity

void TRAJ_plan(float Xf, float Xi, float Vi,
                                            float Vmax, float Amax, float Dmax)
{
    float dX = Xf - Xi;  // Distance to travel
    float stop_dist = (Vi * Vi) / (2.0f * Dmax); // Minimum stopping distance
    float dXstop = copysign(stop_dist, Vi); // Minimum stopping displacement
    float s = sign_hard(dX - dXstop); // Sign of coast velocity (if any)
    Ar_ = s * Amax;  // Maximum Acceleration (signed)
    Dr_ = -s * Dmax; // Maximum Deceleration (signed)
    Vr_ = s * Vmax;  // Maximum Velocity (signed)

    // If we start with a speed faster than cruising, then we need to decel instead of accel
    // aka "double deceleration move" in the paper
    if ((s * Vi) > (s * Vr_)) {
        Ar_ = -s * Amax;
    }

    // Time to accel/decel to/from Vr (cruise speed)
    Ta_ = (Vr_ - Vi) / Ar_;
    Td_ = -Vr_ / Dr_;

    // Integral of velocity ramps over the full accel and decel times to get
    // minimum displacement required to reach cuising speed
    float dXmin = 0.5f*Ta_*(Vr_ + Vi) + 0.5f*Td_*Vr_;

    // Are we displacing enough to reach cruising speed?
    if (s*dX < s*dXmin) {
        // Short move (triangle profile)
        Vr_ = s * sqrtf(fmax((Dr_*SQ(Vi) + 2*Ar_*Dr_*dX) / (Dr_ - Ar_), 0.0f));
        Ta_ = fmax(0.0f, (Vr_ - Vi) / Ar_);
        Td_ = fmax(0.0f, -Vr_ / Dr_);
        Tv_ = 0.0f;
    } else {
        // Long move (trapezoidal profile)
        Tv_ = (dX - dXmin) / Vr_;
    }

    // Fill in the rest of the values used at evaluation-time
    Traj.Tf_ = Ta_ + Tv_ + Td_;
    Xi_ = Xi;
    Xf_ = Xf;
    Vi_ = Vi;
    yAccel_ = Xi + Vi*Ta_ + 0.5f*Ar_*SQ(Ta_); // pos at end of accel phase
}

void TRAJ_eval(float t)
{
    if (t < 0.0f) {  // Initial Condition
        Traj.Y   = Xi_;
        Traj.Yd  = Vi_;
        Traj.Ydd = 0.0f;
    } else if (t < Ta_) {  // Accelerating
        Traj.Y   = Xi_ + Vi_*t + 0.5f*Ar_*SQ(t);
        Traj.Yd  = Vi_ + Ar_*t;
        Traj.Ydd = Ar_;
    } else if (t < Ta_ + Tv_) {  // Coasting
        Traj.Y   = yAccel_ + Vr_*(t - Ta_);
        Traj.Yd  = Vr_;
        Traj.Ydd = 0.0f;
    } else if (t < Traj.Tf_) {  // Deceleration
        float td     = t - Traj.Tf_;
        Traj.Y   = Xf_ + 0.5f*Dr_*SQ(td);
        Traj.Yd  = Dr_*td;
        Traj.Ydd = Dr_;
    } else if (t >= Traj.Tf_) {  // Final Condition
        Traj.Y   = Xf_;
        Traj.Yd  = 0.0f;
        Traj.Ydd = 0.0f;
    } else {
        // TODO: report error here
    }
}
