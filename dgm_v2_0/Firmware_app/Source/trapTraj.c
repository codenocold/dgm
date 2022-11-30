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

#include "trapTraj.h"
#include "pwm_curr.h"
#include <math.h>
#include "util.h"

tTraj Traj;

// A sign function where input 0 has positive sign (not 0)
static inline float sign_hard(float val) {
    return (signbit(val)) ? -1.0f : 1.0f;
}

void TRAJ_plan(float position, float start_position, float start_velocity, float Vmax, float Amax, float Dmax)
{
    float distance = position - start_position;             // Distance to travel
    float stop_dist = SQ(start_velocity) / (2.0f * Dmax);   // Minimum stopping distance
    float dXstop = copysign(stop_dist, start_velocity);     // Minimum stopping displacement
    float s = sign_hard(distance - dXstop);                 // Sign of coast velocity (if any)
    Traj.acc =  s * Amax;   // Maximum Acceleration (signed)
    Traj.dec = -s * Dmax;   // Maximum Deceleration (signed)
    Traj.vel =  s * Vmax;   // Maximum Velocity (signed)

    // If we start with a speed faster than cruising, then we need to decel instead of accel aka "double deceleration move" in the paper
    if ((s * start_velocity) > (s * Traj.vel)) {
        Traj.acc = -s * Amax;
    }

    // Time to accel/decel to/from Vr (cruise speed)
    Traj.t_acc = (Traj.vel - start_velocity) / Traj.acc;
    Traj.t_dec = -Traj.vel / Traj.dec;

    // Integral of velocity ramps over the full accel and decel times to get
    // minimum displacement required to reach cuising speed
    float dXmin = 0.5f * Traj.t_acc * (Traj.vel + start_velocity) + 0.5f * Traj.t_dec * Traj.vel;

    // Are we displacing enough to reach cruising speed?
    if (s * distance < s * dXmin) {
        // Short move (triangle profile)
        Traj.vel = s * sqrtf(fmax((Traj.dec * SQ(start_velocity) + 2.0f * Traj.acc * Traj.dec * distance) / (Traj.dec - Traj.acc), 0.0f));
        Traj.t_acc = fmax(0.0f, (Traj.vel - start_velocity) / Traj.acc);
        Traj.t_dec = fmax(0.0f, -Traj.vel / Traj.dec);
        Traj.t_vel = 0.0f;
    } else {
        // Long move (trapezoidal profile)
        Traj.t_vel = (distance - dXmin) / Traj.vel;
    }

    // Fill in the rest of the values used at evaluation-time
    Traj.t_total = Traj.t_acc + Traj.t_vel + Traj.t_dec;
    Traj.start_position = start_position;
    Traj.start_velocity = start_velocity;
    Traj.end_position = position;
    Traj.acc_distance = start_position + start_velocity * Traj.t_acc + 0.5f * Traj.acc * SQ(Traj.t_acc); // pos at end of accel phase

    Traj.tick = 0;
    Traj.profile_done = false;
}

void TRAJ_eval(void)
{
    if(Traj.profile_done){
        return;
    }
    
    Traj.tick ++;
    float t = Traj.tick * CURRENT_MEASURE_PERIOD;
    
    if (t < 0.0f) {                                     // Initial Condition
        Traj.Y   = Traj.start_position;
        Traj.Yd  = Traj.start_velocity;
        Traj.Ydd = 0.0f;
    } else if (t < Traj.t_acc) {                // Accelerating
        Traj.Y   = Traj.start_position + Traj.start_velocity * t + 0.5f * Traj.acc * SQ(t);
        Traj.Yd  = Traj.start_velocity + Traj.acc * t;
        Traj.Ydd = Traj.acc;
    } else if (t < Traj.t_acc + Traj.t_vel) {   // Coasting
        Traj.Y   = Traj.acc_distance + Traj.vel * (t - Traj.t_acc);
        Traj.Yd  = Traj.vel;
        Traj.Ydd = 0.0f;
    } else if (t < Traj.t_total) {              // Deceleration
        float td = t - Traj.t_total;
        Traj.Y   = Traj.end_position + 0.5f * Traj.dec * SQ(td);
        Traj.Yd  = Traj.dec * td;
        Traj.Ydd = Traj.dec;
    } else if (t >= Traj.t_total) {             // Final Condition
        Traj.Y   = Traj.end_position;
        Traj.Yd  = 0.0f;
        Traj.Ydd = 0.0f;
        Traj.profile_done = true;
    }
}
