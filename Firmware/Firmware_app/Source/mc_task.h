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

#ifndef __MC_TASKS_H__
#define __MC_TASKS_H__

#include "main.h"

// Motor controler state
typedef enum eFSMState {
    BOOT_UP     = 0,
    IDLE        = 1,
    RUN         = 2,
    CALIBRATION = 3,
    ANTICOGGING = 4,
} tFSMState;

typedef struct sMCStatusword
{
    union {
        uint8_t status_code;
        struct
        {
            uint32_t switched_on : 1;
            uint32_t target_reached : 1;
            uint32_t PADDING1 : 6;
        };
    } status;

    union {
        uint8_t errors_code;
        struct
        {
            // ERROR
            uint32_t over_voltage : 1;
            uint32_t under_voltage : 1;
            uint32_t over_current : 1;
            uint32_t drv_over_tmp : 1;
            uint32_t ntc_over_tmp : 1;
            uint32_t PADDING2 : 2;

            // FATAL
            uint32_t selftest : 1;
        };
    } errors;
} tMCStatusword;

extern volatile tMCStatusword StatuswordNew;
extern volatile tMCStatusword StatuswordOld;

void      MCT_init(void);
int       MCT_reset_error(void);
tFSMState MCT_get_state(void);
int       MCT_set_state(tFSMState state);

void MCT_high_frequency_task(void);
void MCT_safety_task(void);
void MCT_low_priority_task(void);

#endif
