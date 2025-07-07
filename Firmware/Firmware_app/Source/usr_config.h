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

#ifndef __USR_CONFIG_H__
#define __USR_CONFIG_H__

#include "main.h"

#define FW_VERSION_MAJOR 3
#define FW_VERSION_MINOR 5

#define OFFSET_LUT_NUM   128U
#define COGGING_MAP_NUM  5000U

typedef enum {
    CAN_BAUDRATE_250K = 0,
    CAN_BAUDRATE_500K,
    CAN_BAUDRATE_800K,
    CAN_BAUDRATE_1000K,
} tCanBaudrate;

typedef struct sUsrConfig
{
    // Motor
    int32_t invert_motor_dir;       // 0 False : 1 True
    int32_t motor_pole_pairs;       // [PP]      (2~30)
    float   motor_phase_resistance; // [R]       (0~10)
    float   motor_phase_inductance; // [H]       (0~1)
    float   current_limit;          // [A]       (0~10)
    float   velocity_limit;         // [r/s]     (0~100)

    // Calibration
    float calib_current; // [A] (0~10)
    float calib_voltage; // [V] (0~50)

    // Controller
    float   pos_p_gain;
    float   vel_p_gain;
    float   vel_i_gain;
    float   current_ff_gain; // [A/(r/s²)]
    float   current_ctrl_bw; // [Hz] (100~2000)
    int32_t default_op_mode;
    int32_t anticogging_enable;     // 0 False : 1 True
    int32_t sync_target_enable;     // 0 False : 1 True
    float   target_velcity_window;  // [r/s]
    float   target_position_window; // [r]
    float   current_ramp_rate;      // [A/s]
    float   velocity_ramp_rate;     // [r/s²]
    float   position_filter_bw;     // [Hz] (0~1000)
    float   profile_velocity;       // [r/s]
    float   profile_accel;          // [r/s²]
    float   profile_decel;          // [r/s²]

    // Protect
    float   protect_under_voltage; // [V] (0~50)
    float   protect_over_voltage;  // [V] (0~50)
    float   protect_over_current;  // [A] (0~10)
    int32_t protect_drv_over_tmp;  // [℃] (0~150)
    int32_t protect_ntc_over_tmp;  // [℃] (0~150)

    // CAN
    int32_t node_id;               // (1~31)
    int32_t can_baudrate;          // REF: tCanBaudrate
    int32_t heartbeat_consumer_ms; // rx heartbeat timeout in ms : 0 Disable
    int32_t heartbeat_producer_ms; // tx heartbeat interval in ms : 0 Disable

    // Encoder
    int32_t calib_valid;                // (Auto)
    int32_t encoder_dir;                // (Auto)
    int32_t encoder_offset;             // (Auto)
    int32_t offset_lut[OFFSET_LUT_NUM]; // (Auto)

    uint32_t crc;
} tUsrConfig;

typedef struct sCoggingMap
{
    int16_t  map[COGGING_MAP_NUM];
    uint32_t crc;
} tCoggingMap;

extern tUsrConfig   UsrConfig;
extern tCoggingMap *pCoggingMap;

void USR_CONFIG_set_default_config(void);
int  USR_CONFIG_erease_config(void);
int  USR_CONFIG_read_config(void);
int  USR_CONFIG_save_config(void);

void USR_CONFIG_set_default_cogging_map(void);
int  USR_CONFIG_erease_cogging_map(void);
int  USR_CONFIG_read_cogging_map(void);
int  USR_CONFIG_save_cogging_map(void);

#endif
