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

#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"
#include "mc_task.h"

// CAN ID 11 bit high to low
#define ID_ECHO_BIT                0x400 // 1 bit
#define ID_NODE_BIT                0x3E0 // 5 bit
#define ID_CMD_BIT                 0x01F // 5 bit

#define IS_ECHO(can_id)            (can_id & ID_ECHO_BIT)
#define GET_NODE_ID(can_id)        ((can_id & ID_NODE_BIT) >> 5)
#define GET_CMD(can_id)            (can_id & ID_CMD_BIT)

// CMD ID
#define CAN_CMD_SET_OP_MODE        0
#define CAN_CMD_MOTOR_ENABLE       1
#define CAN_CMD_MOTOR_DISABLE      2

#define CAN_CMD_SET_TORQUE         3
#define CAN_CMD_SET_VELOCITY       4
#define CAN_CMD_SET_POSITION       5
#define CAN_CMD_SYNC               6

#define CAN_CMD_CALIB_START        7
#define CAN_CMD_CALIB_REPORT       8
#define CAN_CMD_CALIB_ABORT        9

#define CAN_CMD_ANTICOGGING_START  10
#define CAN_CMD_ANTICOGGING_REPORT 11
#define CAN_CMD_ANTICOGGING_ABORT  12

#define CAN_CMD_SET_HOME           13
#define CAN_CMD_ERROR_RESET        14
#define CAN_CMD_GET_STATUSWORD     15
#define CAN_CMD_STATUSWORD_REPORT  16

#define CAN_CMD_GET_VALUE_1        17
#define CAN_CMD_GET_VALUE_2        18

#define CAN_CMD_HEARTBEAT          23
#define CAN_CMD_SET_CONFIG         24
#define CAN_CMD_GET_CONFIG         25
#define CAN_CMD_SAVE_ALL_CONFIG    26
#define CAN_CMD_RESET_ALL_CONFIG   27
#define CAN_CMD_GET_FW_VERSION     28
#define CAN_CMD_DFU_START          29
#define CAN_CMD_DFU_DATA           30
#define CAN_CMD_DFU_END            31

typedef struct
{
    uint32_t id : 24;
    uint32_t dlc : 8;
    uint8_t  data[8];
} CanFrame;

void CAN_set_node_id(uint8_t nodeID);
void CAN_comm_loop(void);
void CAN_reset_rx_timeout(void);
void CAN_reset_tx_timeout(void);

void CAN_receive_callback(void);

void CAN_tx_statusword(tMCStatusword statusword);

// only used in one NVIC
void CAN_calib_report(int32_t step, uint8_t *data);
void CAN_anticogging_report(int32_t step, int32_t value);

#endif /* __CAN_H__ */
