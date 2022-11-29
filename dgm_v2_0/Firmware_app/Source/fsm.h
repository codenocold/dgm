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

#ifndef __FSM_H__
#define __FSM_H__

#include "main.h"

// CMD
#define CMD_MENU				0x1B	// Esc
#define CMD_MOTOR   			'm'
#define CMD_CALIBRATION			'c'
#define CMD_ANTICOGGING			'a'
#define CMD_UPDATE_CONFIGS		'u'
#define CMD_RESET_ERROR			'z'
#define CMD_DEBUG_Q				'q'
#define CMD_DEBUG_W				'w'
#define CMD_UART_SETUP 			's'

// Status
typedef enum eFsmStat {
	FS_STARTUP = 0,
	FS_MENU_MODE,
	FS_MOTOR_MODE,
	FS_CALIBRATION_MODE,
	FS_ANTICOGGING_MODE,
	
	FS_UART_SETUP,
} tFsmStat;

// Errors
#define ERR_OVER_VOLTAGE	((int)1<<0)
#define	ERR_UNDER_VOLTAGE	((int)1<<1)
#define	ERR_OVER_SPEED		((int)1<<2)

int FSM_input(char c);
void FSM_loop(void);
int FSM_get_error(void);
tFsmStat FSM_get_stat(void);

#endif
