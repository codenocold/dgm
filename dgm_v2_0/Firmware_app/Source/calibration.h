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

#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "main.h"
#include "foc.h"

typedef enum eCalibrateStep{
	CS_NULL = 0,
	
	CS_MOTOR_R_START,
	CS_MOTOR_R_LOOP,
	CS_MOTOR_R_END,
	
	CS_MOTOR_L_START,
	CS_MOTOR_L_LOOP,
	CS_MOTOR_L_END,
	
	CS_ENCODER_DIR_PP_START,
	CS_ENCODER_DIR_PP_LOCK,
	CS_ENCODER_DIR_PP_LOOP,
	CS_ENCODER_DIR_PP_END,
	
	CS_ENCODER_OFFSET_START,
	CS_ENCODER_OFFSET_CW_LOOP,
	CS_ENCODER_OFFSET_CCW_LOOP,
	CS_ENCODER_OFFSET_END,
	
	CS_ERROR,
}tCalibrationStep;

typedef enum eCalibrationError{
	CE_NULL = 0,
	CE_PHASE_RESISTANCE_OUT_OF_RANGE,
	CE_MOTOR_POLE_PAIRS_OUT_OF_RANGE
}tCalibrationError;

void CALIBRATION_start(void);
void CALIBRATION_end(void);
void CALIBRATION_loop(FOCStruct *foc);

#endif
