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

#include "anticogging.h"
#include "systick.h"
#include "usr_config.h"
#include "encoder.h"
#include "heap.h"
#include "can.h"
#include "util.h"
#include "fsm.h"

typedef enum eAntiCoggingStep{
	AS_NULL = 0,
	AS_START,
	AS_FORWARD_LOOP,
	AS_BACKWARD_LOOP,
	AS_END,
}tAntiCoggingStep;

bool AnticoggingValid = false;

static tAntiCoggingStep AntiCoggingStep = AS_NULL;

void ANTICOGGING_start(void)
{
	AnticoggingValid = false;
	USR_CONFIG_set_default_cogging_map();
	AntiCoggingStep = AS_START;
}

void ANTICOGGING_end(void)
{
	AntiCoggingStep = AS_NULL;
	
	if(!AnticoggingValid){
		USR_CONFIG_set_default_cogging_map();
	}
}

void ANTICOGGING_loop(ControllerStruct *controller)
{
	static int index = 0;
	static uint32_t tick = 0;
	
	if(AntiCoggingStep == AS_NULL){
		return;
	}
	
	switch(AntiCoggingStep){
		case AS_START:
			{
				// 500 Hz
				if(SYSTICK_get_ms_since(tick) < 2){
					return;
				}
				tick = SYSTICK_get_tick();
	
				float position = Encoder.pos_estimate_;
				float err = position - (int)position;
				if(err > 0.001f){
					controller->input_position -= 0.001f;
				}else if(err < -0.001f){
					controller->input_position += 0.001f;
				}else{
					controller->input_position -= err;
					index = 0;
					AntiCoggingStep = AS_FORWARD_LOOP;
				}
			}
			break;
		
		case AS_FORWARD_LOOP:
			{
				// 10 Hz
				if(SYSTICK_get_ms_since(tick) < 100){
					return;
				}
				tick = SYSTICK_get_tick();
	
				float pos_err = controller->input_position - Encoder.pos_estimate_;
				if(fabs(pos_err) < UsrConfig.anticogging_pos_threshold && fabs(Encoder.vel_estimate_) < UsrConfig.anticogging_vel_threshold){
					pCoggingMap->map[index] = CONTROLLER_get_integrator_current();
					DEBUG("F %d %f\n\r", index, pCoggingMap->map[index]);
					
					uint8_t data[4];
					float_to_data(pCoggingMap->map[index], data);
					CAN_report_anticogging(index, data);
					
					controller->input_position += (1.0f / COGGING_MAP_NUM);
					index ++;
					if(index >= COGGING_MAP_NUM){
						index --;
						controller->input_position -= (1.0f / COGGING_MAP_NUM);
						AntiCoggingStep = AS_BACKWARD_LOOP;
					}
				}
			}
			break;
			
		case AS_BACKWARD_LOOP:
			{
				// 10 Hz
				if(SYSTICK_get_ms_since(tick) < 100){
					return;
				}
				tick = SYSTICK_get_tick();
				
				float pos_err = controller->input_position - Encoder.pos_estimate_;
				if(fabs(pos_err) < UsrConfig.anticogging_pos_threshold && fabs(Encoder.vel_estimate_) < UsrConfig.anticogging_vel_threshold){
					pCoggingMap->map[index] += CONTROLLER_get_integrator_current();
					DEBUG("B %d %f\n\r", index, pCoggingMap->map[index]);
				
					uint8_t data[4];
					float_to_data(pCoggingMap->map[index], data);
					CAN_report_anticogging(index+10000, data);
					
					controller->input_position -= (1.0f / COGGING_MAP_NUM);
					index --;
					if(index < 0){
						AntiCoggingStep = AS_END;
					}
				}
			}
			break;
		
		case AS_END:
			for(index=0; index<COGGING_MAP_NUM; index++){
				pCoggingMap->map[index] /= 2.0f;
			}
			AnticoggingValid = true;
			AntiCoggingStep = AS_NULL;
			FSM_input(CMD_MENU);
			break;
		
		default:
			break;
	}
}
