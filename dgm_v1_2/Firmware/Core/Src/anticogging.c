#include "anticogging.h"
#include "systick.h"
#include "usr_config.h"
#include "encoder.h"
#include "arm_math.h"
#include "heap.h"
#include "fdcan.h"
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

void ANTICOGGING_abort(void)
{
	USR_CONFIG_set_default_cogging_map();
	AntiCoggingStep = AS_NULL;
}

bool ANTICOGGING_is_running(void)
{
	return (AntiCoggingStep != AS_NULL);
}

void ANTICOGGING_loop(ControllerStruct *controller)
{
	static int index = 0;
	static uint32_t tick = 0;
	
	if(AntiCoggingStep == AS_NULL){
		return;
	}
	
	// 200 Hz
	if(SYSTICK_get_ms_since(tick) < 5){
		return;
	}
	tick = SYSTICK_get_tick();
	
	switch(AntiCoggingStep){
		case AS_START:
			{
				float position = Encoder.position;
				float err = position - (int)position;
				if(err > 0.001f){
					controller->input_pos -= 0.001f;
				}else if(err < -0.001f){
					controller->input_pos += 0.001f;
				}else{
					controller->input_pos -= err;
					index = 0;
					AntiCoggingStep = AS_FORWARD_LOOP;
				}
			}
			break;
		
		case AS_FORWARD_LOOP:
			{
				float pos_err = controller->input_pos - Encoder.position;
				if(fabs(pos_err) < UsrConfig.anticogging_pos_threshold && fabs(Encoder.velocity) < UsrConfig.anticogging_vel_threshold){
					pCoggingMap->map[index] = CONTROLLER_get_integrator_current();
					DEBUG("F %d %f\n\r", index, pCoggingMap->map[index]);
					
					uint8_t data[4];
					float_to_data(pCoggingMap->map[index], data);
					FDCAN_report_anticogging(index, data);
					
					controller->input_pos += (1.0f / COGGING_MAP_NUM);
					index ++;
					if(index >= COGGING_MAP_NUM){
						index --;
						controller->input_pos -= (1.0f / COGGING_MAP_NUM);
						AntiCoggingStep = AS_BACKWARD_LOOP;
					}
				}
			}
			break;
			
		case AS_BACKWARD_LOOP:
			{
				float pos_err = controller->input_pos - Encoder.position;
				if(fabs(pos_err) < UsrConfig.anticogging_pos_threshold && fabs(Encoder.velocity) < UsrConfig.anticogging_vel_threshold){
					pCoggingMap->map[index] += CONTROLLER_get_integrator_current();
					DEBUG("B %d %f\n\r", index, pCoggingMap->map[index]);
				
					uint8_t data[4];
					float_to_data(pCoggingMap->map[index], data);
					FDCAN_report_anticogging(index+10000, data);
					
					controller->input_pos -= (1.0f / COGGING_MAP_NUM);
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
			FSM_input(CMD_MENU);
			AntiCoggingStep = AS_NULL;
			break;
		
		default:
			break;
	}
}
