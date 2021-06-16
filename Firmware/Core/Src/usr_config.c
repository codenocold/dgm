#include "usr_config.h"
#include <string.h>
#include "flash.h"
#include "util.h"
#include "heap.h"
#include "anticogging.h"
#include "controller.h"

tUsrConfig UsrConfig;
tCoggingMap *pCoggingMap = NULL;

#define USR_CONFIG_COGGING_MAP_ROM_ADDR	((uint32_t)(0x8000000 + 55 * FLASH_PAGE_SIZE))	// Page 55
#define USR_CONFIG_ROM_ADDR				((uint32_t)(0x8000000 + 60 * FLASH_PAGE_SIZE))	// Page 60

void USR_CONFIG_set_default_config(void)
{
	// Motor
	UsrConfig.pole_pairs = 7;
	UsrConfig.phase_resistance = 0.043198f;
	UsrConfig.phase_inductance = 0.000007f;
	UsrConfig.inertia = 0.001f;				// [Nm/(turn/s^2)]
	UsrConfig.torque_constant = 0.06f;		// [Nm/A]
	
	// Encoder
	UsrConfig.encoder_dir_rev = 0;
	
	// Calib
	UsrConfig.calib_valid = 0;
	UsrConfig.calib_current = 10.0f;
	UsrConfig.calib_max_voltage = 2.0f;
	
	// Anticogging
	UsrConfig.anticogging_enable = 0;
	UsrConfig.anticogging_pos_threshold = 0.002f;
	UsrConfig.anticogging_vel_threshold = 0.01f;

	// Control
	UsrConfig.input_mode = INPUT_MODE_TRAP_TRAJ;
	UsrConfig.control_mode = CONTROL_MODE_POSITION_CONTROL;
	UsrConfig.torque_ramp_rate = 0.01f;
	UsrConfig.vel_ramp_rate = 50.0f;
	UsrConfig.input_pos_filter_bandwidth = 10;
	UsrConfig.traj_vel = 50;
	UsrConfig.traj_accel = 100;
	UsrConfig.traj_decel = 100;
	UsrConfig.gain_scheduling_enable = 0;
	UsrConfig.gain_scheduling_width = 0.0001f;
	UsrConfig.pos_gain = 260.0f;
	UsrConfig.vel_gain = 0.15f;
	UsrConfig.vel_integrator_gain = 0.02f;
	UsrConfig.vel_limit = 90;
	UsrConfig.current_limit = 20;
	UsrConfig.current_ctrl_bandwidth = 1000;
	
	UsrConfig.protect_under_voltage = 12;
	UsrConfig.protect_over_voltage  = 45;
	UsrConfig.protect_over_speed = 100;
	
	// Can
	UsrConfig.can_id = 1;
	UsrConfig.can_timeout_ms = 0;
	UsrConfig.can_sync_target_enable = 0;
}

void USR_CONFIG_set_default_cogging_map(void)
{
	if(pCoggingMap == NULL){
		pCoggingMap = HEAP_malloc(sizeof(tCoggingMap));
	}
	
	for(int i=0; i<COGGING_MAP_NUM; i++){
		pCoggingMap->map[i] = 0;
	}
}

int USR_CONFIG_read_config(void)
{
	int state = 0;

	memcpy(&UsrConfig, (uint8_t*)USR_CONFIG_ROM_ADDR, sizeof(tUsrConfig));

	uint32_t crc;
	crc = crc32((uint8_t*)&UsrConfig, sizeof(tUsrConfig)-4, 0);
	if(crc != UsrConfig.crc){
		state = -1;
	}
	
	return state;
}

int USR_CONFIG_save_config(void)
{
	int status = 0;
	
	uint32_t primask = cpu_enter_critical();
	
    FLASH_unlock();
	
	// Erase
	status = FLASH_erase_page(60, 1);
	
	if(status == 0){
		UsrConfig.crc = crc32((uint8_t*)&UsrConfig, sizeof(tUsrConfig)-4, 0);
		uint64_t* pData = (uint64_t*)&UsrConfig;
		for(int i=0; i<sizeof(tUsrConfig)/8; i++){
			status = FLASH_program(USR_CONFIG_ROM_ADDR+i*8, *(pData+i));
			if(status != 0){
				break;
			}
		}
	}
	
	FLASH_lock();
	
	cpu_exit_critical(primask);
	
	return status;
}

int USR_CONFIG_read_cogging_map(void)
{
	int state = 0;
	
	if(pCoggingMap == NULL){
		pCoggingMap = HEAP_malloc(sizeof(tCoggingMap));
	}
	
	memcpy(pCoggingMap, (uint8_t*)USR_CONFIG_COGGING_MAP_ROM_ADDR, sizeof(tCoggingMap));
	
	uint32_t crc;
	crc = crc32((uint8_t*)pCoggingMap, sizeof(tCoggingMap)-4, 0);
	if(crc != pCoggingMap->crc){
		state = -1;
	}
	
	return state;
}

int USR_CONFIG_save_cogging_map(void)
{
	int status = 0;
	
	if(pCoggingMap == NULL){
		return -1;
	}
	
	uint32_t primask = cpu_enter_critical();
	
    FLASH_unlock();
	
	// Erase
	status = FLASH_erase_page(55, 5);
	
	if(status == 0){
		pCoggingMap->crc = crc32((uint8_t*)pCoggingMap, sizeof(tCoggingMap)-4, 0);
		uint64_t* pData = (uint64_t*)pCoggingMap;
		for(int i=0; i<sizeof(tCoggingMap)/8; i++){
			status = FLASH_program(USR_CONFIG_COGGING_MAP_ROM_ADDR+i*8, *(pData+i));
			if(status != 0){
				break;
			}
		}
	}
	
	FLASH_lock();
	
	cpu_exit_critical(primask);
	
	return status;
}
