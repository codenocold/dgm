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

#include "usr_config.h"
#include <string.h>
#include "util.h"
#include "heap.h"
#include "controller.h"

tUsrConfig UsrConfig;
tCoggingMap *pCoggingMap = NULL;

#define FMC_PAGE_SIZE           		((uint16_t)0x400U)	// 1KB
#define USR_CONFIG_ROM_ADDR				((uint32_t)(0x8000000 + 60 * FMC_PAGE_SIZE))	// Page 60
#define USR_CONFIG_COGGING_MAP_ROM_ADDR	((uint32_t)(0x8000000 + 70 * FMC_PAGE_SIZE))	// Page 70

void USR_CONFIG_set_default_config(void)
{
	// Motor
	UsrConfig.motor_pole_pairs = 7;
	UsrConfig.motor_phase_resistance = 0.05f;
	UsrConfig.motor_phase_inductance = 0.000015f;
	UsrConfig.inertia = 0.001f;
	
	// Calib
	UsrConfig.calib_valid = 0;
	UsrConfig.calib_current = 10.0f;
	UsrConfig.calib_max_voltage = 3.0f;
	
	// Anticogging
	UsrConfig.anticogging_enable = 1;
	UsrConfig.anticogging_pos_threshold = 0.001f;
	UsrConfig.anticogging_vel_threshold = 0.1f;

	// Control
	UsrConfig.control_mode = CONTROL_MODE_POSITION_TRAP;
	UsrConfig.current_ramp_rate = 2.0f;
	UsrConfig.vel_ramp_rate = 50.0f;
	UsrConfig.traj_vel = 30;
	UsrConfig.traj_accel = 100;
	UsrConfig.traj_decel = 100;
	UsrConfig.pos_gain = 120.0f;
	UsrConfig.vel_gain = 2.0f;
	UsrConfig.vel_integrator_gain = 0.2f;
	UsrConfig.vel_limit = 90;
	UsrConfig.current_limit = 20;
	UsrConfig.current_ctrl_p_gain = 0;
	UsrConfig.current_ctrl_i_gain = 0;
	UsrConfig.current_ctrl_bandwidth = 1000;
	
	// Protect
	UsrConfig.protect_under_voltage = 12;
	UsrConfig.protect_over_voltage  = 40;
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
	fmc_state_enum status;
	
	uint32_t primask = cpu_enter_critical();
	
    fmc_unlock();
	
	// Erase
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
	status = fmc_page_erase(USR_CONFIG_ROM_ADDR);
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
	
	if(status == FMC_READY){
		// Program
		UsrConfig.crc = crc32((uint8_t*)&UsrConfig, sizeof(tUsrConfig)-4, 0);
		uint32_t* pData = (uint32_t*)&UsrConfig;
		for(int i=0; i<sizeof(tUsrConfig)/4; i++){
			status = fmc_word_program(USR_CONFIG_ROM_ADDR+i*4, *(pData+i));
			fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR );
			if(status != FMC_READY){
				break;
			}
		}
	}
	
	fmc_lock();
	
	cpu_exit_critical(primask);
	
	return (status != FMC_READY);
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
	
    fmc_unlock();
	
	// Erase
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
	for(int i=0; i<12; i++){
		status += fmc_page_erase(USR_CONFIG_COGGING_MAP_ROM_ADDR + FMC_PAGE_SIZE * i);
		fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
	}
	
	if(status == 0){
		pCoggingMap->crc = crc32((uint8_t*)pCoggingMap, sizeof(tCoggingMap)-4, 0);
		uint32_t* pData = (uint32_t*)pCoggingMap;
		for(int i=0; i<sizeof(tCoggingMap)/4; i++){
			status = fmc_word_program(USR_CONFIG_COGGING_MAP_ROM_ADDR+i*4, *(pData+i));
			fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR );
			if(status != FMC_READY){
				break;
			}
		}
	}
	
	fmc_lock();
	
	cpu_exit_critical(primask);
	
	return status;
}
