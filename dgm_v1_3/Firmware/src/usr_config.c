#include "usr_config.h"
#include <string.h>
#include "util.h"
#include "controller.h"

tUsrConfig UsrConfig;

#define FMC_PAGE_SIZE           		((uint16_t)0x400U)	// 1KB
#define USR_CONFIG_ROM_ADDR				((uint32_t)(0x8000000 + 60 * FMC_PAGE_SIZE))	// Page 60

void USR_CONFIG_set_default_config(void)
{
	// Motor
	UsrConfig.motor_pole_pairs = 7;
	UsrConfig.motor_phase_resistance = 0.05f;
	UsrConfig.motor_phase_inductance = 0.000030f;
	UsrConfig.inertia = 0.001f;
	
	// Calib
	UsrConfig.calib_valid = 0;
	UsrConfig.calib_current = 10.0f;
	UsrConfig.calib_max_voltage = 3.0f;

	// Control
	UsrConfig.control_mode = CONTROL_MODE_POSITION_TRAP;
	UsrConfig.current_ramp_rate = 2.0f;
	UsrConfig.vel_ramp_rate = 50.0f;
	UsrConfig.traj_vel = 50;
	UsrConfig.traj_accel = 100;
	UsrConfig.traj_decel = 100;
	UsrConfig.pos_gain = 160.0f;
	UsrConfig.vel_gain = 0.8f;
	UsrConfig.vel_integrator_gain = 0.08f;
	UsrConfig.vel_limit = 90;
	UsrConfig.current_limit = 20;
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
