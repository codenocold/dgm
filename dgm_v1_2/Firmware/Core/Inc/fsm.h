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
#define CMD_UART_ENCODER  		'e'

// Status
typedef enum eFsmStat {
	FS_STARTUP = 0,
	FS_MENU_MODE,
	FS_MOTOR_MODE,
	FS_CALIBRATION_MODE,
	FS_ANTICOGGING_MODE,
	
	FS_UART_SETUP,
	FS_UART_ENCODER,
} tFsmStat;

// Errors
#define ERR_OVER_VOLTAGE	((int)1<<30)
#define	ERR_UNDER_VOLTAGE	((int)1<<29)
#define	ERR_OVER_SPEED		((int)1<<28)

#define	ERR_DRV_FAULT		((int)1<<21)
#define	ERR_DRV_VDS_OCP		((int)1<<20)
#define	ERR_DRV_GDF			((int)1<<19)	
#define	ERR_DRV_UVLO		((int)1<<18)	
#define	ERR_DRV_OTSD		((int)1<<17)
#define	ERR_DRV_VDS_HA		((int)1<<16)
#define	ERR_DRV_VDS_LA		((int)1<<15)
#define	ERR_DRV_VDS_HB		((int)1<<14)
#define	ERR_DRV_VDS_LB		((int)1<<13)
#define	ERR_DRV_VDS_HC		((int)1<<12)
#define	ERR_DRV_VDS_LC		((int)1<<11)
#define	ERR_DRV_SA_OC		((int)1<<10)
#define	ERR_DRV_SB_OC		((int)1<<09)
#define	ERR_DRV_SC_OC		((int)1<<08)
#define	ERR_DRV_OTW			((int)1<<07)
#define	ERR_DRV_CPUV		((int)1<<06)
#define	ERR_DRV_VGS_HA		((int)1<<05)
#define	ERR_DRV_VGS_LA		((int)1<<04)
#define	ERR_DRV_VGS_HB		((int)1<<03)
#define	ERR_DRV_VGS_LB		((int)1<<02)
#define	ERR_DRV_VGS_HC		((int)1<<01)
#define	ERR_DRV_VGS_LC		((int)1<<00)

int FSM_input(char c);
void FSM_loop(void);
int FSM_get_error(void);
tFsmStat FSM_get_stat(void);

#endif
