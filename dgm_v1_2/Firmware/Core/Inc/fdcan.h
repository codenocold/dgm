#ifndef __FDCAN_H__
#define __FDCAN_H__

#include "main.h"
#include "stm32g4xx_hal_fdcan.h"

typedef enum eCanCmd{
	CAN_CMD_MOTOR_DISABLE = 0,
	CAN_CMD_MOTOR_ENABLE,
	
	CAN_CMD_ERROR_REPORT,
	CAN_CMD_ERROR_RESET,
	
	CAN_CMD_GET_STAT,
	
	CAN_CMD_CALIBRATION_START,
	CAN_CMD_CALIBRATION_REPORT,
	CAN_CMD_CALIBRATION_ABORT,
	CAN_CMD_ANTICOGGING_START,
	CAN_CMD_ANTICOGGING_REPORT,
	CAN_CMD_ANTICOGGING_ABORT,
	
	CAN_CMD_SYNC,
	
	CAN_CMD_SET_TARGET_POSITION,
	CAN_CMD_SET_TARGET_VELOCITY,
	CAN_CMD_SET_TARGET_TORQUE,
	
	CAN_CMD_GET_POSITION,
	CAN_CMD_GET_VELOCITY,
	CAN_CMD_GET_TORQUE,
	CAN_CMD_GET_IQ,
	CAN_CMD_GET_VBUS,
	CAN_CMD_GET_IBUS,
	
	CAN_CMD_SET_CONFIG,
	CAN_CMD_GET_CONFIG,
	CAN_CMD_UPDATE_CONFIGS,
	
	CAN_CMD_GET_FW_VERSION,
	
	CAN_CMD_HEARTBEAT,
}tCanCmd;

typedef enum eCanConfigs{
	CAN_CONFIG_POLE_PAIRS = 1,
	CAN_CONFIG_PHASE_RESISTANCE,
	CAN_CONFIG_PHASE_INDUCTANCE,
	CAN_CONFIG_INERTIA,
	CAN_CONFIG_TORQUE_CONSTANT,
	CAN_CONFIG_ENCODER_DIR_REV,
	CAN_CONFIG_ENCODER_OFFSET,
	CAN_CONFIG_CALIB_VALID,
	CAN_CONFIG_CALIB_CURRENT,
	CAN_CONFIG_CALIB_MAX_VOLTAGE,
	CAN_CONFIG_ANTICOGGING_ENABLE,
	CAN_CONFIG_ANTICOGGING_POS_THRESHOLD,
	CAN_CONFIG_ANTICOGGING_VEL_THRESHOLD,
	CAN_CONFIG_INPUT_MODE,
	CAN_CONFIG_CONTROL_MODE,
	CAN_CONFIG_TORQUE_RAMP_RATE,
	CAN_CONFIG_VEL_RAMP_RATE,
	CAN_CONFIG_INPUT_POS_FILTER_BANDWIDTH,
	CAN_CONFIG_TRAJ_VEL,
	CAN_CONFIG_TRAJ_ACCEL,
	CAN_CONFIG_TRAJ_DECEL,
	CAN_CONFIG_GAIN_SCHEDULING_ENABLE,
	CAN_CONFIG_GAIN_SCHEDULING_WIDTH,
	CAN_CONFIG_POS_GAIN,
	CAN_CONFIG_VEL_GAIN,
	CAN_CONFIG_VEL_INTEGRATOR_GAIN,
	CAN_CONFIG_VEL_LIMIT,
	CAN_CONFIG_CURRENT_LIMIT,
	CAN_CONFIG_CURRENT_CTRL_P_GAIN,
	CAN_CONFIG_CURRENT_CTRL_I_GAIN,
	CAN_CONFIG_CURRENT_CTRL_BW,
	CAN_CONFIG_PROTECT_UNDER_VOLTAGE,
	CAN_CONFIG_PROTECT_OVER_VOLTAGE,
	CAN_CONFIG_PROTECT_OVER_SPEED,
	CAN_CONFIG_CAN_ID,
	CAN_CONFIG_CAN_TIMEOUT_MS,
	CAN_CONFIG_CAN_SYNC_TARGET_ENABLE,
} tCanConfigs;

extern FDCAN_HandleTypeDef hfdcan1;

int FDCAN_init(void);
void FDCAN_report_error(int32_t ecode);
void FDCAN_report_calibration(int step, uint8_t* data);
void FDCAN_report_anticogging(int step, uint8_t* data);
void FDCAN_reset_timeout(void);
void FDCAN_timeout_check_loop(void);
void FDCAN_rx_callback(void);

#endif /* __FDCAN_H__ */
