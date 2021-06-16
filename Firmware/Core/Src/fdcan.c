#include "fdcan.h"
#include "fsm.h"
#include "foc.h"
#include "anticogging.h"
#include "encoder.h"
#include "controller.h"
#include "usr_config.h"
#include "drv8323.h"
#include "util.h"

FDCAN_HandleTypeDef hfdcan1;

static volatile uint32_t mRecTick = 0;

static float mPosition;
static float mVelocity;
static float mTorque;

static void sync_callback(void);
static void config_callback(uint32_t frameID, uint8_t* data, bool isSet);

int FDCAN_init(void)
{
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = DISABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	hfdcan1.Init.NominalPrescaler = 34;
	hfdcan1.Init.NominalSyncJumpWidth = 1;
	hfdcan1.Init.NominalTimeSeg1 = 3;
	hfdcan1.Init.NominalTimeSeg2 = 1;
	hfdcan1.Init.DataPrescaler = 1;
	hfdcan1.Init.DataSyncJumpWidth = 1;
	hfdcan1.Init.DataTimeSeg1 = 1;
	hfdcan1.Init.DataTimeSeg2 = 1;
	hfdcan1.Init.StdFiltersNbr = 0;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK){
		return -1;
	}
	
	HAL_FDCAN_Start(&hfdcan1);
	
	HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	
	return 0;
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_FDCAN);
	
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	/**FDCAN GPIO Configuration
	PA11   ------> FDCAN_RX
	PA12   ------> FDCAN_TX
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_9;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_9;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* FDCAN1 interrupt Init */
	NVIC_SetPriority(FDCAN1_IT0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
	NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

static inline void can_send(uint32_t frameID, uint8_t* pData, uint8_t len)
{
	FDCAN_TxHeaderTypeDef header;
	header.BitRateSwitch = FDCAN_BRS_OFF;
	header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	header.MessageMarker = 0;
	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	header.IdType = FDCAN_STANDARD_ID;
	header.FDFormat = FDCAN_CLASSIC_CAN;
	header.TxFrameType = FDCAN_DATA_FRAME;
	
	header.Identifier = frameID;
	header.DataLength = (uint32_t)(len<<16);
	
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, pData);
}

void FDCAN_report_error(int32_t ecode)
{
	uint8_t data[4];
	uint32_t frameID = (CAN_CMD_ERROR_REPORT << 4) | (UsrConfig.can_id & 0xF);
	int_to_data(ecode, data);
	can_send(frameID, data, 4);
}

void FDCAN_report_calibration(int step, uint8_t* data)
{
	uint8_t send_data[8];
	
	uint32_t frameID = (CAN_CMD_CALIBRATION_REPORT << 4) | (UsrConfig.can_id & 0xF);
	
	int_to_data(step, send_data);
	send_data[4] = data[0];
	send_data[5] = data[1];
	send_data[6] = data[2];
	send_data[7] = data[3];
	can_send(frameID, send_data, 8);
}

void FDCAN_report_anticogging(int step, uint8_t* data)
{
	uint8_t send_data[8];
	
	uint32_t frameID = (CAN_CMD_ANTICOGGING_REPORT << 4) | (UsrConfig.can_id & 0xF);
	
	int_to_data(step, send_data);
	send_data[4] = data[0];
	send_data[5] = data[1];
	send_data[6] = data[2];
	send_data[7] = data[3];
	can_send(frameID, send_data, 8);
}

void FDCAN_reset_timeout(void)
{
	mRecTick = SYSTICK_get_tick();
}

void FDCAN_timeout_check_loop(void)
{
	if(UsrConfig.can_timeout_ms == 0){
		return;
	}
	
	if(SYSTICK_get_ms_since(mRecTick) > UsrConfig.can_timeout_ms){
		FSM_input(CMD_MENU);
	}
}

void FDCAN_rx_callback(void)
{
	uint8_t data[8];
	FDCAN_RxHeaderTypeDef rx_header;
	
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, data);
	
	// Frame
    // CMD    | nodeID
    // 7 bits | 4 bits
	uint32_t frameID = rx_header.Identifier;
	uint32_t cmd = (frameID >> 4);
    uint32_t nodeID = (frameID & 0xF);
	//uint8_t rx_len = rx_header.DataLength >> 16;
    
	if(nodeID != UsrConfig.can_id && nodeID != 0){
		return;
	}
	
	if(UsrConfig.can_timeout_ms != 0){
		mRecTick = SYSTICK_get_tick();
	}
	
	switch(cmd){
		case CAN_CMD_MOTOR_DISABLE:
			int_to_data(FSM_input(CMD_MENU), data);
			can_send(frameID, data, 4);
			break;
		
		case CAN_CMD_MOTOR_ENABLE:
			int_to_data(FSM_input(CMD_MOTOR), data);
			float_to_data(Encoder.position, &data[4]);
			can_send(frameID, data, 8);
			break;
		
		case CAN_CMD_ERROR_RESET:
			int_to_data(FSM_input(CMD_RESET_ERROR), data);
			can_send(frameID, data, 4);
			break;
		
		case CAN_CMD_GET_STAT:
			int_to_data(FSM_get_stat(), data);
			int_to_data(FSM_get_error(), &data[4]);
			can_send(frameID, data, 8);
			break;
		
		case CAN_CMD_CALIBRATION_START:
			int_to_data(FSM_input(CMD_CALIBRATION), data);
			can_send(frameID, data, 4);
			break;
		
		case CAN_CMD_CALIBRATION_ABORT:
			int_to_data(FSM_input(CMD_MENU), data);
			can_send(frameID, data, 4);
			break;
		
		case CAN_CMD_ANTICOGGING_START:
			int_to_data(FSM_input(CMD_ANTICOGGING), data);
			can_send(frameID, data, 4);
			break;
			
		case CAN_CMD_ANTICOGGING_ABORT:
			int_to_data(FSM_input(CMD_MENU), data);
			can_send(frameID, data, 4);
			break;

		case CAN_CMD_SYNC:
			if(UsrConfig.can_sync_target_enable){
				sync_callback();
			}
			break;
			
		case CAN_CMD_SET_TARGET_POSITION:
			mPosition = *(float*)(&data[0]);
			if(!UsrConfig.can_sync_target_enable){
				sync_callback();
			}
			break;
		
		case CAN_CMD_SET_TARGET_VELOCITY:
			mVelocity = *(float*)(&data[0]);
			if(!UsrConfig.can_sync_target_enable){
				sync_callback();
			}
			break;
		
		case CAN_CMD_SET_TARGET_TORQUE:
			mTorque = *(float*)(&data[0]);
			if(!UsrConfig.can_sync_target_enable){
				sync_callback();
			}
			break;
		
		case CAN_CMD_GET_POSITION:
			float_to_data(Encoder.position, data);
			can_send(frameID, data, 4);
			break;
		
		case CAN_CMD_GET_VELOCITY:
			float_to_data(Encoder.velocity, data);
			can_send(frameID, data, 4);
			break;
		
		case CAN_CMD_GET_TORQUE:
			float_to_data(Foc.i_q_filt * UsrConfig.torque_constant, data);
			can_send(frameID, data, 4);
			break;
		
		case CAN_CMD_GET_IQ:
			float_to_data(Foc.i_q_filt, data);
			can_send(frameID, data, 4);
			break;
		
		case CAN_CMD_GET_VBUS:
			float_to_data(Foc.v_bus, data);
			can_send(frameID, data, 4);
			break;
		
		case CAN_CMD_GET_IBUS:
			float_to_data(Foc.i_bus_filt, data);
			can_send(frameID, data, 4);
			break;
		
		case CAN_CMD_SET_CONFIG:
			config_callback(frameID, data, true);
			break;
		
		case CAN_CMD_GET_CONFIG:
			config_callback(frameID, data, false);
			break;
		
		case CAN_CMD_UPDATE_CONFIGS:
			int_to_data(FSM_input(CMD_UPDATE_CONFIGS), data);
			can_send(frameID, data, 4);
			break;
		
		case CAN_CMD_GET_FW_VERSION:
			int_to_data(FW_VERSION_MAJOR, &data[0]);
			int_to_data(FW_VERSION_MINOR, &data[4]);
			can_send(frameID, data, 8);
			break;
		
		default:
			break;
	}
}

static void sync_callback(void)
{
	if(FSM_get_stat() != FS_MOTOR_MODE){
		return;
	}
	
	switch(UsrConfig.input_mode){
		case INPUT_MODE_PASSTHROUGH:
			switch(UsrConfig.control_mode){
				case CONTROL_MODE_TORQUE_CONTROL:
					Controller.input_torque = mTorque;
					break;
				
				case CONTROL_MODE_VELOCITY_CONTROL:
					Controller.input_vel = mVelocity;
					break;
				
				case CONTROL_MODE_POSITION_CONTROL:
					Controller.input_pos = mPosition;
					break;
				
				default:
					break;
			}
			break;
		
		case INPUT_MODE_TORQUE_RAMP:
			Controller.input_torque = mTorque;
			break;
		
		case INPUT_MODE_VEL_RAMP:
			Controller.input_vel = mVelocity;
			break;
		
		case INPUT_MODE_POS_FILTER:
			Controller.input_pos = mPosition;
			break;
		
		case INPUT_MODE_TRAP_TRAJ:
			CONTROLLER_move_to_pos(mPosition);
			break;
		
		default:
			break;
	}
}

static void config_callback(uint32_t frameID, uint8_t* data, bool isSet)
{
	switch(data_to_int(data)){
		case CAN_CONFIG_POLE_PAIRS:
			if(isSet){
				UsrConfig.pole_pairs = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.pole_pairs, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_PHASE_RESISTANCE:
			if(isSet){
				UsrConfig.phase_resistance = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.phase_resistance, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_PHASE_INDUCTANCE:
			if(isSet){
				UsrConfig.phase_inductance = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.phase_inductance, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_INERTIA:
			if(isSet){
				UsrConfig.inertia = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.inertia, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_TORQUE_CONSTANT:
			if(isSet){
				UsrConfig.torque_constant = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.torque_constant, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_ENCODER_DIR_REV:
			if(isSet){
				UsrConfig.encoder_dir_rev = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.encoder_dir_rev, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_ENCODER_OFFSET:
			if(isSet){
				UsrConfig.encoder_offset = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.encoder_offset, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_CALIB_VALID:
			if(isSet){
				UsrConfig.calib_valid = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.calib_valid, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_CALIB_CURRENT:
			if(isSet){
				UsrConfig.calib_current = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.calib_current, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_CALIB_MAX_VOLTAGE:
			if(isSet){
				UsrConfig.calib_max_voltage = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.calib_max_voltage, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_ANTICOGGING_ENABLE:
			if(isSet){
				UsrConfig.anticogging_enable = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.anticogging_enable, &data[4]);
			can_send(frameID, data, 8);
			break;
		
		case CAN_CONFIG_ANTICOGGING_POS_THRESHOLD:
			if(isSet){
				UsrConfig.anticogging_pos_threshold = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.anticogging_pos_threshold, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_ANTICOGGING_VEL_THRESHOLD:
			if(isSet){
				UsrConfig.anticogging_vel_threshold = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.anticogging_vel_threshold, &data[4]);
			can_send(frameID, data, 8);
			break;
		
		case CAN_CONFIG_INPUT_MODE:
			if(isSet){
				UsrConfig.input_mode = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.input_mode, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_CONTROL_MODE:
			if(isSet){
				UsrConfig.control_mode = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.control_mode, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_TORQUE_RAMP_RATE:
			if(isSet){
				UsrConfig.torque_ramp_rate = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.torque_ramp_rate, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_VEL_RAMP_RATE:
			if(isSet){
				UsrConfig.vel_ramp_rate = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.vel_ramp_rate, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_INPUT_POS_FILTER_BANDWIDTH:
			if(isSet){
				UsrConfig.input_pos_filter_bandwidth = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.input_pos_filter_bandwidth, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_TRAJ_VEL:
			if(isSet){
				UsrConfig.traj_vel = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.traj_vel, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_TRAJ_ACCEL:
			if(isSet){
				UsrConfig.traj_accel = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.traj_accel, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_TRAJ_DECEL:
			if(isSet){
				UsrConfig.traj_decel = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.traj_decel, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_GAIN_SCHEDULING_ENABLE:
			if(isSet){
				UsrConfig.gain_scheduling_enable = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.gain_scheduling_enable, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_GAIN_SCHEDULING_WIDTH:
			if(isSet){
				UsrConfig.gain_scheduling_width = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.gain_scheduling_width, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_POS_GAIN:
			if(isSet){
				UsrConfig.pos_gain = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.pos_gain, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_VEL_GAIN:
			if(isSet){
				UsrConfig.vel_gain = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.vel_gain, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_VEL_INTEGRATOR_GAIN:
			if(isSet){
				UsrConfig.vel_integrator_gain = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.vel_integrator_gain, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_VEL_LIMIT:
			if(isSet){
				UsrConfig.vel_limit = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.vel_limit, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_CURRENT_LIMIT:
			if(isSet){
				UsrConfig.current_limit = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.current_limit, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_CURRENT_CTRL_P_GAIN:
			if(isSet){
				UsrConfig.current_ctrl_p_gain = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.current_ctrl_p_gain, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_CURRENT_CTRL_I_GAIN:
			if(isSet){
				UsrConfig.current_ctrl_i_gain = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.current_ctrl_i_gain, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_CURRENT_CTRL_BW:
			if(isSet){
				UsrConfig.current_ctrl_bandwidth = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.current_ctrl_bandwidth, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_PROTECT_UNDER_VOLTAGE:
			if(isSet){
				UsrConfig.protect_under_voltage = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.protect_under_voltage, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_PROTECT_OVER_VOLTAGE:
			if(isSet){
				UsrConfig.protect_over_voltage = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.protect_over_voltage, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_PROTECT_OVER_SPEED:
			if(isSet){
				UsrConfig.protect_over_speed = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.protect_over_speed, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_CAN_ID:
			if(isSet){
				UsrConfig.can_id = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.can_id, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_CAN_TIMEOUT_MS:
			if(isSet){
				UsrConfig.can_timeout_ms = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.can_timeout_ms, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_CAN_SYNC_TARGET_ENABLE:
			if(isSet){
				UsrConfig.can_sync_target_enable = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.can_sync_target_enable, &data[4]);
			can_send(frameID, data, 8);
			break;
		
		default:
			break;
	}
}
