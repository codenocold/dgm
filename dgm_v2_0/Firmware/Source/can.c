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

#include "can.h"
#include "fsm.h"
#include "foc.h"
#include "encoder.h"
#include "controller.h"
#include "usr_config.h"
#include "util.h"
#include "systick.h"

static volatile uint32_t mRecTick = 0;

static float mPosition;
static float mVelocity;
static float mCurrent;

static void sync_callback(void);
static void config_callback(uint32_t frameID, uint8_t* data, bool isSet);

void CAN_init(void)
{
	can_parameter_struct can_parameter;
	
	/* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    
    /* configure CAN0 GPIO */
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_11);	// CAN_RX
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);	// CAN_TX
	
	can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
	can_deinit(CAN0);
	
	// Reset CAN peripheral
	CAN_CTL(CAN0) |= CAN_CTL_SWRST;
	while((CAN_CTL(CAN0) & CAN_CTL_SWRST) != 0);		// reset bit is set to zero after reset
	while((CAN_STAT(CAN0) & CAN_STAT_SLPWS) == 0);	    // should be in sleep mode after reset
	
	can_parameter.working_mode = CAN_NORMAL_MODE;
	can_parameter.auto_retrans = DISABLE;
	can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = ENABLE;
	can_parameter.time_triggered = DISABLE;
	
	// Baudrate: 1M Samplepoint 75%
	int brp = 3;
	int sjw = 1;
	int seg1 = 14;
	int seg2 = 5;
	can_parameter.prescaler = brp;
    can_parameter.resync_jump_width = sjw - 1;
    can_parameter.time_segment_1 = seg1 - 1;
    can_parameter.time_segment_2 = seg2 - 1;

	can_init(CAN0, &can_parameter);
	
	/* initialize filter */
    can1_filter_start_bank(14);
    can_filter_mask_mode_init(0, 0, CAN_EXTENDED_FIFO0, 0);
	can_filter_mask_mode_init(0, 0, CAN_STANDARD_FIFO0, 0);

	/* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 2, 0);
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
}

bool can_send(uint32_t frameID, uint8_t* pData, uint8_t len)
{
	uint8_t mailbox_number = CAN_MAILBOX0;
	
	/* select one empty mailbox */
    if(CAN_TSTAT_TME0 == (CAN_TSTAT(CAN0)&CAN_TSTAT_TME0)){
        mailbox_number = CAN_MAILBOX0;
    }else if(CAN_TSTAT_TME1 == (CAN_TSTAT(CAN0)&CAN_TSTAT_TME1)){
        mailbox_number = CAN_MAILBOX1;
    }else if(CAN_TSTAT_TME2 == (CAN_TSTAT(CAN0)&CAN_TSTAT_TME2)){
        mailbox_number = CAN_MAILBOX2;
    }else{
        mailbox_number = CAN_NOMAILBOX;
    }
    /* return no mailbox empty */
    if(CAN_NOMAILBOX == mailbox_number){
        return false;
    }
	
	/* set transmit mailbox standard identifier */
	CAN_TMI(CAN0, mailbox_number) = (uint32_t)(TMI_SFID(frameID & 0x7FF));
	
	/* set the data length */
	CAN_TMP(CAN0, mailbox_number) &= ~(CAN_TMP_DLENC|CAN_TMP_ESI|CAN_TMP_BRS|CAN_TMP_FDF);
	CAN_TMP(CAN0, mailbox_number) |= len & 0x0F;
	
	/* set the data */
	CAN_TMDATA0(CAN0, mailbox_number) = TMDATA0_DB3(pData[3]) | \
											  TMDATA0_DB2(pData[2]) | \
											  TMDATA0_DB1(pData[1]) | \
											  TMDATA0_DB0(pData[0]);
	CAN_TMDATA1(CAN0, mailbox_number) = TMDATA1_DB7(pData[7]) | \
											  TMDATA1_DB6(pData[6]) | \
											  TMDATA1_DB5(pData[5]) | \
											  TMDATA1_DB4(pData[4]);
	
	/* enable transmission */
    CAN_TMI(CAN0, mailbox_number) |= CAN_TMI_TEN;
	
	return true;
}

bool CAN_receive(CanFrame *rx_frame)
{
	if ((CAN_RFIFO0(CAN0) & CAN_RFIF_RFL_MASK) != 0) {
		rx_frame->can_id = GET_RFIFOMI_SFID(CAN_RFIFOMI(CAN0, 0));

		rx_frame->can_dlc = (uint8_t)(GET_RFIFOMP_DLENC(CAN_RFIFOMP(CAN0, 0)));
		
		rx_frame->data[0] = (uint8_t)(GET_RFIFOMDATA0_DB0(CAN_RFIFOMDATA0(CAN0, 0)));
		rx_frame->data[1] = (uint8_t)(GET_RFIFOMDATA0_DB1(CAN_RFIFOMDATA0(CAN0, 0)));
		rx_frame->data[2] = (uint8_t)(GET_RFIFOMDATA0_DB2(CAN_RFIFOMDATA0(CAN0, 0)));
		rx_frame->data[3] = (uint8_t)(GET_RFIFOMDATA0_DB3(CAN_RFIFOMDATA0(CAN0, 0)));
		rx_frame->data[4] = (uint8_t)(GET_RFIFOMDATA1_DB4(CAN_RFIFOMDATA1(CAN0, 0)));
		rx_frame->data[5] = (uint8_t)(GET_RFIFOMDATA1_DB5(CAN_RFIFOMDATA1(CAN0, 0)));
		rx_frame->data[6] = (uint8_t)(GET_RFIFOMDATA1_DB6(CAN_RFIFOMDATA1(CAN0, 0)));
		rx_frame->data[7] = (uint8_t)(GET_RFIFOMDATA1_DB7(CAN_RFIFOMDATA1(CAN0, 0)));

		CAN_RFIFO0(CAN0) |= CAN_RFIFO0_RFD0; // release FIFO

	    return true;
	} else {
		return false;
	}
}

void CAN_report_error(int32_t ecode)
{
	uint8_t data[4];
	uint32_t frameID = (CAN_CMD_ERROR_REPORT << 4) | (UsrConfig.can_id & 0xF);
	int_to_data(ecode, data);
	can_send(frameID, data, 4);
}

void CAN_report_calibration(int step, uint8_t* data)
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

void CAN_report_anticogging(int step, uint8_t* data)
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

void CAN_reset_timeout(void)
{
	mRecTick = SYSTICK_get_tick();
}

void CAN_timeout_check_loop(void)
{
	if(UsrConfig.can_timeout_ms == 0){
		return;
	}
	
	if(SYSTICK_get_ms_since(mRecTick) > UsrConfig.can_timeout_ms){
		FSM_input(CMD_MENU);
	}
}

void CAN_rx_callback(CanFrame* rx_frame)
{
	// Frame
    // CMD    | nodeID
    // 7 bits | 4 bits
	uint32_t frameID = rx_frame->can_id;
	uint32_t cmd = (frameID >> 4);
    uint32_t nodeID = (frameID & 0xF);

	if(nodeID != UsrConfig.can_id && nodeID != 0){
		return;
	}
	
	if(UsrConfig.can_timeout_ms != 0){
		mRecTick = SYSTICK_get_tick();
	}
	
	switch(cmd){
		case CAN_CMD_MOTOR_DISABLE:
			int_to_data(FSM_input(CMD_MENU), rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
			break;
		
		case CAN_CMD_MOTOR_ENABLE:
			int_to_data(FSM_input(CMD_MOTOR), rx_frame->data);
			float_to_data(Encoder.pos_estimate_, &rx_frame->data[4]);
			can_send(frameID, rx_frame->data, 8);
			break;
		
		case CAN_CMD_ERROR_RESET:
			int_to_data(FSM_input(CMD_RESET_ERROR), rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
			break;
		
		case CAN_CMD_GET_STAT:
			int_to_data(FSM_get_stat(), rx_frame->data);
			int_to_data(FSM_get_error(), &rx_frame->data[4]);
			can_send(frameID, rx_frame->data, 8);
			break;
		
		case CAN_CMD_CALIBRATION_START:
			int_to_data(FSM_input(CMD_CALIBRATION), rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
			break;
		
		case CAN_CMD_CALIBRATION_ABORT:
			int_to_data(FSM_input(CMD_MENU), rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
			break;
		
		case CAN_CMD_ANTICOGGING_START:
			int_to_data(FSM_input(CMD_ANTICOGGING), rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
			break;
		
		case CAN_CMD_ANTICOGGING_ABORT:
			int_to_data(FSM_input(CMD_MENU), rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
			break;

		case CAN_CMD_SYNC:
			if(UsrConfig.can_sync_target_enable){
				sync_callback();
			}
			break;
			
		case CAN_CMD_SET_TARGET_POSITION:
			mPosition = data_to_float(&rx_frame->data[0]);
			if(!UsrConfig.can_sync_target_enable){
				sync_callback();
			}
			break;
		
		case CAN_CMD_SET_TARGET_VELOCITY:
			mVelocity = data_to_float(&rx_frame->data[0]);
			if(!UsrConfig.can_sync_target_enable){
				sync_callback();
			}
			break;
		
		case CAN_CMD_SET_TARGET_CURRENT:
			mCurrent = data_to_float(&rx_frame->data[0]);
			if(!UsrConfig.can_sync_target_enable){
				sync_callback();
			}
			break;
		
		case CAN_CMD_GET_POSITION:
			float_to_data(Encoder.pos_estimate_, rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
			break;
		
		case CAN_CMD_GET_VELOCITY:
			float_to_data(Encoder.vel_estimate_, rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
			break;
		
		case CAN_CMD_GET_CURRENT:
			float_to_data(Foc.i_q_filt, rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
			break;
		
		case CAN_CMD_GET_VBUS:
			float_to_data(Foc.v_bus, rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
			break;
		
		case CAN_CMD_GET_IBUS:
			float_to_data(Foc.i_bus_filt, rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
			break;
		
		case CAN_CMD_SET_CONFIG:
			config_callback(frameID, rx_frame->data, true);
			break;
		
		case CAN_CMD_GET_CONFIG:
			config_callback(frameID, rx_frame->data, false);
			break;
		
		case CAN_CMD_UPDATE_CONFIGS:
			int_to_data(FSM_input(CMD_UPDATE_CONFIGS), rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
			break;
		
		case CAN_CMD_RESET_ALL_CONFIGS:
			USR_CONFIG_set_default_config();
			int_to_data(0, rx_frame->data);
			can_send(frameID, rx_frame->data, 4);
		
		case CAN_CMD_GET_FW_VERSION:
			int_to_data(FW_VERSION_MAJOR, &rx_frame->data[0]);
			int_to_data(FW_VERSION_MINOR, &rx_frame->data[4]);
			can_send(frameID, rx_frame->data, 8);
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
	
	switch(UsrConfig.control_mode){
		case CONTROL_MODE_CURRENT:
		case CONTROL_MODE_CURRENT_RAMP:
			Controller.input_current = mCurrent;
			break;
		
		case CONTROL_MODE_VELOCITY:
		case CONTROL_MODE_VELOCITY_RAMP:
			Controller.input_velocity = mVelocity;
			break;
		
		case CONTROL_MODE_POSITION:
			Controller.input_position = mPosition;
			break;
		
		case CONTROL_MODE_POSITION_TRAP:
			CONTROLLER_move_to_pos(mPosition);
			break;
		
		default:
			break;
	}
}

static void config_callback(uint32_t frameID, uint8_t* data, bool isSet)
{
	switch(data_to_int(data)){
		case CAN_CONFIG_MOTOR_POLE_PAIRS:
			if(isSet){
				UsrConfig.motor_pole_pairs = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.motor_pole_pairs, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_MOTOR_PHASE_RESISTANCE:
			if(isSet){
				UsrConfig.motor_phase_resistance = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.motor_phase_resistance, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_MOTOR_PHASE_INDUCTANCE:
			if(isSet){
				UsrConfig.motor_phase_inductance = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.motor_phase_inductance, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_INERTIA:
			if(isSet){
				UsrConfig.inertia = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.inertia, &data[4]);
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
			
		case CAN_CONFIG_CONTROL_MODE:
			if(isSet){
				UsrConfig.control_mode = data_to_int(&data[4]);
			}
			int_to_data(UsrConfig.control_mode, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_CURRENT_RAMP_RATE:
			if(isSet){
				UsrConfig.current_ramp_rate = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.current_ramp_rate, &data[4]);
			can_send(frameID, data, 8);
			break;
			
		case CAN_CONFIG_VEL_RAMP_RATE:
			if(isSet){
				UsrConfig.vel_ramp_rate = data_to_float(&data[4]);
			}
			float_to_data(UsrConfig.vel_ramp_rate, &data[4]);
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
				FOC_update_current_gain();
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
