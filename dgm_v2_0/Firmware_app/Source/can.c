/*
	Copyright 2021 codenocold codenocold@qq.com
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
#include "foc.h"
#include "encoder.h"
#include "controller.h"
#include "usr_config.h"
#include "util.h"
#include "dfu.h"
#include <string.h>

static uint8_t mNodeID;
static uint32_t mRxTick = 0;
static uint32_t mTxTick = 0;
static uint32_t mCanBusErrNew = 0;
static uint32_t mCanBusErrOld = 0;

static void can_error_check(void);
static bool can_tx(CanFrame *tx_frame);
static bool can_rx(CanFrame *rx_frame);

static void parse_frame(CanFrame *frame);
static void config_callback(uint8_t *data, bool isSet);

void CAN_set_node_id(uint8_t nodeID)
{
    mNodeID = nodeID;
}

void CAN_comm_loop(void)
{
	// rx heartbeat timeout check
	if(UsrConfig.heartbeat_consumer_ms){
		if(get_ms_since(mRxTick) > UsrConfig.heartbeat_consumer_ms){
			MCT_set_state(IDLE);
		}
	}
    
    // tx heartbeat timeout check
    if(UsrConfig.heartbeat_producer_ms){
        if(get_ms_since(mTxTick) > UsrConfig.heartbeat_producer_ms){
            // Send heartbeat
            CanFrame tx_frame;
            tx_frame.id = (mNodeID << 6) | CAN_CMD_HEARTBEAT;
            tx_frame.dlc = 0;
            can_tx(&tx_frame);
        }
    }
    
    // CAN bus error check
    can_error_check();
}

void CAN_receive_callback(void)
{
	CanFrame rxframe;
	while(can_rx(&rxframe)){
        parse_frame(&rxframe);
	}
}

void CAN_reset_rx_timeout(void)
{
	mRxTick = SystickCount;
}

void CAN_reset_tx_timeout(void)
{
    mTxTick = SystickCount;
}

void CAN_tx_statusword(tMCStatusword statusword)
{
	CanFrame tx_frame;
    tx_frame.id = (mNodeID << 6) | CAN_CMD_STATUSWORD_REPORT;
	tx_frame.dlc = 0;
	tx_frame.dlc += uint32_to_data(statusword.status.status_code, &tx_frame.data[tx_frame.dlc]);
	tx_frame.dlc += uint32_to_data(statusword.errors.errors_code, &tx_frame.data[tx_frame.dlc]);
	can_tx(&tx_frame);
}

void CAN_calib_report(int32_t step, uint8_t *data)
{
	CanFrame tx_frame;
    tx_frame.id = (mNodeID << 6) | CAN_CMD_CALIB_REPORT;
	tx_frame.dlc = 0;
	tx_frame.dlc += int32_to_data(step, &tx_frame.data[tx_frame.dlc]);
	memcpy(&tx_frame.data[tx_frame.dlc], data, 4);
	tx_frame.dlc += 4;
    can_tx(&tx_frame);
}

void CAN_anticogging_report(int32_t step, int32_t value)
{
    CanFrame tx_frame;
    tx_frame.id = (mNodeID << 6) | CAN_CMD_ANTICOGGING_REPORT;
	tx_frame.dlc = 0;
	tx_frame.dlc += int32_to_data(step, &tx_frame.data[tx_frame.dlc]);
	tx_frame.dlc += int32_to_data(value, &tx_frame.data[tx_frame.dlc]);
    can_tx(&tx_frame);
}

static void can_error_check(void)
{
    mCanBusErrNew = CAN_ERR(CAN0);
    if(mCanBusErrOld != mCanBusErrNew){
        mCanBusErrOld = mCanBusErrNew;
        
        // warning error
        if(mCanBusErrNew & CAN_ERR_WERR){
            
        }

        // passive error
        if(mCanBusErrNew & CAN_ERR_PERR){
            // Reset Tx fifo
            can_transmission_stop(CAN0, CAN_MAILBOX0);
            can_transmission_stop(CAN0, CAN_MAILBOX1);
            can_transmission_stop(CAN0, CAN_MAILBOX2);
            
            // Reset Rx fifo
            CAN_RFIFO0(CAN0) |= CAN_RFIFO0_RFD0;
        }
        
        // bus-off error
        if(mCanBusErrNew & CAN_ERR_BOERR){
            // Reset Tx fifo
            can_transmission_stop(CAN0, CAN_MAILBOX0);
            can_transmission_stop(CAN0, CAN_MAILBOX1);
            can_transmission_stop(CAN0, CAN_MAILBOX2);
            
            // Reset Rx fifo
            CAN_RFIFO0(CAN0) |= CAN_RFIFO0_RFD0;
        }
        
        // CAN error type
        switch (can_error_get(CAN0)) {
            case CAN_ERROR_FILL:
                // fill error
                
                break;
            
            case CAN_ERROR_FORMATE:
                // format error
                
                break;
            
            case CAN_ERROR_ACK:
                // ack error
                
                break;
            
            case CAN_ERROR_BITRECESSIVE:
                // bit recessive error
                
                break;
            
            case CAN_ERROR_BITDOMINANTER:
                // bit dominant error
                
                break;
            
            case CAN_ERROR_CRC:
                // crc error
                
                break;
            
            default:
                break;
        }
        CAN_ERR(CAN0) = CAN_ERR_ERRN;
    }
}

static bool can_tx(CanFrame *tx_frame)
{
	uint8_t mailbox_number;
	
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
	CAN_TMI(CAN0, mailbox_number) = (uint32_t)(TMI_SFID(tx_frame->id & 0x7FF));
	
	/* set the data length */
	CAN_TMP(CAN0, mailbox_number) &= ~(CAN_TMP_DLENC|CAN_TMP_ESI|CAN_TMP_BRS|CAN_TMP_FDF);
	CAN_TMP(CAN0, mailbox_number) |= tx_frame->dlc & 0x0F;
	
	/* set the data */
	CAN_TMDATA0(CAN0, mailbox_number) = TMDATA0_DB3(tx_frame->data[3]) | \
										TMDATA0_DB2(tx_frame->data[2]) | \
										TMDATA0_DB1(tx_frame->data[1]) | \
										TMDATA0_DB0(tx_frame->data[0]);
	CAN_TMDATA1(CAN0, mailbox_number) = TMDATA1_DB7(tx_frame->data[7]) | \
									    TMDATA1_DB6(tx_frame->data[6]) | \
										TMDATA1_DB5(tx_frame->data[5]) | \
										TMDATA1_DB4(tx_frame->data[4]);
	
	/* enable transmission */
    CAN_TMI(CAN0, mailbox_number) |= CAN_TMI_TEN;
    
    CAN_reset_tx_timeout();
	
	return true;
}

static bool can_rx(CanFrame *rx_frame)
{
	if ((CAN_RFIFO0(CAN0) & CAN_RFIF_RFL_MASK) != 0) {
		rx_frame->id = GET_RFIFOMI_SFID(CAN_RFIFOMI(CAN0, 0));

		rx_frame->dlc = (uint8_t)(GET_RFIFOMP_DLENC(CAN_RFIFOMP(CAN0, 0)));
		
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

static void parse_frame(CanFrame *frame)
{
	int ret;
	bool echo = false;
    
    // nodeID | CMD
	// 5 bit  | 6 bit
    
    // Node id check
    uint8_t nodeID = (frame->id & NODE_ID_BIT) >> 6;
    if(nodeID != mNodeID && nodeID != 0){
		return;
	}

    CAN_reset_rx_timeout();
	
	switch(frame->id & CMD_BIT){
		case CAN_CMD_MOTOR_DISABLE:
			if(frame->dlc == 0){
				ret = MCT_set_state(IDLE);
				int32_to_data(ret, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;
		
		case CAN_CMD_MOTOR_ENABLE:
			if(frame->dlc == 0){
				ret = MCT_set_state(RUN);
				int32_to_data(ret, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;

        case CAN_CMD_SET_TORQUE:
			if(frame->dlc == 4){
				if(UsrConfig.invert_motor_dir){
					Controller.input_torque_buffer = -data_to_float(&frame->data[0]);
				}else{
					Controller.input_torque_buffer = +data_to_float(&frame->data[0]);
				}
				if(!UsrConfig.sync_target_enable){
					CONTROLLER_sync_callback();
				}
			}
			break;
            
        case CAN_CMD_SET_VELOCITY:
			if(frame->dlc == 4){
				if(UsrConfig.invert_motor_dir){
					Controller.input_velocity_buffer = -data_to_float(&frame->data[0]);
				}else{
					Controller.input_velocity_buffer = +data_to_float(&frame->data[0]);
				}
				if(!UsrConfig.sync_target_enable){
					CONTROLLER_sync_callback();
				}
			}
			break;
		
       case CAN_CMD_SET_POSITION:
			if(frame->dlc == 4){
				if(UsrConfig.invert_motor_dir){
					Controller.input_position_buffer = -data_to_float(&frame->data[0]);
				}else{
					Controller.input_position_buffer = +data_to_float(&frame->data[0]);
				}
				if(!UsrConfig.sync_target_enable){
					CONTROLLER_sync_callback();
				}
			}
			break;
        
        case CAN_CMD_SYNC:
			if(frame->dlc == 0){
				if(UsrConfig.sync_target_enable){
					CONTROLLER_sync_callback();
				}
			}
			break;
        
        case CAN_CMD_CALIB_START:
			if(frame->dlc == 0){
				ret = MCT_set_state(CALIBRATION);
				int32_to_data(ret, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;
		
		case CAN_CMD_CALIB_ABORT:
			if(frame->dlc == 0){
				ret = MCT_set_state(IDLE);
				int32_to_data(ret, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;
        
        case CAN_CMD_ANTICOGGING_START:
			if(frame->dlc == 0){
				ret = MCT_set_state(ANTICOGGING);
				int32_to_data(ret, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;
        
        case CAN_CMD_ANTICOGGING_ABORT:
			if(frame->dlc == 0){
				ret = MCT_set_state(IDLE);
				int32_to_data(ret, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;
            
        case CAN_CMD_SET_HOME:
			if(frame->dlc == 0){
				ret = CONTROLLER_set_home();
				int32_to_data(ret, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;
 
		case CAN_CMD_ERROR_RESET:
			if(frame->dlc == 0){
				MCT_reset_error();
				int32_to_data(0, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;

        case CAN_CMD_GET_STATUSWORD:
			if(frame->dlc == 0){
				uint32_to_data(StatuswordNew.status.status_code, &frame->data[0]);
				uint32_to_data(StatuswordNew.errors.errors_code, &frame->data[4]);
				frame->dlc = 8;
				echo = true;
			}
			break;
        
        case CAN_CMD_GET_TORQUE:
			if(frame->dlc == 0){
				if(UsrConfig.invert_motor_dir){
					float_to_data(-Foc.i_q_filt * UsrConfig.torque_constant, &frame->data[0]);
				}else{
					float_to_data(+Foc.i_q_filt * UsrConfig.torque_constant, &frame->data[0]);
				}
				frame->dlc = 4;
				echo = true;
			}
            break;
            
        case CAN_CMD_GET_VELOCITY:
			if(frame->dlc == 0){
				if(UsrConfig.invert_motor_dir){
					float_to_data(-Encoder.vel, &frame->data[0]);
				}else{
					float_to_data(+Encoder.vel, &frame->data[0]);
				}
				frame->dlc = 4;
				echo = true;
			}
			break;
            
        case CAN_CMD_GET_POSITION:
			if(frame->dlc == 0){
				if(UsrConfig.invert_motor_dir){
					float_to_data(-Encoder.pos, &frame->data[0]);
				}else{
					float_to_data(+Encoder.pos, &frame->data[0]);
				}
				frame->dlc = 4;
				echo = true;
			}
			break;
            
        case CAN_CMD_GET_I_Q:
			if(frame->dlc == 0){
				if(UsrConfig.invert_motor_dir){
					float_to_data(-Foc.i_q_filt, &frame->data[0]);
				}else{
					float_to_data(+Foc.i_q_filt, &frame->data[0]);
				}
				frame->dlc = 4;
				echo = true;
			}
			break;
		
		case CAN_CMD_GET_VBUS:
			if(frame->dlc == 0){
				float_to_data(Foc.v_bus_filt, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;
		
		case CAN_CMD_GET_IBUS:
			if(frame->dlc == 0){
				float_to_data(Foc.i_bus_filt, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;
		
		case CAN_CMD_GET_POWER:
			if(frame->dlc == 0){
				float_to_data(Foc.power_filt, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;

		case CAN_CMD_SET_CONFIG:
			if(frame->dlc == 8){
				config_callback(frame->data, true);
				frame->dlc = 8;
				echo = true;
			}
			break;
		
		case CAN_CMD_GET_CONFIG:
            if(frame->dlc == 4){
				config_callback(frame->data, false);
				frame->dlc = 8;
				echo = true;
			}
			break;
		
		case CAN_CMD_SAVE_ALL_CONFIG:
			if(frame->dlc == 0){
				ret = 0;
				ret += USR_CONFIG_save_config();
				ret += USR_CONFIG_save_cogging_map();
				int32_to_data(ret, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;
		
		case CAN_CMD_RESET_ALL_CONFIG:
			if(frame->dlc == 0){
				if(MCT_get_state() == IDLE){
					USR_CONFIG_set_default_config();
					USR_CONFIG_set_default_cogging_map();
					ret = 0;
				}else{
					ret = -1;
				}
				int32_to_data(ret, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;

		case CAN_CMD_GET_FW_VERSION:
			if(frame->dlc == 0){
				int32_to_data(FW_VERSION_MAJOR, &frame->data[0]);
				int32_to_data(FW_VERSION_MINOR, &frame->data[4]);
				frame->dlc = 8;
				echo = true;
			}
			break;
		
		case CAN_CMD_WRITE_APP_BACK_START:
			if(frame->dlc == 0){
				ret = DFU_write_app_back_start();
				int32_to_data(ret, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;
		
		case CAN_CMD_WRITE_APP_BACK:
			DFU_write_app_back(&frame->data[0], frame->dlc);
			break;
		
		case CAN_CMD_CHECK_APP_BACK:
			if(frame->dlc == 8){
				ret = DFU_check_app_back(data_to_uint32(&frame->data[0]), data_to_uint32(&frame->data[4]));
				int32_to_data(ret, &frame->data[0]);
				frame->dlc = 4;
				echo = true;
			}
			break;
		
		case CAN_CMD_DFU_START:
			if(frame->dlc == 0){
				watch_dog_feed();
				DFU_jump_bootloader();
			}
			break;
        
		default:
			break;
	}
	
	if(echo){
        can_tx(frame);
	}
}

static void config_callback(uint8_t *data, bool isSet)
{
	int32_t idx = data_to_int32(data) - 1;
    int32_t config_number = sizeof(tUsrConfig)/4 - 132;
    
	if(idx >= config_number){
		int32_to_data(-1, &data[0]);
        int32_to_data( 0, &data[4]);
        return;
	}
    
	uint32_t *pConfig = &(((uint32_t*)(&UsrConfig))[idx]);

	if(isSet){
        if(MCT_get_state() != IDLE){
            int32_to_data(-1, &data[0]);
            int32_to_data( 0, &data[4]);
            return;
        }
		memcpy(pConfig, &data[4], 4);
        
        FOC_update_current_ctrl_gain(UsrConfig.current_ctrl_bw);
        CONTROLLER_update_input_pos_filter_gain(UsrConfig.position_filter_bw);
	}else{
		memcpy(&data[4], pConfig, 4);
	}
}
