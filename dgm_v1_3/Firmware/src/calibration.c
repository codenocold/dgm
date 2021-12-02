#include "calibration.h"
#include "systick.h"
#include "encoder.h"
#include <string.h>
#include "util.h"
#include "usr_config.h"
#include "util.h"
#include "fsm.h"
#include "can.h"
#include "foc.h"
#include "pwm_curr_fdbk.h"

#define CALB_SPEED				M_2PI
#define MOTOR_POLE_PAIRS_MAX	30
#define SAMPLES_PER_PPAIR		OFFSET_LUT_NUM

static tCalibrationStep mCalibStep = CS_NULL;
static tCalibrationError mCalibError = CE_NULL;
static int p_error_arr[SAMPLES_PER_PPAIR * MOTOR_POLE_PAIRS_MAX];

void CALIBRATION_start(void)
{
	UsrConfig.calib_valid = false;
	UsrConfig.encoder_dir_rev = 0;
	
	mCalibStep = CS_MOTOR_R_START;
}

void CALIBRATION_end(void)
{
	mCalibStep = CS_NULL;
}

void CALIBRATION_loop(FOCStruct *foc)
{
	static uint32_t loop_count;
	
	// R
	static const float kI = 2.0f;                       // [(V/s)/A]
    static const int num_R_cycles = (int)(3.0f / DT); 	// Test runs for 3s
	
	// L
	static const int num_L_cycles = 5000;
	static float voltages[2];
    static float Ialphas[2];
	
	static int raw_offset;
	static int sample_count;
	static float next_sample_time;
	static float theta_ref;
	static float start_position;
	static float end_position;

	float time = (float)(loop_count*DT);
	
	switch(mCalibStep){
		case CS_NULL:
			break;
		
		case CS_MOTOR_R_START:
			loop_count = 0;
			voltages[0] = 0.0f;
			FOC_arm();
			mCalibStep = CS_MOTOR_R_LOOP;
			break;
		
		case CS_MOTOR_R_LOOP:
			voltages[0] += (kI * DT) * (UsrConfig.calib_current - foc->i_a);
			if (voltages[0] > UsrConfig.calib_max_voltage || voltages[0] < -UsrConfig.calib_max_voltage){
				FOC_disarm();
				mCalibError = CE_PHASE_RESISTANCE_OUT_OF_RANGE;
				mCalibStep = CS_ERROR;
				break;
			}
			
			// Test voltage along phase A
			apply_voltage_timings(foc->v_bus, voltages[0], 0.0f, 0.0f);
			
			// Test runs for 3s
			if(loop_count >= num_R_cycles){
				FOC_disarm();
				mCalibStep = CS_MOTOR_R_END;
			}
			break;
			
		case CS_MOTOR_R_END:
			UsrConfig.motor_phase_resistance = (voltages[0] / UsrConfig.calib_current) * (2.0f / 3.0f);
			DEBUG("phase_resistance = %f\n\r", UsrConfig.motor_phase_resistance);
		
			{
				uint8_t data[4];
				float_to_data(UsrConfig.motor_phase_resistance, data);
				CAN_report_calibration(1, data);
			}
			
			mCalibStep = CS_MOTOR_L_START;
			break;
		
		case CS_MOTOR_L_START:
			Ialphas[0] = 0.0f;
			Ialphas[1] = 0.0f;
			voltages[0] = -UsrConfig.calib_max_voltage;
			voltages[1] = +UsrConfig.calib_max_voltage;
			loop_count = 0;
			sample_count = 0;
			FOC_arm();
			apply_voltage_timings(foc->v_bus, voltages[0], 0, 0);
			mCalibStep = CS_MOTOR_L_LOOP;
			break;
		
		case CS_MOTOR_L_LOOP:
			if(loop_count >= 2){	// 10KHz
				loop_count = 0;
				
				int i = sample_count & 1;
				Ialphas[i] += foc->i_a;
				
				// Test voltage along phase A
				apply_voltage_timings(foc->v_bus, voltages[i], 0.0f, 0.0f);
				
				if(++sample_count >= (num_L_cycles << 1)){
					FOC_disarm();
					mCalibStep = CS_MOTOR_L_END;
				}
			}
			break;
			
		case CS_MOTOR_L_END:
			{
				float dI_by_dt = (Ialphas[0] - Ialphas[1]) / (float)(DT * num_L_cycles);
				float L = UsrConfig.calib_max_voltage / dI_by_dt;
				UsrConfig.motor_phase_inductance = L * (2.0f / 3.0f);
				DEBUG("phase_inductance = %f\n\r", UsrConfig.motor_phase_inductance);
				
				{
					uint8_t data[4];
					float_to_data(UsrConfig.motor_phase_inductance, data);
					CAN_report_calibration(2, data);
				}
				
				float tc = UsrConfig.current_ctrl_bandwidth;
				float bw = 1.0f / (tc * 1e-6f);
				float kp = UsrConfig.motor_phase_inductance * bw;
				float ki = UsrConfig.motor_phase_resistance * bw;
				UsrConfig.current_ctrl_p_gain = kp;
				UsrConfig.current_ctrl_i_gain = ki;
				
				mCalibStep = CS_ENCODER_DIR_FIND_START;
			}
			break;
		
		case CS_ENCODER_DIR_FIND_START:
			loop_count = 0;
			voltages[0] = UsrConfig.calib_current * UsrConfig.motor_phase_resistance * 3.0f / 2.0f;;
			FOC_arm();
			mCalibStep = CS_ENCODER_DIR_FIND_LOCK;
			break;
			
		case CS_ENCODER_DIR_FIND_LOCK:
			apply_voltage_timings(foc->v_bus, (voltages[0] * time / 2.0f), 0, 0);
			if (time >= 2){
				start_position = Encoder.position;
				loop_count = 0;
				mCalibStep = CS_ENCODER_DIR_FIND_LOOP;
				break;
			}
			break;
		
		case CS_ENCODER_DIR_FIND_LOOP:
			apply_voltage_timings(foc->v_bus, voltages[0], 0, CALB_SPEED*time);
			if (time >= 4.0f*M_PI/CALB_SPEED){
				end_position = Encoder.position;
				FOC_disarm();
				mCalibStep = CS_ENCODER_DIR_FIND_END;
				break;
			}
			break;
			
		case CS_ENCODER_DIR_FIND_END:
			// Check motor pole pairs
			UsrConfig.motor_pole_pairs = round(2.0f/fabsf(end_position-start_position));
			DEBUG("Pole pairs = %d\n\r", UsrConfig.motor_pole_pairs);
			if(UsrConfig.motor_pole_pairs > MOTOR_POLE_PAIRS_MAX){
				mCalibError = CE_MOTOR_POLE_PAIRS_OUT_OF_RANGE;
				mCalibStep = CS_ERROR;
				break;
			}
			
			// Check response and direction
			if(end_position > start_position){
				// motor same dir as encoder
				UsrConfig.encoder_dir_rev = 0;
			}else{
				// motor opposite dir as encoder
				UsrConfig.encoder_dir_rev = 1;
			}
			DEBUG("Encoder dir rev = %d\n\r", UsrConfig.encoder_dir_rev);
			
			{
				uint8_t data[4];
				int_to_data(UsrConfig.motor_pole_pairs, data);
				CAN_report_calibration(3, data);
				
				int_to_data(UsrConfig.encoder_dir_rev, data);
				CAN_report_calibration(4, data);
			}
			
			mCalibStep = CS_ENCODER_OFFSET_START;
			break;
			
		case CS_ENCODER_OFFSET_START:
			sample_count = 0;
			next_sample_time = 0;
			theta_ref = 0;
			loop_count = 0;
			FOC_arm();
			mCalibStep = CS_ENCODER_OFFSET_LOCK;
			break;
			
		case CS_ENCODER_OFFSET_LOCK:
			apply_voltage_timings(foc->v_bus, (voltages[0] * time / 2.0f), 0, theta_ref);
			if (time >= 2){
				loop_count = 0;
				raw_offset = Encoder.raw;
				mCalibStep = CS_ENCODER_OFFSET_CW_LOOP;
				break;
			}
			break;
		
		case CS_ENCODER_OFFSET_CW_LOOP:
			// rotate voltage vector through one mechanical rotation in the positive direction
			theta_ref += CALB_SPEED*DT;
			apply_voltage_timings(foc->v_bus, voltages[0], 0, theta_ref);
		
			// sample SAMPLES_PER_PPAIR times per pole-pair
			if(time > next_sample_time){
				int count_ref = theta_ref * (float)ENCODER_CPR/(M_2PI*UsrConfig.motor_pole_pairs);
				int error = Encoder.raw - count_ref;
				p_error_arr[sample_count] = error + ENCODER_CPR * (error<0);
				next_sample_time += M_2PI/(CALB_SPEED*SAMPLES_PER_PPAIR);
				if(sample_count < UsrConfig.motor_pole_pairs*SAMPLES_PER_PPAIR){
					sample_count++;
				}
			}

			if (time >= M_2PI*UsrConfig.motor_pole_pairs/CALB_SPEED){
				next_sample_time = 0;
				loop_count = 0;
				mCalibStep = CS_ENCODER_OFFSET_CCW_LOOP;
				break;
			}
			break;
			
		case CS_ENCODER_OFFSET_CCW_LOOP:
			// rotate voltage vector through one mechanical rotation in the negative direction
			theta_ref -= CALB_SPEED*DT;
			apply_voltage_timings(foc->v_bus, voltages[0], 0, theta_ref);
		
			// sample SAMPLES_PER_PPAIR times per pole-pair
			if((time > next_sample_time) && (sample_count > 0)){
				int count_ref = theta_ref * (float)ENCODER_CPR/(M_2PI*UsrConfig.motor_pole_pairs);
				int error = Encoder.raw - count_ref;
				error = error + ENCODER_CPR * (error<0);
				p_error_arr[sample_count] = (p_error_arr[sample_count] + error)/2;
				if(sample_count > 0){
					sample_count--;
				}
				next_sample_time += M_2PI/(CALB_SPEED*SAMPLES_PER_PPAIR);
			}

			if (time >= M_2PI*UsrConfig.motor_pole_pairs/CALB_SPEED){
				raw_offset += Encoder.raw;
				raw_offset /= 2;
				FOC_disarm();
				mCalibStep = CS_ENCODER_OFFSET_END;
				break;
			}
			break;
			
		case CS_ENCODER_OFFSET_END:
			{
				// Calculate average offset
				int ezero_mean = 0;
				for(int i = 0; i<((int)UsrConfig.motor_pole_pairs*SAMPLES_PER_PPAIR); i++){
					ezero_mean += p_error_arr[i];
				}
				UsrConfig.encoder_offset = ezero_mean/(UsrConfig.motor_pole_pairs*SAMPLES_PER_PPAIR);
				DEBUG("Encoder Offset = %d\n\r",  UsrConfig.encoder_offset);
				
				uint8_t data[4];
				int_to_data(UsrConfig.encoder_offset, data);
				CAN_report_calibration(5, data);
		
				// Moving average to filter out cogging ripple
				int window = SAMPLES_PER_PPAIR;
				raw_offset = OFFSET_LUT_NUM * raw_offset / ENCODER_CPR;
				for(int i = 0; i<OFFSET_LUT_NUM; i++){
					int moving_avg = 0;
					for(int j = (-window)/2; j<(window)/2; j++){
						int index = i*UsrConfig.motor_pole_pairs*SAMPLES_PER_PPAIR/OFFSET_LUT_NUM + j;
						if(index < 0){
							index += (SAMPLES_PER_PPAIR*UsrConfig.motor_pole_pairs);
						}else if(index > (SAMPLES_PER_PPAIR*UsrConfig.motor_pole_pairs-1)){
							index -= (SAMPLES_PER_PPAIR*UsrConfig.motor_pole_pairs);
						}
						moving_avg += p_error_arr[index];
					}
					moving_avg = moving_avg/window;
					int lut_index = raw_offset + i;
					if(lut_index > (OFFSET_LUT_NUM-1)){
						lut_index -= OFFSET_LUT_NUM;
					}
					UsrConfig.offset_lut[lut_index] = moving_avg - UsrConfig.encoder_offset;
				}
				
				// CAN report
				for(int i=0; i<OFFSET_LUT_NUM; i++){
					int_to_data(UsrConfig.offset_lut[i], data);
					CAN_report_calibration(i+10, data);
					SYSTICK_delay_ms(10);
				}
				
				UsrConfig.calib_valid = true;
				FSM_input(CMD_MENU);
			}
			break;
		
		case CS_ERROR:
			DEBUG("Calib Error %d\n\r", mCalibError);
			{
				uint8_t data[4];
				int_to_data(mCalibError, data);
				CAN_report_calibration(-1, data);
			}
			CALIBRATION_end();
			mCalibStep = CS_NULL;
			break;
		
		default:
			break;
	}
	
	loop_count ++;
}
