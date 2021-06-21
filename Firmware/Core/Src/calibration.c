#include "calibration.h"
#include "systick.h"
#include "encoder.h"
#include "hw_config.h"
#include "arm_math.h"
#include <string.h>
#include "util.h"
#include "usr_config.h"
#include "util.h"
#include "anticogging.h"
#include "heap.h"
#include "fsm.h"
#include "fdcan.h"

#define CALB_SPEED	10.0f					// Calibration speed in rad/s
#define MOTOR_POLE_PAIRS_MAX	20
#define SAMPLES_PER_PPAIR OFFSET_LUT_NUM

static tCalibrationStep mCalibStep = CS_NULL;
static tCalibrationError mCalibError = CE_NULL;
static int *p_error_arr = NULL;

static void apply_voltage_timings(float vbus, float v_d, float v_q, float pwm_phase);

void CALIBRATION_start(void)
{
	if(pCoggingMap != NULL){
		AnticoggingValid = false;
		HEAP_free(pCoggingMap);
		pCoggingMap = NULL;
	}
	
	if(p_error_arr == NULL){
		p_error_arr = HEAP_malloc(SAMPLES_PER_PPAIR*MOTOR_POLE_PAIRS_MAX*4);
	}
	
	UsrConfig.encoder_dir_rev = 0;
	UsrConfig.calib_valid = false;
	
	mCalibStep = CS_MOTOR_R_START;
}

void CALIBRATION_end(void)
{
	mCalibStep = CS_NULL;
	
	HEAP_free(p_error_arr);
	p_error_arr = NULL;
	
	if(0 == USR_CONFIG_read_cogging_map()){
		AnticoggingValid = true;
	}else{
		USR_CONFIG_set_default_cogging_map();
		AnticoggingValid = false;
	}
}

void CALIBRATION_loop(FOCStruct *foc)
{
	static uint32_t loop_count;
	
	// R
	static const float kI = 5.0f;                       // [(V/s)/A]
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
			voltages[0] += (kI * DT) * (UsrConfig.calib_current - foc->i_c);
			if (voltages[0] > UsrConfig.calib_max_voltage || voltages[0] < -UsrConfig.calib_max_voltage){
				FOC_disarm();
				mCalibError = CE_PHASE_RESISTANCE_OUT_OF_RANGE;
				mCalibStep = CS_ERROR;
				break;
			}
			
			// Test voltage along phase C
			apply_voltage_timings(foc->v_bus, voltages[0], 0.0f, PI*4.0f/3.0f);
			
			// Test runs for 3s
			if(loop_count >= num_R_cycles){
				FOC_disarm();
				mCalibStep = CS_MOTOR_R_END;
			}
			break;
			
		case CS_MOTOR_R_END:
			UsrConfig.phase_resistance = (voltages[0] / UsrConfig.calib_current) * (2.0f / 3.0f);
			DEBUG("phase_resistance = %f\n\r", UsrConfig.phase_resistance);
		
			{
				uint8_t data[4];
				float_to_data(UsrConfig.phase_resistance, data);
				FDCAN_report_calibration(1, data);
			}
			
			mCalibStep = CS_MOTOR_L_START;
			break;
		
		case CS_MOTOR_L_START:
			loop_count = 0;
			sample_count = 0;
			voltages[0] = -UsrConfig.calib_max_voltage;
			voltages[1] = +UsrConfig.calib_max_voltage;
			Ialphas[0] = 0.0f;
			Ialphas[1] = 0.0f;
			FOC_arm();
			mCalibStep = CS_MOTOR_L_LOOP;
			break;
		
		case CS_MOTOR_L_LOOP:
			if(loop_count >= 3){	// 25/3 = 8KHz
				loop_count = 0;
				
				int i = sample_count & 1;
				Ialphas[i] += foc->i_c;
				
				// Test voltage along phase C
				apply_voltage_timings(foc->v_bus, voltages[i], 0.0f, PI*4.0f/3.0f);
				
				if(++sample_count >= (num_L_cycles << 1)){
					FOC_disarm();
					mCalibStep = CS_MOTOR_L_END;
				}
			}
			break;
			
		case CS_MOTOR_L_END:
			{
				float dI_by_dt = (Ialphas[0] - Ialphas[1]) / (DT * num_L_cycles);
				float L = UsrConfig.calib_max_voltage / dI_by_dt;
				UsrConfig.phase_inductance = L * (2.0f / 3.0f);
				DEBUG("phase_inductance = %f\n\r", UsrConfig.phase_inductance);
				
				{
					uint8_t data[4];
					float_to_data(UsrConfig.phase_inductance, data);
					FDCAN_report_calibration(2, data);
				}
				
				// Calculate current control gains
				float p_gain = UsrConfig.current_ctrl_bandwidth * UsrConfig.phase_inductance;
				float plant_pole = UsrConfig.phase_resistance / UsrConfig.phase_inductance;
				float i_gain = plant_pole * p_gain;
				UsrConfig.current_ctrl_p_gain = p_gain;
				UsrConfig.current_ctrl_i_gain = i_gain;
				
				mCalibStep = CS_ENCODER_DIR_FIND_START;
			}
			break;
		
		case CS_ENCODER_DIR_FIND_START:
			loop_count = 0;
			voltages[0] = UsrConfig.calib_current * UsrConfig.phase_resistance;
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
			if (time >= 4.0f*PI/CALB_SPEED){
				end_position = Encoder.position;
				FOC_disarm();
				mCalibStep = CS_ENCODER_DIR_FIND_END;
				break;
			}
			break;
			
		case CS_ENCODER_DIR_FIND_END:
			// Check motor pole pairs
			UsrConfig.pole_pairs = round(2.0f/fabsf(end_position-start_position));
			DEBUG("Pole pairs = %d\n\r", UsrConfig.pole_pairs);
			if(UsrConfig.pole_pairs > MOTOR_POLE_PAIRS_MAX){
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
				int_to_data(UsrConfig.pole_pairs, data);
				FDCAN_report_calibration(3, data);
				
				int_to_data(UsrConfig.encoder_dir_rev, data);
				FDCAN_report_calibration(4, data);
			}
			
			mCalibStep = CS_ENCODER_OFFSET_START;
			break;
			
		case CS_ENCODER_OFFSET_START:
			sample_count = 0;
			next_sample_time = 0;
			theta_ref = 0;
			loop_count = 0;
			voltages[0] = UsrConfig.calib_current * UsrConfig.phase_resistance;
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
				int count_ref = theta_ref * (float)ENCODER_CPR/(2.0f*PI*UsrConfig.pole_pairs);
				int error = Encoder.raw - count_ref;
				p_error_arr[sample_count] = error + ENCODER_CPR * (error<0);
				next_sample_time += 2.0f*PI/(CALB_SPEED*SAMPLES_PER_PPAIR);
				if(sample_count <= UsrConfig.pole_pairs*SAMPLES_PER_PPAIR-1){
					sample_count++;
				}
			}

			if (time >= 2.0f*PI*UsrConfig.pole_pairs/CALB_SPEED){
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
				int count_ref = theta_ref * (float)ENCODER_CPR/(2.0f*PI*UsrConfig.pole_pairs);
				int error = Encoder.raw - count_ref;
				error = error + ENCODER_CPR * (error<0);
				p_error_arr[sample_count] = (p_error_arr[sample_count] + error)/2;
				sample_count--;
				next_sample_time += 2.0f*PI/(CALB_SPEED*SAMPLES_PER_PPAIR);
			}

			if (time >= 2.0f*PI*UsrConfig.pole_pairs/CALB_SPEED){
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
				for(int i = 0; i<((int)UsrConfig.pole_pairs*SAMPLES_PER_PPAIR); i++){
					ezero_mean += p_error_arr[i];
				}
				UsrConfig.encoder_offset = ezero_mean/(UsrConfig.pole_pairs*SAMPLES_PER_PPAIR);
				DEBUG("Encoder Offset = %d\n\r",  UsrConfig.encoder_offset);
				
				uint8_t data[4];
				int_to_data(UsrConfig.encoder_offset, data);
				FDCAN_report_calibration(5, data);
		
				// Moving average to filter out cogging ripple
				int window = SAMPLES_PER_PPAIR;
				raw_offset = OFFSET_LUT_NUM * raw_offset / ENCODER_CPR;
				for(int i = 0; i<OFFSET_LUT_NUM; i++){
					int moving_avg = 0;
					for(int j = (-window)/2; j<(window)/2; j++){
						int index = i*UsrConfig.pole_pairs*SAMPLES_PER_PPAIR/OFFSET_LUT_NUM + j;
						if(index < 0){
							index += (SAMPLES_PER_PPAIR*UsrConfig.pole_pairs);
						}else if(index > (SAMPLES_PER_PPAIR*UsrConfig.pole_pairs-1)){
							index -= (SAMPLES_PER_PPAIR*UsrConfig.pole_pairs);
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
					FDCAN_report_calibration(i+10, data);
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
				FDCAN_report_calibration(-1, data);
			}
			CALIBRATION_end();
			mCalibStep = CS_NULL;
			break;
		
		default:
			break;
	}
	
	loop_count ++;
}

static void apply_voltage_timings(float vbus, float v_d, float v_q, float pwm_phase)
{
	// Modulation
    float V_to_mod = 1.0f / ((2.0f / 3.0f) * vbus);
    float mod_d = V_to_mod * v_d;
    float mod_q = V_to_mod * v_q;
	
	// Vector modulation saturation, lock integrator if saturated
    float mod_scalefactor = 0.80f * SQRT3_BY_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
    }
	
	// Inverse park transform
	float mod_alpha;
    float mod_beta;
	inverse_park(mod_d, mod_q, pwm_phase, &mod_alpha, &mod_beta);

	// SVM
	float dtc_a, dtc_b, dtc_c;
	svm(mod_alpha, mod_beta, &dtc_a, &dtc_b, &dtc_c);

	// Apply duty
	TIM1->CCR3 = (uint16_t)(dtc_a * (float)PWM_ARR);
	TIM1->CCR2 = (uint16_t)(dtc_b * (float)PWM_ARR);
	TIM1->CCR1 = (uint16_t)(dtc_c * (float)PWM_ARR);
}
