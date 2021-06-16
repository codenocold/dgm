#ifndef __DRV8323_H__
#define __DRV8323_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
	DRV8323_FAULT_STATUS_1 	= 0 << 11,
	DRV8323_FAULT_STATUS_2 	= 1 << 11,
	DRV8323_DRIVER_CONTROL 	= 2 << 11,
	DRV8323_GATE_DRIVE_HS 	= 3 << 11,
	DRV8323_GATE_DRIVE_LS	= 4 << 11,
	DRV8323_OCP_CONTROL		= 5 << 11,
	DRV8323_CSA_CONTROL		= 6 << 11,
} DRV8323_Reg;

/// Drive Control Fields ///
#define DIS_CPUV_EN         0x0     /// Charge pump UVLO fault
#define DIS_CPUV_DIS        0x1
#define DIS_GDF_EN          0x0     /// Gate drive fauilt
#define DIS_GDF_DIS         0x1
#define OTW_REP_EN          0x1     /// Over temp warning reported on nFAULT/FAULT bit
#define OTW_REP_DIS         0x0
#define PWM_MODE_6X         0x0     /// PWM Input Modes
#define PWM_MODE_3X         0x1
#define PWM_MODE_1X         0x2
#define PWM_MODE_IND        0x3
#define PWM_1X_COM_SYNC     0x0     /// 1x PWM Mode synchronou rectification
#define PWM_1X_COM_ASYNC    0x1
#define PWM_1X_DIR_0        0x0     /// In 1x PWM mode this bit is ORed with the INHC (DIR) input
#define PWM_1X_DIR_1        0x1

/// Gate Drive HS Fields ///
#define LOCK_ON             0x6
#define LOCK_OFF            0x3
#define IDRIVEP_HS_10MA     0x0     /// Gate drive high side turn on current
#define IDRIVEP_HS_30MA     0x1
#define IDRIVEP_HS_60MA     0x2
#define IDRIVEP_HS_80MA     0x3
#define IDRIVEP_HS_120MA    0x4
#define IDRIVEP_HS_140MA    0x5
#define IDRIVEP_HS_170MA    0x6
#define IDRIVEP_HS_190MA    0x7
#define IDRIVEP_HS_260MA    0x8
#define IDRIVEP_HS_330MA    0x9
#define IDRIVEP_HS_370MA    0xA
#define IDRIVEP_HS_440MA    0xB
#define IDRIVEP_HS_570MA    0xC
#define IDRIVEP_HS_680MA    0xD
#define IDRIVEP_HS_820MA    0xE
#define IDRIVEP_HS_1000MA   0xF
#define IDRIVEN_HS_20MA     0x0     /// High side turn off current
#define IDRIVEN_HS_60MA     0x1     
#define IDRIVEN_HS_120MA    0x2
#define IDRIVEN_HS_160MA    0x3
#define IDRIVEN_HS_240MA    0x4
#define IDRIVEN_HS_280MA    0x5
#define IDRIVEN_HS_340MA    0x6
#define IDRIVEN_HS_380MA    0x7
#define IDRIVEN_HS_520MA    0x8
#define IDRIVEN_HS_660MA    0x9
#define IDRIVEN_HS_740MA    0xA
#define IDRIVEN_HS_880MA    0xB
#define IDRIVEN_HS_1140MA   0xC
#define IDRIVEN_HS_1360MA   0xD
#define IDRIVEN_HS_1640MA   0xE
#define IDRIVEN_HS_2000MA   0xF

/// Gate Drive LS Fields ///
#define TDRIVE_500NS        0x0     /// Peak gate-current drive time
#define TDRIVE_1000NS       0x1
#define TDRIVE_2000NS       0x2
#define TDRIVE_4000NS       0x3
#define IDRIVEP_LS_10MA     0x0     /// Gate drive high side turn on current
#define IDRIVEP_LS_30MA     0x1
#define IDRIVEP_LS_60MA     0x2
#define IDRIVEP_LS_80MA     0x3
#define IDRIVEP_LS_120MA    0x4
#define IDRIVEP_LS_140MA    0x5
#define IDRIVEP_LS_170MA    0x6
#define IDRIVEP_LS_190MA    0x7
#define IDRIVEP_LS_260MA    0x8
#define IDRIVEP_LS_330MA    0x9
#define IDRIVEP_LS_370MA    0xA
#define IDRIVEP_LS_440MA    0xB
#define IDRIVEP_LS_570MA    0xC
#define IDRIVEP_LS_680MA    0xD
#define IDRIVEP_LS_820MA    0xE
#define IDRIVEP_LS_1000MA   0xF
#define IDRIVEN_LS_20MA     0x0     /// High side turn off current
#define IDRIVEN_LS_60MA     0x1     
#define IDRIVEN_LS_120MA    0x2
#define IDRIVEN_LS_160MA    0x3
#define IDRIVEN_LS_240MA    0x4
#define IDRIVEN_LS_280MA    0x5
#define IDRIVEN_LS_340MA    0x6
#define IDRIVEN_LS_380MA    0x7
#define IDRIVEN_LS_520MA    0x8
#define IDRIVEN_LS_660MA    0x9
#define IDRIVEN_LS_740MA    0xA
#define IDRIVEN_LS_880MA    0xB
#define IDRIVEN_LS_1140MA   0xC
#define IDRIVEN_LS_1360MA   0xD
#define IDRIVEN_LS_1640MA   0xE
#define IDRIVEN_LS_2000MA   0xF

/// OCP Control Fields ///
#define TRETRY_4MS          0x0     /// VDS OCP and SEN OCP retry time
#define TRETRY_50US         0x1
#define DEADTIME_50NS       0x0     /// Deadtime
#define DEADTIME_100NS      0x1
#define DEADTIME_200NS      0x2
#define DEADTIME_400NS      0x3
#define OCP_LATCH           0x0     /// OCP Mode
#define OCP_RETRY           0x1
#define OCP_REPORT          0x2
#define OCP_NONE            0x3
#define OCP_DEG_2US         0x0     /// OCP Deglitch Time
#define OCP_DEG_4US         0x1
#define OCP_DEG_6US         0x2
#define OCP_DEG_8US         0x3
#define VDS_LVL_0_06        0x0
#define VDS_LVL_0_13        0x1
#define VDS_LVL_0_2         0x2
#define VDS_LVL_0_26        0x3
#define VDS_LVL_0_31        0x4
#define VDS_LVL_0_45        0x5
#define VDS_LVL_0_53        0x6
#define VDS_LVL_0_6         0x7
#define VDS_LVL_0_68        0x8
#define VDS_LVL_0_75        0x9
#define VDS_LVL_0_94        0xA
#define VDS_LVL_1_13        0xB
#define VDS_LVL_1_3         0xC
#define VDS_LVL_1_5         0xD
#define VDS_LVL_1_7         0xE
#define VDS_LVL_1_88        0xF

/// CSA Control Fields ///
#define CSA_FET_SP          0x0     /// Current sense amplifier positive input
#define CSA_FET_SH          0x1
#define VREF_DIV_1          0x0     /// Amplifier reference voltage is VREV/1
#define VREF_DIV_2          0x1     /// Amplifier reference voltage is VREV/2
#define CSA_GAIN_5          0x0     /// Current sensor gain
#define CSA_GAIN_10         0x1
#define CSA_GAIN_20         0x2
#define CSA_GAIN_40         0x3
#define DIS_SEN_EN          0x0     /// Overcurrent Fault
#define DIS_SEN_DIS         0x1
#define SEN_LVL_0_25        0x0     /// Sense OCP voltage level
#define SEN_LVL_0_5         0x1
#define SEN_LVL_0_75        0x2
#define SEN_LVL_1_0         0x3

int DRV8323_init(void);
int DRV8323_reset(void);

uint16_t DRV8323_read_reg(const DRV8323_Reg reg);
void DRV8323_write_reg(const DRV8323_Reg reg, uint16_t regVal);
bool DRV8323_isFault(void);
uint32_t DRV8323_getFault(void);
void DRV8323_set_enable(bool enable);
uint16_t DRV8323_read_FSR1(void);
uint16_t DRV8323_read_FSR2(void);
void DRV8323_write_DCR(int DIS_CPUV, int DIS_GDF, int OTW_REP, int PWM_MODE, int PWM_COM, int PWM_DIR, int COAST, int BRAKE, int CLR_FLT);
void DRV8323_write_HSR(int LOCK, int IDRIVEP_HS, int IDRIVEN_HS);
void DRV8323_write_LSR(int CBC, int TDRIVE, int IDRIVEP_LS, int IDRIVEN_LS);
void DRV8323_write_OCPCR(int TRETRY, int DEAD_TIME, int OCP_MODE, int OCP_DEG, int VDS_LVL);
void DRV8323_write_CSACR(int CSA_FET, int VREF_DIV, int LS_REF, int CSA_GAIN, int DIS_SEN, int CSA_CAL_A, int CSA_CAL_B, int CSA_CAL_C, int SEN_LVL);
void DRV8323_enable_gd(void);
void DRV8323_disable_gd(void);
void DRV8323_calibrate(void);

#endif
