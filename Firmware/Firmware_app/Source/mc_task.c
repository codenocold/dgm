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

#include "mc_task.h"
#include "anticogging.h"
#include "calibration.h"
#include "can.h"
#include "controller.h"
#include "encoder.h"
#include "foc.h"
#include "pwm_curr.h"
#include "usr_config.h"
#include "util.h"
#include <math.h>
#include <string.h>

typedef struct sFSM
{
    tFSMState state;
    tFSMState state_next;
    uint8_t   state_next_ready;
} tFSM;

static volatile tFSM mFSM;

volatile tMCStatusword StatuswordNew;
volatile tMCStatusword StatuswordOld;
;

#define CHARGE_BOOT_CAP_MS    10
#define CHARGE_BOOT_CAP_TICKS (uint16_t) ((PWM_FREQUENCY * CHARGE_BOOT_CAP_MS) / 1000)
static uint16_t mChargeBootCapDelay = 0;

static void enter_state(void);
static void exit_state(void);
static void led_act_loop(void);

void MCT_init(void)
{
    mFSM.state            = BOOT_UP;
    mFSM.state_next       = BOOT_UP;
    mFSM.state_next_ready = 0;

    StatuswordNew.status.status_code = 0;
    StatuswordNew.errors.errors_code = 0;
    StatuswordOld                    = StatuswordNew;
}

int MCT_reset_error(void)
{
    StatuswordNew.errors.errors_code &= 0x80;
    StatuswordOld.errors.errors_code &= 0x80;
    return 0;
}

tFSMState MCT_get_state(void)
{
    return mFSM.state;
}

// return
//    0 Success
//   -1 Invalid
//   -2 Error code
//   -3 Calib invalid
int MCT_set_state(tFSMState state)
{
    int ret = 0;

    switch (mFSM.state) {
    case BOOT_UP:
        if (state == IDLE) {
            mFSM.state_next = IDLE;
        } else {
            ret = -1;
        }
        break;

    case IDLE:
        switch (state) {
        case IDLE:
            FOC_disarm();
            mChargeBootCapDelay = 0;
            mFSM.state_next     = IDLE;
            break;

        case RUN:
            if (StatuswordNew.errors.errors_code) {
                ret = -2;
            } else if (!UsrConfig.calib_valid) {
                ret = -3;
            } else {
                FOC_arm();
                mChargeBootCapDelay = CHARGE_BOOT_CAP_TICKS;
                mFSM.state_next     = RUN;
            }
            break;

        case CALIBRATION:
            if (StatuswordNew.errors.errors_code) {
                ret = -2;
            } else {
                FOC_arm();
                mChargeBootCapDelay = CHARGE_BOOT_CAP_TICKS;
                mFSM.state_next     = CALIBRATION;
            }
            break;

        case ANTICOGGING:
            if (StatuswordNew.errors.errors_code) {
                ret = -2;
            } else if (!UsrConfig.calib_valid) {
                ret = -3;
            } else {
                FOC_arm();
                mChargeBootCapDelay = CHARGE_BOOT_CAP_TICKS;
                mFSM.state_next     = ANTICOGGING;
            }
            break;

        default:
            ret = -1;
            break;
        }
        break;

    default:
        if (state == IDLE) {
            mFSM.state_next = IDLE;
        } else {
            ret = -1;
        }
        break;
    }

    mFSM.state_next_ready = 0;

    return ret;
}

static void enter_state(void)
{
    switch (mFSM.state) {
    case BOOT_UP:
        break;

    case IDLE:
        break;

    case RUN:
        CONTROLLER_reset();
        StatuswordNew.status.switched_on    = 1;
        StatuswordNew.status.target_reached = 1;
        StatuswordOld.status                = StatuswordNew.status;
        break;

    case CALIBRATION:
        CALIBRATION_start();
        break;

    case ANTICOGGING:
        CONTROLLER_reset();
        ANTICOGGING_start();
        break;

    default:
        break;
    }
}

static void exit_state(void)
{
    switch (mFSM.state) {
    case BOOT_UP:
        CAN_reset_rx_timeout();
        CAN_reset_tx_timeout();
        mFSM.state_next_ready = 1;
        break;

    case IDLE:
        if (mChargeBootCapDelay) {
            mChargeBootCapDelay--;
        } else {
            mFSM.state_next_ready = 1;
        }
        break;

    case RUN:
        FOC_disarm();
        StatuswordNew.status.switched_on    = 0;
        StatuswordNew.status.target_reached = 0;
        StatuswordOld.status                = StatuswordNew.status;
        mFSM.state_next_ready               = 1;
        break;

    case CALIBRATION:
        CALIBRATION_end();
        mFSM.state_next_ready = 1;
        break;

    case ANTICOGGING:
        ANTICOGGING_end();
        mFSM.state_next_ready = 1;
        break;

    default:
        break;
    }
}

void MCT_high_frequency_task(void)
{
    /* state transition management */
    if (mFSM.state_next != mFSM.state) {
        exit_state();
        if (mFSM.state_next_ready) {
            mFSM.state = mFSM.state_next;
            enter_state();
        }
    }

    ENCODER_loop();

    Foc.v_bus = read_vbus();
    UTILS_LP_FAST(Foc.v_bus_filt, Foc.v_bus, 0.05f);
    Foc.i_a = read_iphase_a();
    Foc.i_b = read_iphase_b();
    Foc.i_c = -(Foc.i_a + Foc.i_b);

    switch (mFSM.state) {
    case BOOT_UP:
        break;

    case CALIBRATION:
        CALIBRATION_loop();

        // check over current
        if (ABS(Foc.i_a) > UsrConfig.protect_over_current || ABS(Foc.i_b) > UsrConfig.protect_over_current
            || ABS(Foc.i_c) > UsrConfig.protect_over_current) {
            FOC_disarm();
            MCT_set_state(IDLE);
            StatuswordNew.errors.over_current = 1;
        }
        break;

    case ANTICOGGING:
        ANTICOGGING_loop();

    case RUN:
        CONTROLLER_loop();

        // check over current
        if (ABS(Foc.i_a) > UsrConfig.protect_over_current || ABS(Foc.i_b) > UsrConfig.protect_over_current
            || ABS(Foc.i_c) > UsrConfig.protect_over_current) {
            FOC_disarm();
            MCT_set_state(IDLE);
            StatuswordNew.errors.over_current = 1;
        }
        break;

    default:
        break;
    }
}

void MCT_safety_task(void)
{
    // VBUS check
    if (mFSM.state != BOOT_UP) {
        // Over voltage check
        if (Foc.v_bus > UsrConfig.protect_over_voltage) {
            StatuswordNew.errors.over_voltage = 1;
        }

        // Under voltage check
        if (Foc.v_bus < UsrConfig.protect_under_voltage) {
            StatuswordNew.errors.under_voltage = 1;
        }

        // drv over tmp
        if (read_drv_temp() > UsrConfig.protect_drv_over_tmp) {
            StatuswordNew.errors.drv_over_tmp = 1;
        }

        // ntc over tmp
        if (read_ntc_temp() > UsrConfig.protect_ntc_over_tmp) {
            StatuswordNew.errors.ntc_over_tmp = 1;
        }
    }

    watch_dog_feed();
}

void MCT_low_priority_task(void)
{
    bool isSend = false;

    // State check
    if (StatuswordOld.status.status_code != StatuswordNew.status.status_code) {
        StatuswordOld.status.status_code = StatuswordNew.status.status_code;
    }

    // Error check
    if (StatuswordOld.errors.errors_code != StatuswordNew.errors.errors_code) {
        if (StatuswordNew.errors.errors_code) {
            FOC_disarm();
            MCT_set_state(IDLE);
        }
        isSend                           = true;
        StatuswordOld.errors.errors_code = StatuswordNew.errors.errors_code;
    }

    if (isSend) {
        CAN_tx_statusword(StatuswordNew);
    }

    led_act_loop();
    CAN_comm_loop();
}

static void led_act_loop(void)
{
    static uint16_t tick       = 0;
    static uint32_t tick_100Hz = 0;

    // 100Hz
    if (get_ms_since(tick_100Hz) < 10) {
        return;
    }
    tick_100Hz = SystickCount;

    switch (mFSM.state) {
    case IDLE:
        if (tick == 0) {
            LED_ACT_SET();
        } else if (tick == 10) {
            LED_ACT_RESET();
        } else if (tick > 100) {
            tick = 0xFFFF;
        }
        break;

    case RUN:
        if (tick == 0) {
            LED_ACT_SET();
        } else if (tick == 10) {
            LED_ACT_RESET();
        } else if (tick == 20) {
            LED_ACT_SET();
        } else if (tick == 30) {
            LED_ACT_RESET();
        } else if (tick > 100) {
            tick = 0xFFFF;
        }
        break;

    case CALIBRATION:
        if (tick == 0) {
            LED_ACT_SET();
        } else if (tick == 10) {
            LED_ACT_RESET();
        } else if (tick == 20) {
            LED_ACT_SET();
        } else if (tick == 30) {
            LED_ACT_RESET();
        } else if (tick == 40) {
            LED_ACT_SET();
        } else if (tick == 50) {
            LED_ACT_RESET();
        } else if (tick > 150) {
            tick = 0xFFFF;
        }
        break;

    case ANTICOGGING:
        if (tick == 0) {
            LED_ACT_SET();
        } else if (tick == 10) {
            LED_ACT_RESET();
        } else if (tick == 20) {
            LED_ACT_SET();
        } else if (tick == 30) {
            LED_ACT_RESET();
        } else if (tick == 40) {
            LED_ACT_SET();
        } else if (tick == 50) {
            LED_ACT_RESET();
        } else if (tick == 60) {
            LED_ACT_SET();
        } else if (tick == 70) {
            LED_ACT_RESET();
        } else if (tick > 200) {
            tick = 0xFFFF;
        }
        break;

    default:
        break;
    }

    tick++;
}
