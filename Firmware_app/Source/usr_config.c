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

#include "usr_config.h"
#include "controller.h"
#include "heap.h"
#include "util.h"
#include <string.h>

tUsrConfig   UsrConfig;
tCoggingMap *pCoggingMap = NULL;

void USR_CONFIG_set_default_config(void)
{
    // Motor
    UsrConfig.invert_motor_dir       = 0;
    UsrConfig.inertia                = 0.0001f;
    UsrConfig.torque_constant        = 0.051274;
    UsrConfig.motor_pole_pairs       = 4;
    UsrConfig.motor_phase_resistance = 0.693f;
    UsrConfig.motor_phase_inductance = 385e-6f;
    UsrConfig.current_limit          = 4;
    UsrConfig.velocity_limit         = 60;

    // Encoder
    UsrConfig.calib_current = 5.0f;
    UsrConfig.calib_voltage = 3.0f;

    // Anticogging
    UsrConfig.anticogging_enable = 1;

    // Controller
    UsrConfig.control_mode           = CONTROL_MODE_POSITION_PROFILE;
    UsrConfig.pos_gain               = 80.0f;
    UsrConfig.vel_gain               = 0.03f;
    UsrConfig.vel_integrator_gain    = 0.01f;
    UsrConfig.current_ctrl_bw        = 20000;
    UsrConfig.sync_target_enable     = 0;
    UsrConfig.target_velcity_window  = 0.5f;
    UsrConfig.target_position_window = 0.01f;
    UsrConfig.torque_ramp_rate       = 0.1f;
    UsrConfig.velocity_ramp_rate     = 100.0f;
    UsrConfig.position_filter_bw     = 2;
    UsrConfig.profile_velocity       = 30;
    UsrConfig.profile_accel          = 50;
    UsrConfig.profile_decel          = 50;

    // Protect
    UsrConfig.protect_under_voltage = 12;
    UsrConfig.protect_over_voltage  = 30;
    UsrConfig.protect_over_current  = 8;
    UsrConfig.protect_i_bus_max     = 5;

    // CAN
    UsrConfig.node_id               = 1;
    UsrConfig.can_baudrate          = CAN_BAUDRATE_500K;
    UsrConfig.heartbeat_consumer_ms = 0;
    UsrConfig.heartbeat_producer_ms = 0;

    // Encoder
    UsrConfig.calib_valid = 0;
}

int USR_CONFIG_erease_config(void)
{
    uint32_t       addr;
    fmc_state_enum status;

    fmc_unlock();

    // Erase
    for (addr = USR_CONFIG_ADDR; addr < (USR_CONFIG_ADDR + USR_CONFIG_MAX_SIZE); addr += PAGE_SIZE) {
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
        status = fmc_page_erase(addr);
        if (status != FMC_READY) {
            fmc_lock();
            return -1;
        }
    }

    fmc_lock();

    // Check
    for (addr = USR_CONFIG_ADDR; addr < (USR_CONFIG_ADDR + USR_CONFIG_MAX_SIZE); addr += 4) {
        if (0xFFFFFFFF != *((uint32_t *) addr)) {
            return -2;
        }
    }

    return 0;
}

int USR_CONFIG_read_config(void)
{
    int state = 0;

    memcpy(&UsrConfig, (uint8_t *) USR_CONFIG_ADDR, sizeof(tUsrConfig));

    uint32_t crc;
    crc = crc32((uint8_t *) &UsrConfig, sizeof(tUsrConfig) - 4);
    if (crc != UsrConfig.crc) {
        state = -1;
    }

    return state;
}

int USR_CONFIG_save_config(void)
{
    // Erase
    if (USR_CONFIG_erease_config()) {
        return -1;
    }

    fmc_unlock();

    // Program
    UsrConfig.crc   = crc32((uint8_t *) &UsrConfig, sizeof(tUsrConfig) - 4);
    uint32_t *pData = (uint32_t *) &UsrConfig;
    for (int i = 0; i < sizeof(tUsrConfig) / 4; i++) {
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
        if (FMC_READY != fmc_word_program(USR_CONFIG_ADDR + i * 4, *(pData + i))) {
            fmc_lock();
            return -2;
        }
    }

    fmc_lock();

    return 0;
}

void USR_CONFIG_set_default_cogging_map(void)
{
    if (pCoggingMap == NULL) {
        pCoggingMap = HEAP_malloc(sizeof(tCoggingMap));
    }

    for (int i = 0; i < COGGING_MAP_NUM; i++) {
        pCoggingMap->map[i] = 0;
    }
}

int USR_CONFIG_erease_cogging_map(void)
{
    uint32_t       addr;
    fmc_state_enum status;

    fmc_unlock();

    // Erase
    for (addr = COGGING_MAP_ADDR; addr < (COGGING_MAP_ADDR + COGGING_MAP_MAX_SIZE); addr += PAGE_SIZE) {
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
        status = fmc_page_erase(addr);
        if (status != FMC_READY) {
            fmc_lock();
            return -1;
        }
    }

    fmc_lock();

    // Check
    for (addr = COGGING_MAP_ADDR; addr < (COGGING_MAP_ADDR + COGGING_MAP_MAX_SIZE); addr += 4) {
        if (0xFFFFFFFF != *((uint32_t *) addr)) {
            return -2;
        }
    }

    return 0;
}

int USR_CONFIG_read_cogging_map(void)
{
    int state = 0;

    if (pCoggingMap == NULL) {
        pCoggingMap = HEAP_malloc(sizeof(tCoggingMap));
    }

    memcpy(pCoggingMap, (uint8_t *) COGGING_MAP_ADDR, sizeof(tCoggingMap));

    uint32_t crc;
    crc = crc32((uint8_t *) pCoggingMap, sizeof(tCoggingMap) - 4);
    if (crc != pCoggingMap->crc) {
        state = -1;
    }

    return state;
}

int USR_CONFIG_save_cogging_map(void)
{
    // Erase
    if (USR_CONFIG_erease_cogging_map()) {
        return -1;
    }

    fmc_unlock();

    // Program
    pCoggingMap->crc = crc32((uint8_t *) pCoggingMap, sizeof(tCoggingMap) - 4);
    uint32_t *pData  = (uint32_t *) pCoggingMap;
    for (int i = 0; i < sizeof(tCoggingMap) / 4; i++) {
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
        if (FMC_READY != fmc_word_program(COGGING_MAP_ADDR + i * 4, *(pData + i))) {
            fmc_lock();
            return -2;
        }
    }

    fmc_lock();

    return 0;
}
