/*!
    \file    gd32c10x_fmc.c
    \brief   FMC driver

    \version 2020-12-31, V1.0.0, firmware for GD32C10x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32c10x_fmc.h"

/* FMC mask */
#define LOW_8BITS_MASK             ((uint32_t)0x000000FFU)
#define HIGH_8BITS_MASK            ((uint32_t)0x0000FF00U)
#define LOW_8BITS_MASK1            ((uint32_t)0x00FF0000U)
#define HIGH_8BITS_MASK1           ((uint32_t)0xFF000000U)
#define LOW_16BITS_MASK            ((uint32_t)0x0000FFFFU)
#define HIGH_16BITS_MASK           ((uint32_t)0xFFFF0000U)

/* USER of option bytes mask */
#define OB_USER_MASK               ((uint8_t)0xF8U)

/* OB_WP mask */
#define OB_WP0_MASK                ((uint32_t)0x000000FFU)
#define OB_WP1_MASK                ((uint32_t)0x0000FF00U)
#define OB_WP2_MASK                ((uint32_t)0x00FF0000U)
#define OB_WP3_MASK                ((uint32_t)0xFF000000U)

/* return the FMC state */
static fmc_state_enum fmc_state_get(void);
/* check FMC ready or not */
static fmc_state_enum fmc_ready_wait(uint32_t timeout);

/* FMC main memory programming functions */

/*!
    \brief      enable pre-fetch
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_prefetch_enable(void)
{
    FMC_WS |= FMC_WS_PFEN;
}

/*!
    \brief      disable pre-fetch
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_prefetch_disable(void)
{
    FMC_WS &= ~FMC_WS_PFEN;
}

/*!
    \brief      enable IBUS cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_ibus_enable(void)
{
    FMC_WS |= FMC_WS_ICEN;
}

/*!
    \brief      disable IBUS cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_ibus_disable(void)
{
    FMC_WS &= ~FMC_WS_ICEN;
}

/*!
    \brief      enable DBUS cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_dbus_enable(void)
{
    FMC_WS |= FMC_WS_DCEN;
}

/*!
    \brief      disable DBUS cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_dbus_disable(void)
{
    FMC_WS &= ~FMC_WS_DCEN;
}

/*!
    \brief      reset IBUS cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_ibus_reset(void)
{
    FMC_WS |= FMC_WS_ICRST;
}

/*!
    \brief      reset DBUS cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_dbus_reset(void)
{
    FMC_WS |= FMC_WS_DCRST;
}

/*!
    \brief      set program width to flash memory
    \param[in]  pgw
                only one parameter can be selected which is shown as below:
      \arg        FMC_PROG_W_32B: 32-bit program width to flash memory
      \arg        FMC_PROG_W_64B: 64-bit program width to flash memory
    \param[out] none
    \retval     none
*/
void fmc_program_width_set(uint32_t pgw)
{
    uint32_t reg;

    reg = FMC_WS;
    /* configure program width to flash memory */
    reg &= ~FMC_WS_PGW;
    FMC_WS = (reg | pgw);
}

/*!
    \brief      unlock the main FMC operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_unlock(void)
{
    if(RESET != (FMC_CTL & FMC_CTL_LK)){
        /* write the FMC unlock key */
        FMC_KEY = UNLOCK_KEY0;
        FMC_KEY = UNLOCK_KEY1;
    }
}

/*!
    \brief      lock the main FMC operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_lock(void)
{
    /* set the LK bit */
    FMC_CTL |= FMC_CTL_LK;
}

/*!
    \brief      set the wait state counter value
    \param[in]  wscnt:wait state counter value
                only one parameter can be selected which is shown as below:
      \arg        FMC_WAIT_STATE_0: FMC 0 wait
      \arg        FMC_WAIT_STATE_1: FMC 1 wait
      \arg        FMC_WAIT_STATE_2: FMC 2 wait
      \arg        FMC_WAIT_STATE_3: FMC 3 wait
    \param[out] none
    \retval     none
*/
void fmc_wscnt_set(uint32_t wscnt)
{
    uint32_t reg;

    reg = FMC_WS;
    /* set the wait state counter value */
    reg &= ~FMC_WS_WSCNT;
    FMC_WS = (reg | wscnt);
}

/*!
    \brief      FMC erase page
    \param[in]  page_address: target page address
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
*/
fmc_state_enum fmc_page_erase(uint32_t page_address)
{
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state){
        FMC_CTL |= FMC_CTL_PER;
        FMC_ADDR = page_address;
        FMC_CTL |= FMC_CTL_START;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the PER bit */
        FMC_CTL &= ~FMC_CTL_PER;
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      FMC erase whole chip
    \param[in]  none
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
*/
fmc_state_enum fmc_mass_erase(void)
{
    fmc_state_enum fmc_state;
    /* wait for the FMC ready */
    fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state){
        /* start whole chip erase */
        FMC_CTL |= FMC_CTL_MER;
        FMC_CTL |= FMC_CTL_START;
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        /* reset the MER bit */
        FMC_CTL &= ~FMC_CTL_MER;
    }

    /* return the FMC state  */
    return fmc_state;
}

/*!
    \brief      FMC program a double word at the corresponding address
    \param[in]  address: address to program
    \param[in]  data: double word to program
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
*/
fmc_state_enum fmc_doubleword_program(uint32_t address, uint64_t data)
{
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state){
        /* set the PGW and PG bit to start program */
        FMC_WS |= FMC_WS_PGW;
        FMC_CTL |= FMC_CTL_PG;
        *(__IO uint64_t*)(address) = data;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the PG and PGW bit */
        FMC_CTL &= ~FMC_CTL_PG;
        FMC_WS &= ~FMC_WS_PGW;
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      FMC program a word at the corresponding address
    \param[in]  address: address to program
    \param[in]  data: word to program
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
*/
fmc_state_enum fmc_word_program(uint32_t address, uint32_t data)
{
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT); 

    if(FMC_READY == fmc_state){
        /* set the PG bit to start program */
        FMC_CTL |= FMC_CTL_PG;
        REG32(address) = data;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the PG bit */
        FMC_CTL &= ~FMC_CTL_PG;
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      unlock the option byte operation
                it is better to used in pairs with ob_lock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_unlock(void)
{
    if(RESET == (FMC_CTL & FMC_CTL_OBWEN)){
        /* write the FMC key */
        FMC_OBKEY = UNLOCK_KEY0;
        FMC_OBKEY = UNLOCK_KEY1;
    }
}

/*!
    \brief      lock the option byte operation
                it is better to used in pairs with ob_unlock after an operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_lock(void)
{
    /* reset the OBWEN bit */
    FMC_CTL &= ~FMC_CTL_OBWEN;
}

/*!
    \brief      erase the FMC option bytes
                programmer must ensure FMC & option byte are both unlocked before calling this function
    \param[in]  none
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_TOERR: timeout error
*/
fmc_state_enum ob_erase(void)
{
    uint8_t temp_spc;
    uint32_t temp;
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    /* check the option bytes security protection value */
    if(RESET == ob_security_protection_flag_get()){
        temp_spc = FMC_NSPC;
    }else{
        temp_spc = FMC_USPC;
    }

    temp = HIGH_16BITS_MASK | ((uint32_t)temp_spc);

    if(FMC_READY == fmc_state){
        /* start erase the option bytes */
        FMC_CTL |= FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_START;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state){
            /* reset the OBER bit and enable the option bytes programming */
            FMC_CTL &= ~FMC_CTL_OBER;
            FMC_CTL |= FMC_CTL_OBPG;

            /* restore the last get option byte security protection code */
            OB_SPC_USER = temp;

            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the OBPG bit */
            FMC_CTL &= ~FMC_CTL_OBPG;
        }else{
            /* reset the OBER bit */
            FMC_CTL &= ~FMC_CTL_OBER;
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      enable write protection
    \param[in]  ob_wp: specify sector to be write protected
                one or more parameters can be selected which are shown as below:
      \arg        OB_WP_NONE: disable all write protection
      \arg        OB_WP_x(x=0..31): write protect specify sector
      \arg        OB_WP_ALL: write protect all sector
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_TOERR: timeout error
*/
fmc_state_enum ob_write_protection_enable(uint32_t ob_wp)
{
    uint32_t i;
    uint32_t op_byte[4];
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    for(i = 0U; i < 4U; i++){
        op_byte[i] = OP_BYTE(i);
    }
    ob_wp      = (uint32_t)(~ob_wp);
    op_byte[2] = (ob_wp & LOW_8BITS_MASK) | ((ob_wp & HIGH_8BITS_MASK) << 8U);
    op_byte[3] = ((ob_wp & LOW_8BITS_MASK1) >> 16U) | ((ob_wp & HIGH_8BITS_MASK1) >> 8U);

    if(FMC_READY == fmc_state){
        /* start erase the option byte */
        FMC_CTL |= FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_START;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state){
            /* reset the OBER bit and enable the option bytes programming */
            FMC_CTL &= ~FMC_CTL_OBER;
            FMC_CTL |= FMC_CTL_OBPG;

            for(i = 0U; i < 4U; i++){
                OP_BYTE(i) = op_byte[i];
                /* wait for the FMC ready */
                fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
                if(FMC_READY != fmc_state){
                    break;
                }
            }

            /* reset the OBPG bit */
            FMC_CTL &= ~FMC_CTL_OBPG;
        }else{
            /* reset the OBER bit */
            FMC_CTL &= ~FMC_CTL_OBER;
        }
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      configure security protection
    \param[in]  ob_spc: specify security protection code
                only one parameter can be selected which is shown as below:
      \arg        FMC_NSPC: no security protection
      \arg        FMC_USPC: under security protection
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_TOERR: timeout error
*/
fmc_state_enum ob_security_protection_config(uint8_t ob_spc)
{
    uint8_t i;
    uint32_t op_byte[4];
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    for(i = 0U; i < 4U; i++){
        op_byte[i] = OP_BYTE(i);
    }
    op_byte[0] = ((uint32_t)(ob_spc)) | ((op_byte[0] & HIGH_16BITS_MASK));

    if(FMC_READY == fmc_state){
        /* start erase the option byte */
        FMC_CTL |= FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_START;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state){
            /* reset the OBER bit and enable the option bytes programming */
            FMC_CTL &= ~FMC_CTL_OBER;
            FMC_CTL |= FMC_CTL_OBPG;

            for(i = 0U; i < 4U; i++){
                OP_BYTE(i) = op_byte[i];
                /* wait for the FMC ready */
                fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
                if(FMC_READY != fmc_state){
                    break;
                }
            }

            /* reset the OBPG bit */
            FMC_CTL &= ~FMC_CTL_OBPG;
        }else{
            /* reset the OBER bit */
            FMC_CTL &= ~FMC_CTL_OBER;
        }
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      program option bytes USER
                programmer must ensure FMC & option bytes are both unlocked before calling this function
    \param[in]  ob_fwdgt: option bytes free watchdog value
                only one parameter can be selected which is shown as below:
      \arg        OB_FWDGT_SW: software free watchdog
      \arg        OB_FWDGT_HW: hardware free watchdog
    \param[in]  ob_deepsleep: option bytes deepsleep reset value
                only one parameter can be selected which is shown as below:
      \arg        OB_DEEPSLEEP_NRST: no reset when entering deepsleep mode
      \arg        OB_DEEPSLEEP_RST: generate a reset instead of entering deepsleep mode 
    \param[in]  ob_stdby: option bytes standby reset value
                only one parameter can be selected which is shown as below:
      \arg        OB_STDBY_NRST: no reset when entering standby mode
      \arg        OB_STDBY_RST: generate a reset instead of entering standby mode
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_TOERR: timeout error
*/
fmc_state_enum ob_user_write(uint8_t ob_fwdgt, uint8_t ob_deepsleep, uint8_t ob_stdby)
{
    uint8_t i;
    uint32_t temp;
    uint32_t op_byte[4];
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    for(i = 0U; i < 4U; i++){
        op_byte[i] = OP_BYTE(i);
    }
    temp = ((uint8_t)((uint8_t)((uint8_t)(ob_fwdgt) | ob_deepsleep) | ob_stdby) | (OB_USER_MASK));
    op_byte[0] = ((uint32_t)(temp) << 16U) | ((op_byte[0] & LOW_16BITS_MASK));

    if(FMC_READY == fmc_state){
        /* start erase the option byte */
        FMC_CTL |= FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_START;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state){
            /* reset the OBER bit and enable the option bytes programming */
            FMC_CTL &= ~FMC_CTL_OBER;
            FMC_CTL |= FMC_CTL_OBPG;

            for(i = 0U; i < 4U; i++){
                OP_BYTE(i) = op_byte[i];
                /* wait for the FMC ready */
                fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
                if(FMC_READY != fmc_state){
                    break;
                }
            }

            /* reset the OBPG bit */
            FMC_CTL &= ~FMC_CTL_OBPG;
        }else{
            /* reset the OBER bit */
            FMC_CTL &= ~FMC_CTL_OBER;
        }
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      program option bytes data
    \param[in]  ob_data: the byte to be programmed
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_TOERR: timeout error
*/

fmc_state_enum ob_data_program(uint16_t ob_data)
{
    uint8_t i;
    uint32_t op_byte[4];
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    for(i = 0U; i < 4U; i++){
        op_byte[i] = OP_BYTE(i);
    }
    op_byte[1] = (uint32_t)((ob_data & LOW_8BITS_MASK) | ((ob_data & HIGH_8BITS_MASK) << 8U));

    if(FMC_READY == fmc_state){
        /* start erase the option byte */
        FMC_CTL |= FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_START;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state){
            /* reset the OBER bit and enable the option bytes programming */
            FMC_CTL &= ~FMC_CTL_OBER;
            FMC_CTL |= FMC_CTL_OBPG;

            for(i = 0U; i < 4U; i++){
                OP_BYTE(i) = op_byte[i];
                /* wait for the FMC ready */
                fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
                if(FMC_READY != fmc_state){
                    break;
                }
            }

            /* reset the OBPG bit */
            FMC_CTL &= ~FMC_CTL_OBPG;
        }else{
            /* reset the OBER bit */
            FMC_CTL &= ~FMC_CTL_OBER;
        }
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      get OB_USER in register FMC_OBSTAT
    \param[in]  none
    \param[out] none
    \retval     the FMC user option bytes values
*/
uint8_t ob_user_get(void)
{
    /* return the FMC user option bytes value */
    return (uint8_t)(FMC_OBSTAT >> 2U);
}

/*!
    \brief      get OB_DATA in register FMC_OBSTAT
    \param[in]  none
    \param[out] none
    \retval     ob_data
*/
uint16_t ob_data_get(void)
{
    return (uint16_t)(FMC_OBSTAT >> 10U);
}

/*!
    \brief      get the FMC option byte write protection (OB_WP) in register FMC_WP
    \param[in]  none
    \param[out] none
    \retval     the FMC write protection option bytes value
*/
uint32_t ob_write_protection_get(void)
{
    /* return the FMC write protection option bytes value */
    return FMC_WP;
}

/*!
    \brief      get the FMC option bytes security protection state
    \param[in]  none
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus ob_security_protection_flag_get(void)
{
    FlagStatus spc_state = RESET;

    if(RESET != (FMC_OBSTAT & FMC_OBSTAT_SPC)){
        spc_state = SET;
    }else{
        spc_state = RESET;
    }
    return spc_state;
}

/*!
    \brief      check flag is set or not
    \param[in]  flag: check FMC flag
                only one parameter can be selected which is shown as below:
      \arg        FMC_FLAG_BUSY: FMC busy flag bit
      \arg        FMC_FLAG_PGERR: FMC operation error flag bit
      \arg        FMC_FLAG_PGAERR: FMC program alignment error flag bit
      \arg        FMC_FLAG_WPERR: FMC erase/program protection error flag bit
      \arg        FMC_FLAG_END: FMC end of operation flag bit
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus fmc_flag_get(uint32_t flag)
{
    FlagStatus status = RESET;

    if(FMC_STAT & flag){
        status = SET;
    }
    /* return the state of corresponding FMC flag */
    return status;
}

/*!
    \brief      clear the FMC flag
    \param[in]  flag: clear FMC flag
                one or more parameters can be selected which is shown as below:
      \arg        FMC_FLAG_PGERR: FMC operation error flag bit
      \arg        FMC_FLAG_PGAERR: FMC program alignment error flag bit
      \arg        FMC_FLAG_WPERR: FMC erase/program protection error flag bit
      \arg        FMC_FLAG_END: FMC end of operation flag bit
    \param[out] none
    \retval     none
*/
void fmc_flag_clear(uint32_t flag)
{
    /* clear the flags */
    FMC_STAT = flag;
}

/*!
    \brief      enable FMC interrupt
    \param[in]  interrupt: the FMC interrupt source
                only one parameter can be selected which is shown as below:
      \arg        FMC_INT_END: enable FMC end of program interrupt
      \arg        FMC_INT_ERR: enable FMC error interrupt
    \param[out] none
    \retval     none
*/
void fmc_interrupt_enable(uint32_t interrupt)
{
    FMC_CTL |= interrupt;
}

/*!
    \brief      disable FMC interrupt
    \param[in]  interrupt: the FMC interrupt source
                only one parameter can be selected which is shown as below:
      \arg        FMC_INT_END: enable FMC end of program interrupt
      \arg        FMC_INT_ERR: enable FMC error interrupt
    \param[out] none
    \retval     none
*/
void fmc_interrupt_disable(uint32_t interrupt)
{
    FMC_CTL &= ~(uint32_t)interrupt;
}

/*!
    \brief      get FMC interrupt flag state
    \param[in]  flag: FMC interrupt flags, refer to fmc_interrupt_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        FMC_INT_FLAG_PGERR: FMC operation error interrupt flag bit
      \arg        FMC_FLAG_PGAERR: FMC program alignment error flag bit
      \arg        FMC_INT_FLAG_WPERR: FMC erase/program protection error interrupt flag bit
      \arg        FMC_INT_FLAG_END: FMC end of operation interrupt flag bit
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus fmc_interrupt_flag_get(uint32_t flag)
{
    FlagStatus status = RESET;

    if(FMC_STAT & flag){
        status = SET;
    }
    /* return the state of corresponding FMC flag */
    return status;
}

/*!
    \brief      clear FMC interrupt flag state
    \param[in]  flag: FMC interrupt flags, refer to can_interrupt_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        FMC_INT_FLAG_PGERR: FMC operation error interrupt flag bit
      \arg        FMC_FLAG_PGAERR: FMC program alignment error flag bit
      \arg        FMC_INT_FLAG_WPERR: FMC erase/program protection error interrupt flag bit
      \arg        FMC_INT_FLAG_END: FMC end of operation interrupt flag bit
    \param[out] none
    \retval     none
*/
void fmc_interrupt_flag_clear(uint32_t flag)
{
    /* clear the flag */
    FMC_STAT = flag;
}

/*!
    \brief      get the FMC state
    \param[in]  none
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
*/
static fmc_state_enum fmc_state_get(void)
{
    fmc_state_enum fmc_state = FMC_READY;
  
    if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_BUSY)){
        fmc_state = FMC_BUSY;
    }else{
        if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_WPERR)){
            fmc_state = FMC_WPERR;
        }else{
            if((uint32_t)0x00U != (FMC_STAT & (FMC_STAT_PGERR))){
                fmc_state = FMC_PGERR;
            }else{
                if((uint32_t)0x00U != (FMC_STAT & (FMC_STAT_PGAERR))){
                    fmc_state = FMC_PGAERR;
                }
            }
        }
  }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      check whether FMC is ready or not
    \param[in]  timeout: count of loop
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
*/
static fmc_state_enum fmc_ready_wait(uint32_t timeout)
{
    fmc_state_enum fmc_state = FMC_BUSY;

    /* wait for FMC ready */
    do{
        /* get FMC state */
        fmc_state = fmc_state_get();
        timeout--;
    }while((FMC_BUSY == fmc_state) && (0x00U != timeout));
    
    if(FMC_BUSY == fmc_state){
        fmc_state = FMC_TOERR;
    }
    /* return the FMC state */
    return fmc_state;
}
