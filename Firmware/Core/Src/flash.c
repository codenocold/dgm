#include "flash.h"

#define FLASH_TIMEOUT_VALUE             1000000U

#define FLASH_KEY1                0x45670123U              /*!< Flash key1 */
#define FLASH_KEY2                0xCDEF89ABU              /*!< Flash key2: used with FLASH_KEY1 to unlock the FLASH registers access */

typedef enum{
	FLASH_CACHE_DISABLED = 0,
	FLASH_CACHE_ICACHE_ENABLED,
	FLASH_CACHE_DCACHE_ENABLED,
	FLASH_CACHE_ICACHE_DCACHE_ENABLED
} FLASH_CacheTypeDef;

/** @defgroup FLASH_Flags FLASH Flags Definition
  * @{
  */
#define FLASH_FLAG_EOP            FLASH_SR_EOP             /*!< FLASH End of operation flag */
#define FLASH_FLAG_OPERR          FLASH_SR_OPERR           /*!< FLASH Operation error flag */
#define FLASH_FLAG_PROGERR        FLASH_SR_PROGERR         /*!< FLASH Programming error flag */
#define FLASH_FLAG_WRPERR         FLASH_SR_WRPERR          /*!< FLASH Write protection error flag */
#define FLASH_FLAG_PGAERR         FLASH_SR_PGAERR          /*!< FLASH Programming alignment error flag */
#define FLASH_FLAG_SIZERR         FLASH_SR_SIZERR          /*!< FLASH Size error flag  */
#define FLASH_FLAG_PGSERR         FLASH_SR_PGSERR          /*!< FLASH Programming sequence error flag */
#define FLASH_FLAG_MISERR         FLASH_SR_MISERR          /*!< FLASH Fast programming data miss error flag */
#define FLASH_FLAG_FASTERR        FLASH_SR_FASTERR         /*!< FLASH Fast programming error flag */
#define FLASH_FLAG_RDERR          FLASH_SR_RDERR           /*!< FLASH PCROP read error flag */
#define FLASH_FLAG_OPTVERR        FLASH_SR_OPTVERR         /*!< FLASH Option validity error flag  */
#define FLASH_FLAG_BSY            FLASH_SR_BSY             /*!< FLASH Busy flag */
#define FLASH_FLAG_ECCC           FLASH_ECCR_ECCC          /*!< FLASH ECC correction in 64 LSB bits */
#define FLASH_FLAG_ECCD           FLASH_ECCR_ECCD          /*!< FLASH ECC detection in 64 LSB bits */
#define FLASH_FLAG_SR_ERRORS      (FLASH_FLAG_OPERR   | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | \
                                   FLASH_FLAG_PGAERR  | FLASH_FLAG_SIZERR  | FLASH_FLAG_PGSERR | \
                                   FLASH_FLAG_MISERR  | FLASH_FLAG_FASTERR | FLASH_FLAG_RDERR  | \
                                   FLASH_FLAG_OPTVERR)
								   
#define FLASH_FLAG_ECCR_ERRORS    (FLASH_FLAG_ECCC | FLASH_FLAG_ECCD)

#define __HAL_FLASH_INSTRUCTION_CACHE_RESET()   do { SET_BIT(FLASH->ACR, FLASH_ACR_ICRST);   \
                                                     CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICRST); \
                                                   } while (0)
#define __HAL_FLASH_INSTRUCTION_CACHE_ENABLE()  SET_BIT(FLASH->ACR, FLASH_ACR_ICEN)
#define __HAL_FLASH_INSTRUCTION_CACHE_DISABLE() CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICEN)
						   
#define __HAL_FLASH_DATA_CACHE_RESET()          do { SET_BIT(FLASH->ACR, FLASH_ACR_DCRST);   \
                                                     CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCRST); \
                                                   } while (0)
#define __HAL_FLASH_DATA_CACHE_ENABLE()         SET_BIT(FLASH->ACR, FLASH_ACR_DCEN)
#define __HAL_FLASH_DATA_CACHE_DISABLE()        CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCEN)

#define __HAL_FLASH_GET_FLAG(__FLAG__)          ((((__FLAG__) & FLASH_FLAG_ECCR_ERRORS) != 0U) ? \
                                                 (READ_BIT(FLASH->ECCR, (__FLAG__)) == (__FLAG__)) : \
                                                 (READ_BIT(FLASH->SR,   (__FLAG__)) == (__FLAG__)))
												 
#define __HAL_FLASH_CLEAR_FLAG(__FLAG__)        do { if(((__FLAG__) & FLASH_FLAG_ECCR_ERRORS) != 0U) { SET_BIT(FLASH->ECCR, ((__FLAG__) & FLASH_FLAG_ECCR_ERRORS)); }\
                                                     if(((__FLAG__) & ~(FLASH_FLAG_ECCR_ERRORS)) != 0U) { WRITE_REG(FLASH->SR, ((__FLAG__) & ~(FLASH_FLAG_ECCR_ERRORS))); }\
                                                   } while (0)												 

static int flash_wait_for_last_operation(void)
{
	uint32_t error;
	uint32_t timeout_cnt;
	static const uint32_t timeout_cnt_num = FLASH_TIMEOUT_VALUE;

	timeout_cnt = 0;
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)){
		timeout_cnt ++;
		if(timeout_cnt > timeout_cnt_num){
			return -1;
		}
	}

	/* Check FLASH operation error flags */
	error = (FLASH->SR & FLASH_FLAG_SR_ERRORS);
	if (error != 0u){
		/* Clear error programming flags */
		__HAL_FLASH_CLEAR_FLAG(error);
		return -2;
	}

	/* Check FLASH End of Operation flag  */
	if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP)){
		/* Clear FLASH End of Operation pending bit */
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
	}

	return 0;
}
												   
int FLASH_unlock(void)
{
	int status = 0;

	if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U){
		/* Authorize the FLASH Registers access */
		WRITE_REG(FLASH->KEYR, FLASH_KEY1);
		WRITE_REG(FLASH->KEYR, FLASH_KEY2);

		/* verify Flash is unlocked */
		if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U){
			status = -1;
		}
	}

	return status;
}

int FLASH_lock(void)
{
	int status = -1;

	/* Set the LOCK Bit to lock the FLASH Registers access */
	SET_BIT(FLASH->CR, FLASH_CR_LOCK);

	/* verify Flash is locked */
	if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U){
		status = 0;
	}

	return status;
}

int FLASH_erase_page(uint32_t page, uint32_t num_of_pages)
{
	int status;
	uint32_t page_index;
	FLASH_CacheTypeDef CacheToReactivate;

	/* Wait for last operation to be completed */
	status = flash_wait_for_last_operation();
	
	if(status == 0){
		/* Deactivate the cache if they are activated to avoid data misbehavior */
		if (READ_BIT(FLASH->ACR, FLASH_ACR_ICEN) != 0U){
			/* Disable instruction cache  */
			__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

			if (READ_BIT(FLASH->ACR, FLASH_ACR_DCEN) != 0U){
				/* Disable data cache  */
				__HAL_FLASH_DATA_CACHE_DISABLE();
				CacheToReactivate = FLASH_CACHE_ICACHE_DCACHE_ENABLED;
			}else{
				CacheToReactivate = FLASH_CACHE_ICACHE_ENABLED;
			}
		}else if (READ_BIT(FLASH->ACR, FLASH_ACR_DCEN) != 0U){
			/* Disable data cache  */
			__HAL_FLASH_DATA_CACHE_DISABLE();
			CacheToReactivate = FLASH_CACHE_DCACHE_ENABLED;
		}else{
			CacheToReactivate = FLASH_CACHE_DISABLED;
		}
		
		for (page_index = page; page_index < (page + num_of_pages); page_index++){
			/* Proceed to erase the page */
			MODIFY_REG(FLASH->CR, FLASH_CR_PNB, ((page_index & 0xFFU) << FLASH_CR_PNB_Pos));
			SET_BIT(FLASH->CR, FLASH_CR_PER);
			SET_BIT(FLASH->CR, FLASH_CR_STRT);

			/* Wait for last operation to be completed */
			status = flash_wait_for_last_operation();

			/* If the erase operation is completed, disable the PER Bit */
			CLEAR_BIT(FLASH->CR, (FLASH_CR_PER | FLASH_CR_PNB));

			if (status != 0){
				break;
			}
		}
		
		/* Flush instruction cache  */
		if ((CacheToReactivate == FLASH_CACHE_ICACHE_ENABLED) || (CacheToReactivate == FLASH_CACHE_ICACHE_DCACHE_ENABLED)){
			/* Reset instruction cache */
			__HAL_FLASH_INSTRUCTION_CACHE_RESET();
			/* Enable instruction cache */
			__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
		}

		/* Flush data cache */
		if ((CacheToReactivate == FLASH_CACHE_DCACHE_ENABLED) || (CacheToReactivate == FLASH_CACHE_ICACHE_DCACHE_ENABLED)){
			/* Reset data cache */
			__HAL_FLASH_DATA_CACHE_RESET();
			/* Enable data cache */
			__HAL_FLASH_DATA_CACHE_ENABLE();
		}
	}
	
	return status;
}

int FLASH_program(uint32_t Address, uint64_t Data)
{
	int status;
	uint32_t prog_bit = 0;

	/* Wait for last operation to be completed */
	status = flash_wait_for_last_operation();

	if (status == 0){
		/* Set PG bit */
		SET_BIT(FLASH->CR, FLASH_CR_PG);

		/* Program first word */
		*(uint32_t *)Address = (uint32_t)Data;

		/* Barrier to ensure programming is performed in 2 steps, in right order
		(independently of compiler optimization behavior) */
		__ISB();

		/* Program second word */
		*(uint32_t *)(Address + 4U) = (uint32_t)(Data >> 32U);
		
		prog_bit = FLASH_CR_PG;

		/* Wait for last operation to be completed */
		status = flash_wait_for_last_operation();
	}

	/* If the program operation is completed, disable the PG or FSTPG Bit */
    if (prog_bit != 0U){
		CLEAR_BIT(FLASH->CR, prog_bit);
    }

	/* return status */
	return status;
}
