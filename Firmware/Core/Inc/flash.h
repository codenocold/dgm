#ifndef __FLASH_H__
#define __FLASH_H__

#include "main.h"

#define FLASH_PAGE_SIZE                 0x800U  /* 2 KB */

int FLASH_unlock(void);
int FLASH_lock(void);
int FLASH_erase_page(uint32_t page, uint32_t num_of_pages);
int FLASH_program(uint32_t Address, uint64_t Data);

#endif
