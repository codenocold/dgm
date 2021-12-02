#ifndef __HEAP_H__
#define __HEAP_H__

#include "main.h"

#define TOTAL_HEAP_SIZE		((size_t)(1024*12))		// byte

void *HEAP_malloc(size_t xWantedSize);
void HEAP_free(void *pv);
size_t HEAP_get_free_size(void);
size_t HEAP_get_minimumEver_free_size(void);

#endif
