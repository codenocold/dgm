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

#include "heap.h"
#include <stdlib.h>

#define portBYTE_ALIGNMENT      8
#define portBYTE_ALIGNMENT_MASK (0x0007)

/* Block sizes must not get too small. */
#define heapMINIMUM_BLOCK_SIZE  ((size_t) (xHeapStructSize << 1))

/* Assumes 8bit bytes! */
#define heapBITS_PER_BYTE       ((size_t) 8)

static uint8_t ucHeap[TOTAL_HEAP_SIZE];

/* Define the linked list structure.  This is used to link free blocks in order
of their memory address. */
typedef struct A_BLOCK_LINK
{
    struct A_BLOCK_LINK *pxNextFreeBlock; /*<< The next free block in the list. */
    size_t               xBlockSize;      /*<< The size of the free block. */
} BlockLink_t;

/*-----------------------------------------------------------*/

/*
 * Inserts a block of memory that is being freed into the correct position in
 * the list of free memory blocks.  The block being freed will be merged with
 * the block in front it and/or the block behind it if the memory blocks are
 * adjacent to each other.
 */
static void prvInsertBlockIntoFreeList(BlockLink_t *pxBlockToInsert);

/*
 * Called automatically to setup the required heap structures the first time
 * pvPortMalloc() is called.
 */
static void prvHeapInit(void);

/*-----------------------------------------------------------*/

/* The size of the structure placed at the beginning of each allocated memory
block must by correctly byte aligned. */
static const size_t xHeapStructSize = (sizeof(BlockLink_t) + ((size_t) (portBYTE_ALIGNMENT - 1)))
                                      & ~((size_t) portBYTE_ALIGNMENT_MASK);

/* Create a couple of list links to mark the start and end of the list. */
static BlockLink_t xStart, *pxEnd = NULL;

/* Keeps track of the number of free bytes remaining, but says nothing about
fragmentation. */
static size_t xFreeBytesRemaining            = 0U;
static size_t xMinimumEverFreeBytesRemaining = 0U;

/* Gets set to the top bit of an size_t type.  When this bit in the xBlockSize
member of an BlockLink_t structure is set then the block belongs to the
application.  When the bit is free the block is still part of the free heap
space. */
static size_t xBlockAllocatedBit = 0;

/*-----------------------------------------------------------*/

void *HEAP_malloc(size_t xWantedSize)
{
    BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
    void        *pvReturn = NULL;

    /* If this is the first call to malloc then the heap will require
    initialisation to setup the list of free blocks. */
    if (pxEnd == NULL) {
        prvHeapInit();
    }

    /* Check the requested block size is not so large that the top bit is
    set.  The top bit of the block size member of the BlockLink_t structure
    is used to determine who owns the block - the application or the
    kernel, so it must be free. */
    if ((xWantedSize & xBlockAllocatedBit) == 0) {
        /* The wanted size is increased so it can contain a BlockLink_t
        structure in addition to the requested amount of bytes. */
        if (xWantedSize > 0) {
            xWantedSize += xHeapStructSize;

            /* Ensure that blocks are always aligned to the required number
            of bytes. */
            if ((xWantedSize & portBYTE_ALIGNMENT_MASK) != 0x00) {
                /* Byte alignment required. */
                xWantedSize += (portBYTE_ALIGNMENT - (xWantedSize & portBYTE_ALIGNMENT_MASK));
            }
        }

        if ((xWantedSize > 0) && (xWantedSize <= xFreeBytesRemaining)) {
            /* Traverse the list from the start    (lowest address) block until
            one    of adequate size is found. */
            pxPreviousBlock = &xStart;
            pxBlock         = xStart.pxNextFreeBlock;
            while ((pxBlock->xBlockSize < xWantedSize) && (pxBlock->pxNextFreeBlock != NULL)) {
                pxPreviousBlock = pxBlock;
                pxBlock         = pxBlock->pxNextFreeBlock;
            }

            /* If the end marker was reached then a block of adequate size
            was    not found. */
            if (pxBlock != pxEnd) {
                /* Return the memory space pointed to - jumping over the
                BlockLink_t structure at its start. */
                pvReturn = (void *) (((uint8_t *) pxPreviousBlock->pxNextFreeBlock) + xHeapStructSize);

                /* This block is being returned for use so must be taken out
                of the list of free blocks. */
                pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

                /* If the block is larger than required it can be split into
                two. */
                if ((pxBlock->xBlockSize - xWantedSize) > heapMINIMUM_BLOCK_SIZE) {
                    /* This block is to be split into two.  Create a new
                    block following the number of bytes requested. The void
                    cast is used to prevent byte alignment warnings from the
                    compiler. */
                    pxNewBlockLink = (void *) (((uint8_t *) pxBlock) + xWantedSize);

                    /* Calculate the sizes of two blocks split from the
                    single block. */
                    pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
                    pxBlock->xBlockSize        = xWantedSize;

                    /* Insert the new block into the list of free blocks. */
                    prvInsertBlockIntoFreeList(pxNewBlockLink);
                }

                xFreeBytesRemaining -= pxBlock->xBlockSize;

                if (xFreeBytesRemaining < xMinimumEverFreeBytesRemaining) {
                    xMinimumEverFreeBytesRemaining = xFreeBytesRemaining;
                }

                /* The block is being returned - it is allocated and owned
                by the application and has no "next" block. */
                pxBlock->xBlockSize |= xBlockAllocatedBit;
                pxBlock->pxNextFreeBlock = NULL;
            }
        }
    }

    return pvReturn;
}
/*-----------------------------------------------------------*/

void HEAP_free(void *pv)
{
    uint8_t     *puc = (uint8_t *) pv;
    BlockLink_t *pxLink;

    if (pv != NULL) {
        /* The memory being freed will have an BlockLink_t structure immediately
        before it. */
        puc -= xHeapStructSize;

        /* This casting is to keep the compiler from issuing warnings. */
        pxLink = (void *) puc;

        if ((pxLink->xBlockSize & xBlockAllocatedBit) != 0) {
            if (pxLink->pxNextFreeBlock == NULL) {
                /* The block is being returned to the heap - it is no longer
                allocated. */
                pxLink->xBlockSize &= ~xBlockAllocatedBit;

                /* Add this block to the list of free blocks. */
                xFreeBytesRemaining += pxLink->xBlockSize;
                prvInsertBlockIntoFreeList(((BlockLink_t *) pxLink));
            }
        }
    }
}
/*-----------------------------------------------------------*/

size_t HEAP_get_free_size(void)
{
    return xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

size_t HEAP_get_minimumEver_free_size(void)
{
    return xMinimumEverFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

static void prvHeapInit(void)
{
    BlockLink_t *pxFirstFreeBlock;
    uint8_t     *pucAlignedHeap;
    size_t       uxAddress;
    size_t       xTotalHeapSize = TOTAL_HEAP_SIZE;

    /* Ensure the heap starts on a correctly aligned boundary. */
    uxAddress = (size_t) ucHeap;

    if ((uxAddress & portBYTE_ALIGNMENT_MASK) != 0) {
        uxAddress += (portBYTE_ALIGNMENT - 1);
        uxAddress &= ~((size_t) portBYTE_ALIGNMENT_MASK);
        xTotalHeapSize -= uxAddress - (size_t) ucHeap;
    }

    pucAlignedHeap = (uint8_t *) uxAddress;

    /* xStart is used to hold a pointer to the first item in the list of free
    blocks.  The void cast is used to prevent compiler warnings. */
    xStart.pxNextFreeBlock = (void *) pucAlignedHeap;
    xStart.xBlockSize      = (size_t) 0;

    /* pxEnd is used to mark the end of the list of free blocks and is inserted
    at the end of the heap space. */
    uxAddress = ((size_t) pucAlignedHeap) + xTotalHeapSize;
    uxAddress -= xHeapStructSize;
    uxAddress &= ~((size_t) portBYTE_ALIGNMENT_MASK);
    pxEnd                  = (void *) uxAddress;
    pxEnd->xBlockSize      = 0;
    pxEnd->pxNextFreeBlock = NULL;

    /* To start with there is a single free block that is sized to take up the
    entire heap space, minus the space taken by pxEnd. */
    pxFirstFreeBlock                  = (void *) pucAlignedHeap;
    pxFirstFreeBlock->xBlockSize      = uxAddress - (size_t) pxFirstFreeBlock;
    pxFirstFreeBlock->pxNextFreeBlock = pxEnd;

    /* Only one block exists - and it covers the entire usable heap space. */
    xMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
    xFreeBytesRemaining            = pxFirstFreeBlock->xBlockSize;

    /* Work out the position of the top bit in a size_t variable. */
    xBlockAllocatedBit = ((size_t) 1) << ((sizeof(size_t) * heapBITS_PER_BYTE) - 1);
}
/*-----------------------------------------------------------*/

static void prvInsertBlockIntoFreeList(BlockLink_t *pxBlockToInsert)
{
    BlockLink_t *pxIterator;
    uint8_t     *puc;

    /* Iterate through the list until a block is found that has a higher address
    than the block being inserted. */
    for (pxIterator = &xStart; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock) {
        /* Nothing to do here, just iterate to the right position. */
    }

    /* Do the block being inserted, and the block it is being inserted after
    make a contiguous block of memory? */
    puc = (uint8_t *) pxIterator;
    if ((puc + pxIterator->xBlockSize) == (uint8_t *) pxBlockToInsert) {
        pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
        pxBlockToInsert = pxIterator;
    }

    /* Do the block being inserted, and the block it is being inserted before
    make a contiguous block of memory? */
    puc = (uint8_t *) pxBlockToInsert;
    if ((puc + pxBlockToInsert->xBlockSize) == (uint8_t *) pxIterator->pxNextFreeBlock) {
        if (pxIterator->pxNextFreeBlock != pxEnd) {
            /* Form one big block from the two blocks. */
            pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
            pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
        } else {
            pxBlockToInsert->pxNextFreeBlock = pxEnd;
        }
    } else {
        pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
    }

    /* If the block being inserted plugged a gab, so was merged with the block
    before and the block after, then it's pxNextFreeBlock pointer will have
    already been set, and should not be set here as that would make it point
    to itself. */
    if (pxIterator != pxBlockToInsert) {
        pxIterator->pxNextFreeBlock = pxBlockToInsert;
    }
}
