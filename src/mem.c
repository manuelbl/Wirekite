/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

/* Simple memory management
 *
 * The entire heap as initially added to the freelist.
 * The heap is never extended.
 *
 * Allocation requests search for the smallest chunk in the freelist
 * that satisfies the allocation required.
 *
 * In the allocated chunk, the size is stored at the beginning and a
 * pointer to the address after the size is returned.
 *
 * If a chunk is in the freelist, the first 8 bytes contain the
 * chunk_t struct, that stores the size and provides the link to
 * the next element in the linked list.
 *
 * When a chunk is deallocated, it is inserted into the freelist, which
 * is kept is ascending order according to the chunk address. If possible,
 * the deallocated chunk is merged with the previous and the next chunk.
 */

#include <string.h>
#include <unistd.h>
#include "kinetis.h"
#include "mem.h"


typedef struct chunk_t {
    uint32_t size;
    struct chunk_t* next;
} chunk_t;

static chunk_t freelist;

#define NO_SIZE 0xffffffff


static uint32_t get_max_mem();


void mm_init(void* heap, uint32_t heap_len)
{
    if (heap == NULL && heap_len == 0) {
        heap_len = (get_max_mem() - 64) & 0xfffffff8;
        heap = sbrk(heap_len);
    }

    chunk_t* chunk = (chunk_t*)heap;
    chunk->size = heap_len;
    chunk->next = NULL;
    freelist.next = chunk;
}

void* mm_alloc(uint32_t size)
{
    __disable_irq();

    // round up to allocation size and add 4 bytes to store size
    uint32_t required_size = ((size + 3) & 0xfffffffc) + 4;
    
    // search for smallest chunk in the freelist
    // that is large enough to fit the block
    uint32_t smallest_size = NO_SIZE;
    chunk_t* smallest_size_prev = NULL;
    chunk_t* prev = &freelist;
    chunk_t* curr = freelist.next;
    
    while (curr) {
        uint32_t s = curr->size;
        if (s >= required_size && s < smallest_size) {
            smallest_size = s;
            smallest_size_prev = prev;
        }
        prev = curr;
        curr = curr->next;
    }
    
    if (smallest_size == NO_SIZE) {
        __enable_irq();
        return NULL; // no sufficiently large chunk left
    }
    
    uint32_t* p;
    chunk_t* chunk = smallest_size_prev->next;
    if (smallest_size < required_size + 12) {
        // use entire chunk; remove it from freelist
        smallest_size_prev->next = chunk->next;
        p = (uint32_t*)chunk;
        // no need to save the size;
        // it's already at the start of the chunk
    } else {
        // split chunk
        p = (uint32_t*)(((uint8_t*)chunk) + chunk->size - required_size);
        chunk->size -= required_size;
        *p = required_size;
    }

    __enable_irq();
    memset(p + 1, 0, required_size - 4);
    return p + 1;
}


void mm_free(void* ptr)
{
    __disable_irq();

    chunk_t* chunk = (chunk_t*)(((uint32_t*)ptr) - 1);
    uint32_t size = chunk->size;
    
    // search for insertion location in sorted linked list
    chunk_t* curr;
    for (curr = &freelist; curr->next != NULL && curr->next < chunk; curr = curr->next);
    
    if ((uint8_t*)chunk + size == (uint8_t*)curr->next) {
        // merge with next chunk
        size += curr->next->size;
        chunk->size = size;
        chunk->next = curr->next->next;
    } else {
        // link to next chunk
        chunk->next = curr->next;
    }
    
    if ((uint8_t*)curr + curr->size == (uint8_t*)chunk) {
        // merge with previous chunk
        curr->size += size;
        curr->next = chunk->next;
    } else {
        // link previous with deallocated
        curr->next = chunk;
    }

    __enable_irq();
}



extern char *__brkval;

// Copy from kinetis.c (about line 1136)
#ifndef STACK_MARGIN
#if defined(__MKL26Z64__)
#define STACK_MARGIN  512
#elif defined(__MK20DX128__)
#define STACK_MARGIN  1024
#elif defined(__MK20DX256__)
#define STACK_MARGIN  4096
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define STACK_MARGIN  8192
#endif
#endif

//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wunused-parameter"

uint32_t get_max_mem()
{
    char* stack;

    __asm__ volatile("mov %0, sp" : "=r" (stack) ::);

    return stack - STACK_MARGIN - __brkval;
}
