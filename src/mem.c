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
#include "debug.h"


typedef struct __attribute__((packed, aligned(4))) chunk_t {
    uint32_t size;
    struct chunk_t* next;
} chunk_t;

static chunk_t freelist;
static void* heap_start;

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
    freelist.size = heap_len;
    freelist.next = chunk;
    heap_start = heap;
}


void* mm_alloc(uint32_t size)
{
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
        DEBUG_OUT("Out of memory");
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

    memset(p + 1, 0, required_size - 4);
    return p + 1;
}


void mm_free(void* ptr)
{
    if (ptr == NULL)
        return;
        
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
}


uint32_t mm_avail()
{
    uint32_t size = 0;
    chunk_t* curr = freelist.next;
    
    while (curr) {
        size += curr->size;
        curr = curr->next;
    }
    
    return size;
}


uint32_t mm_max_avail_block()
{
    uint32_t max_avail = 0;
    chunk_t* curr = freelist.next;
    
    while (curr) {
        uint32_t s = curr->size;
        if (s > max_avail)
            max_avail = s;
        curr = curr->next;
    }
    
    return max_avail;
}


#ifdef _DEBUG

static int mem_in_use;

static int check_blocks_in_use(uint8_t* start, uint8_t* end)
{
    // iterate chunks in use
    uint8_t* p = start;
    while (p < end) {
        uint32_t size = *(uint32_t*)(p);
        if (p + size > end)
            return 5;
        mem_in_use += size;
        p += size;
    }

    if (p != end)
        return 6;
    
    return 0;
}

int mm_check_integrity()
{
    mem_in_use = 0;
    int mem_free = 0;

    if (freelist.size < 0x800 || freelist.size > 0x100000)
        return 1;

    chunk_t* curr = freelist.next;
    if (curr != heap_start) {
        int r = check_blocks_in_use((uint8_t*)heap_start, (uint8_t*)curr);
        if (r != 0)
            return r;
    }

    while (curr->next) {
        uint8_t* curr_c = (uint8_t*)curr;
        uint8_t* next_c = (uint8_t*)curr->next;

        uint32_t s = curr->size;
        if (s > freelist.size)
            return 2;

        mem_free += s;

        // chuncks must be in ascending order
        if (curr_c >= next_c)
            return 3;
        // chunck must not extend into next block
        // and leave a gap of more than 4 bytes
        if (curr_c + s + 4 >= next_c)
            return 4;
        
        int r = check_blocks_in_use(curr_c + s, next_c);
        if (r != 0)
            return r;
        
        curr = curr->next;
    }

    mem_free += curr->size;

    if ((uint8_t*)curr < ((uint8_t*)heap_start) + freelist.size) {
        int r = check_blocks_in_use((uint8_t*)curr, ((uint8_t*)heap_start) + freelist.size);
        if (r != 0)
            return r;
    }

    return 0; // everyting seems ok
}

void dump_used_blocks(uint8_t* start, uint8_t* end)
{
    while (start < end) {
        uint32_t size = *(uint32_t*)start;
        char msg[] = "U .... .. .... ....";
        bytes_to_hex(msg + 2, start + 4, 2);
        bytes_to_hex(msg + 7, start + 6, 1);
        bytes_to_hex(msg + 10, start + 8, 2);
        bytes_to_hex(msg + 15, start + 10, 2);
        DEBUG_OUT(msg);
        start += size;
    }
}

void dump_free_block(chunk_t* chunk)
{
    char msg[] = "F ....";
    bytes_to_hex(msg + 2, (uint8_t*)&chunk->size, 2);
    DEBUG_OUT(msg);
}

void mm_dump_used()
{
    chunk_t* curr = freelist.next;
    dump_used_blocks((uint8_t*)heap_start, (uint8_t*)curr);

    while (curr->next) {
        dump_free_block(curr);
        dump_used_blocks(((uint8_t*)curr) + curr->size, (uint8_t*)curr->next);        
        curr = curr->next;
    }

    dump_free_block(curr);
    dump_used_blocks(((uint8_t*)curr) + curr->size, ((uint8_t*)heap_start) + freelist.size);
}

#endif


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
