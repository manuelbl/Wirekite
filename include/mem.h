/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __mem_pin_h__
#define __mem_pin_h__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


// Initialize the memory management with the specified heap block
void mm_init(void* heap, uint32_t heap_len);
// Allocate a memory block
void* mm_alloc(uint32_t size);
// Free a memory block
void mm_free(void* ptr);

 
#ifdef __cplusplus
}
#endif

#endif
