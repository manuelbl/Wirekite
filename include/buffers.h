/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __buffers_h__
#define __buffers_h__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Poor man's memory management
 * All allocated buffers are 64 byte.
 */

#define TX_BUFFER_SIZE 64
#define TX_NUM_BUFFERS 60

// Initialize memory management
void buffers_init();

// Allocate buffer
void* buffers_alloc_buf();
// Free buffer
void buffers_free_buf(void* buffer);


#ifdef __cplusplus
}
#endif

#endif
