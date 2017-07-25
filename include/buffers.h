/**
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

#define TX_BUFFER_SIZE 64
#define TX_NUM_BUFFERS 4


void buffers_init();

void* buffers_get_buf();
void buffers_free_buf(void* buffer);


#ifdef __cplusplus
}
#endif

#endif
