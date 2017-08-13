/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include <string.h>
#include "kinetis.h"

#include "buffers.h"


typedef enum {
    BUF_FREE = 0,
    BUF_IN_USE = 1
} buffer_state;

static uint8_t buffers[TX_NUM_BUFFERS][TX_BUFFER_SIZE] __attribute__ ((aligned (4)));
static volatile buffer_state buffer_states[TX_NUM_BUFFERS];


void buffers_init()
{
    // nothing to do
}

void* buffers_alloc_buf()
{
    uint8_t* buf = NULL;

    // Search for next free buffer
    __disable_irq();
    for (int i = 0; i < TX_NUM_BUFFERS; i++) {
        if (buffer_states[i] == BUF_FREE) {
            buffer_states[i] = BUF_IN_USE;
            buf = &buffers[i][0];
            break;
        }
    }
    __enable_irq();

    return buf;
}

void buffers_free_buf(void* buffer)
{
    // reconstruct index from buffer address
    int buffer_index = ((uint8_t*)buffer - buffers[0]) / TX_BUFFER_SIZE;

    __disable_irq();
    buffer_states[buffer_index] = BUF_FREE;
    __enable_irq();
}
