/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __timer_h__
#define __timer_h__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


// the systick interrupt will increment this at 1 kHz rate
extern volatile uint32_t systick_millis_count;

uint32_t micros(void);
static inline uint32_t millis(void) __attribute__((always_inline, unused));
static inline uint32_t millis(void) { return systick_millis_count; }
void delay(uint32_t ms);



#ifdef __cplusplus
}
#endif

#endif
