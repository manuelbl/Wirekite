/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __delay_h__
#define __delay_h__

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


typedef void (*delay_callback_t)(uint32_t);

void delay_init();
void delay_reset();
void delay_wait(uint32_t us, delay_callback_t callback, uint32_t param);

#ifdef __cplusplus
}
#endif

#endif
