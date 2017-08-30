/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __uart_h__
#define __uart_h__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*uart_event_func_t)(void);

void uart0_init(int32_t baudrate);
void uart0_end();
void uart0_write(const char* ptr, int32_t len);
void uart0_println(const char* ptr);
void uart0_flush();
int32_t uart0_avail();
int32_t uart0_read(char* ptr, int32_t len);


#ifdef __cplusplus
}
#endif

#endif
