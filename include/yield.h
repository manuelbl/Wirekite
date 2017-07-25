/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __yield_h__
#define __yield_h__

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


typedef void (*yield_func_t)(void);

void yield(void);
void yield_add_func(yield_func_t f);
void yield_remove_func(yield_func_t f);


#ifdef __cplusplus
}
#endif

#endif
