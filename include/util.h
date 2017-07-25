/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __util_h__
#define __util_h__


#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


void bytes_to_hex(char* dst, const uint8_t* data, uint16_t size);


#ifdef __cplusplus
}
#endif

#endif
