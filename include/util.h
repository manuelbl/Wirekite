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

typedef struct {
    uint16_t f_div;
    uint16_t divider;
} freq_div_t;

uint16_t frequency_lookup(const freq_div_t* freq_table, int freq_table_cnt, uint16_t target_divider);


#ifdef _DEBUG

void bytes_to_hex(char* dst, const uint8_t* data, uint16_t size);

#endif


#ifdef __cplusplus
}
#endif

#endif
