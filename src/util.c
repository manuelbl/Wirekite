/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include "util.h"


uint16_t frequency_lookup(const freq_div_t* freq_table, int freq_table_cnt, uint16_t target_divider)
{
    // binary search
    int lower = 0;
    int upper = freq_table_cnt - 1;
    while (lower < upper) {
        int mid = (upper + lower) / 2;
        if (freq_table[mid].divider < target_divider)
            lower = mid + 1;
        else
            upper = mid;
    }
    // result:
    //    forall i  < upper: freq_divs[i].divider <  target_div
    //    forall i => upper: freq_divs[i].divider >= target_div
    
    return upper;
}


#ifdef _DEBUG 

static char HEX_DIGITS[] = {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};

void bytes_to_hex(char* dst, const uint8_t* data, uint16_t size)
{
    while (size > 0) {
        size--;
        uint8_t ch = data[size];
        *dst = HEX_DIGITS[ch >> 4];
        dst++;
        *dst = HEX_DIGITS[ch & 0x0f];
        dst++;
    }
}

#endif

