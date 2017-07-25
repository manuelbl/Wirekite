/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include "util.h"


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
