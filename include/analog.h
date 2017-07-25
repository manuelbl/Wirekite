/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __analog_h__
#define __analog_h__

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


#define ANALOG_PIN_A0   0
#define ANALOG_PIN_A1   1
#define ANALOG_PIN_A2   2
#define ANALOG_PIN_A3   3
#define ANALOG_PIN_A4   4
#define ANALOG_PIN_A5   5
#define ANALOG_PIN_A6   6
#define ANALOG_PIN_A7   7
#define ANALOG_PIN_A8   8
#define ANALOG_PIN_A9   9
#define ANALOG_PIN_A10  10
#define ANALOG_PIN_A11  11
#define ANALOG_PIN_A12  12
#define ANALOG_PIN_VREF 128
#define ANALOG_PIN_TEMP 129
#define ANALOG_PIN_VREFL 130
#define ANALOG_PIN_BAND_GAP 131

#define ANALOG_PIN_ERROR 0xff
#define ANALOG_PIN_NONE  0xfe
#define ANALOG_PIN_CALIB_COMPLETE  0xfd


typedef uint8_t analog_pin;

void analog_init();
analog_pin analog_pin_init(uint8_t pin_idx, uint16_t attributes, uint16_t trigger_ms);
void analog_pin_release(analog_pin pin);
void analog_request_conversion(analog_pin pin);

analog_pin analog_get_completed_pin(int16_t* value);
void analog_reset();
void analog_timer_tick();


#ifdef __cplusplus
}
#endif

#endif
