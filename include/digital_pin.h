/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __digital_pin_h__
#define __digital_pin_h__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DIGI_PIN_0   0
#define DIGI_PIN_1   1
#define DIGI_PIN_2   2
#define DIGI_PIN_3   3
#define DIGI_PIN_4   4
#define DIGI_PIN_5   5
#define DIGI_PIN_6   6
#define DIGI_PIN_7   7
#define DIGI_PIN_8   8
#define DIGI_PIN_9   9
#define DIGI_PIN_10 10
#define DIGI_PIN_11 11
#define DIGI_PIN_12 12
#define DIGI_PIN_13 13
#define DIGI_PIN_14 14
#define DIGI_PIN_15 15
#define DIGI_PIN_16 16
#define DIGI_PIN_17 17
#define DIGI_PIN_18 18
#define DIGI_PIN_19 19
#define DIGI_PIN_20 20
#define DIGI_PIN_21 21
#define DIGI_PIN_22 22
#define DIGI_PIN_23 23
#define DIGI_PIN_24 24
#define DIGI_PIN_25 25
#define DIGI_PIN_26 26
#define DIGI_PIN_LED DIGI_PIN_13

#define DIGI_PIN_INPUT 0
#define DIGI_PIN_OUTPUT 1

#define DIGI_PIN_DEFAULT 0
#define DIGI_PIN_IN_PULLUP 4
#define DIGI_PIN_IN_PULLDOWN 8
#define DIGI_PIN_IN_TRIGGER_RAISING 16
#define DIGI_PIN_IN_TRIGGER_FALLING 32
#define DIGI_PIN_OUT_LOW_CURRENT 4
#define DIGI_PIN_OUT_HIGH_CURRENT 8


#define DIGI_PIN_ON 1
#define DIGI_PIN_OFF 0


typedef uint8_t digital_pin;

digital_pin digital_pin_init(uint8_t pin_idx, uint8_t direction, uint16_t attributes);
void digital_pin_release(digital_pin pin);
void digital_pin_set_output(digital_pin pin, uint8_t value);
uint8_t digital_pin_get_input(digital_pin pin);
digital_pin digital_pin_get_interrupt_pin();
void digital_pin_reset();

#ifdef __cplusplus
}
#endif

#endif
