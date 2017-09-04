/*
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


#define DIGI_PIN_ERROR 0xff

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

digital_pin digital_pin_init(uint8_t pin_idx, uint8_t direction, uint16_t attributes, uint8_t initial_value);
void digital_pin_release(digital_pin pin);
void digital_pin_set_output(digital_pin pin, uint8_t value);
uint8_t digital_pin_get_input(digital_pin pin);
digital_pin digital_pin_get_interrupt_pin();
void digital_pin_reset();

#ifdef __cplusplus
}
#endif

#endif
