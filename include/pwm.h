/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __pwm_h__
#define __pwm_h__

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#if defined(__MKL26Z64__)

// Teensy LC
#define PWM_PIN_3   0
#define PWM_PIN_4   1
#define PWM_PIN_6   2
#define PWM_PIN_9   3
#define PWM_PIN_10  4
#define PWM_PIN_16  5
#define PWM_PIN_17  6
#define PWM_PIN_20  7
#define PWM_PIN_22  8
#define PWM_PIN_23  9

#elif defined(__MK20DX256__)

#define PWM_PIN_3   0
#define PWM_PIN_4   1
#define PWM_PIN_5   2
#define PWM_PIN_6   3
#define PWM_PIN_9   4
#define PWM_PIN_10  5
#define PWM_PIN_20  6
#define PWM_PIN_21  7
#define PWM_PIN_22  8
#define PWM_PIN_23  9
#define PWM_PIN_25  10
#define PWM_PIN_32  11


#endif

#define PWM_PIN_ERROR 0xff


#define PWM_CHAN_ATTR_HIGH_PULSE 0
#define PWM_CHAN_ATTR_LOW_PULSE 2

#define PWM_TIMER_ATTR_EDGE 0
#define PWM_TIMER_ATTR_CENTER 1


typedef uint8_t pwm_pin;


void pwm_init();
void pwm_timer_config(uint8_t timer, uint32_t frequency, uint16_t attributes);
void pwm_channel_config(uint8_t timer, uint8_t channel, uint16_t attributes);
pwm_pin pwm_pin_init(uint8_t digi_pin);
void pwm_pin_release(pwm_pin port_id);
void pwm_pin_set_value(pwm_pin port_id, int16_t value);
void pwm_reset();


#ifdef __cplusplus
}
#endif

#endif
