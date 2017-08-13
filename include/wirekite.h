/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __wirekite_h__
#define __wirekite_h__


#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


#define PORT_GROUP_MASK 0xff00
#define PORT_GROUP_DETAIL_MASK 0x00ff
#define PORT_GROUP_DIGI_PIN 0x0100
#define PORT_GROUP_ANALOG_IN 0x0200
#define PORT_GROUP_PWM 0x0300
#define PORT_GROUP_I2C 0x0400


void wk_check_usb_rx();
void wk_send_port_event(uint16_t port_id, uint8_t evt, uint16_t request_id, uint8_t* data, uint16_t data_len);
void wk_send_port_event_2(uint16_t port_id, uint8_t evt, uint16_t request_id, uint8_t* data, uint16_t data_len, uint8_t attr1, uint16_t attr2);


#ifdef __cplusplus
}
#endif

#endif
