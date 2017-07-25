/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __usb_h__
#define __usb_h__


#include <stdint.h>
#include "proto.h"


#ifdef __cplusplus
extern "C" {
#endif


void usb_init(const char* serial_number);

void endp1_tx_msg(wk_msg_header* msg);

volatile uint8_t* endp2_get_rx_buffer();
int16_t endp2_get_rx_size();
void endp2_consume_rx_buffer();


#ifdef __cplusplus
}
#endif

#endif
