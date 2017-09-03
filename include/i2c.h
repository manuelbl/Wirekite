/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __i2c_h__
#define __i2c_h__

#include "proto.h"


#ifdef __cplusplus
extern "C" {
#endif


#define I2C_PINS_16_17  0
#define I2C_PINS_19_18  1
#if defined(__MKL26Z64__)
#define I2C_PINS_22_23  2
#elif defined(__MK20DX256__)
#define I2C_PINS_29_30  2
#endif
#define I2C_PINS_MAX  2


#define I2C_PORT_ERROR 0xff


#define I2C_STATUS_OK 0
#define I2C_STATUS_TIMEOUT 1
#define I2C_STATUS_ARB_LOST 2
#define I2C_STATUS_ADDR_NAK 3
#define I2C_STATUS_DATA_NAK 4
#define I2C_STATUS_OUT_OF_MEMORY 5
#define I2C_STATUS_BUS_BUSY 6
#define I2C_STATUS_UNKNOWN 7


typedef uint8_t i2c_port;

void i2c_init();
void i2c_reset();

i2c_port i2c_master_init(uint8_t pins, uint16_t attributes, uint32_t frequency);
void i2c_port_release(i2c_port port);
void i2c_port_reset(i2c_port port);

// Always takes ownership of the request
void i2c_master_start_send(wk_port_request* request);
// Does not take ownership of the request
void i2c_master_start_recv(wk_port_request* request);


#ifdef __cplusplus
}
#endif

#endif
