/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

 #ifndef __spi_h__
 #define __spi_h__
 
 #include "proto.h"
 
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 #define SPI_CONFIG_MSB_FIRST 0
 #define SPI_CONFIG_LSB_FIRST 1
 #define SPI_CONFIG_8_BIT 0
 #define SPI_CONFIG_16_BIT 2
 #define SPI_CONFIG_MODE0 0
 #define SPI_CONFIG_MODE1 4
 #define SPI_CONFIG_MODE2 8
 #define SPI_CONFIG_MODE3 12

 #define SPI_CONFIG_SB_MASK 1
 #define SPI_CONFIG_BITS_MASK 2
 #define SPI_CONFIG_MODE_MASK 12

 
 #define I2C_PINS_16_17  0
 #define I2C_PINS_19_18  1
 #if defined(__MKL26Z64__)
 #define I2C_PINS_22_23  2
 #elif defined(__MK20DX256__)
 #define I2C_PINS_29_30  2
 #endif
 #define I2C_PINS_MAX  2
 
 
 #define SPI_PORT_ERROR 0xff
 
 
 #define SPI_STATUS_OK 0
 #define SPI_STATUS_TIMEOUT 1
 #define SPI_STATUS_UNKNOWN 7
 
 
 typedef uint8_t spi_port;
 
 void spi_init();
 void spi_reset();
 
 spi_port spi_master_init(uint16_t sck_mosi, uint16_t miso_cs, uint16_t attributes, uint32_t frequency);
 void spi_port_release(spi_port port);

 // Always takes ownership of the request
 void spi_master_start_send(wk_port_request* request);
 // Does not take ownership of the request
 void spi_master_start_recv(wk_port_request* request);
 
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif
 