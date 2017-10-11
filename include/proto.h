/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __proto_h__
#define __proto_h__


#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


#define WK_MSG_TYPE_CONFIG_REQUEST 1
#define WK_MSG_TYPE_CONFIG_RESPONSE 2
#define WK_MSG_TYPE_PORT_REQUEST 3
#define WK_MSG_TYPE_PORT_EVENT 4

#define WK_CFG_ACTION_CONFIG_PORT 1
#define WK_CFG_ACTION_RELEASE 2
#define WK_CFG_ACTION_RESET 3
#define WK_CFG_ACTION_CONFIG_MODULE 4
#define WK_CFG_ACTION_QUERY 5

#define WK_PORT_ACTION_SET_VALUE 1
#define WK_PORT_ACTION_GET_VALUE 2
#define WK_PORT_ACTION_TX_DATA 3
#define WK_PORT_ACTION_RX_DATA 4
#define WK_PORT_ACTION_TX_N_RX_DATA 5

#define WK_CFG_PORT_TYPE_DIGI_PIN 1
#define WK_CFG_PORT_TYPE_ANALOG_IN 2
#define WK_CFG_PORT_TYPE_PWM 3
#define WK_CFG_PORT_TYPE_I2C 4
#define WK_CFG_PORT_TYPE_SPI 5

#define WK_CFG_QUERY_MEM_AVAIL 1
#define WK_CFG_QUERY_MEM_MAX_BLOCK 2
#define WK_CFG_QUERY_MEM_MCU 3
#define WK_CFG_QUERY_VERSION 4

#define WK_CFG_MCU_TEENSY_LC 1
#define WK_CFG_MCU_TEENSY_3_2 2

#define WK_CFG_MODULE_PWM_TIMER 1
#define WK_CFG_MODULE_PWM_CHANNEL 2

#define WK_RESULT_OK 0
#define WK_RESULT_INV_DATA 1

#define WK_EVENT_DODO 0
#define WK_EVENT_SINGLE_SAMPLE 1
#define WK_EVENT_TX_COMPLETE 2
#define WK_EVENT_DATA_RECV 3


typedef struct {
  uint16_t message_size;
  uint8_t message_type;
  uint8_t reserved0;
  uint16_t port_id;
  uint16_t request_id;
} wk_msg_header;


typedef struct {
  wk_msg_header header;
  uint8_t action;
  uint8_t port_type;
  uint16_t pin_config;
  uint32_t value1;
  uint16_t port_attributes1;
  uint16_t port_attributes2;
} wk_config_request;


typedef struct {
  wk_msg_header header;
  uint16_t result;
  uint16_t optional1;
  uint32_t value1;
} wk_config_response;


typedef struct {
  wk_msg_header header;
  uint8_t action;
  uint8_t action_attribute1;
  uint16_t action_attribute2;
  uint32_t value1;
  uint8_t data[4]; // variable length; can be 0 bytes
} wk_port_request;

#define WK_PORT_REQUEST_ALLOC_SIZE(data_len) ((uint16_t)(sizeof(wk_port_request) - 4 + data_len))
#define WK_PORT_REQUEST_DATA_LEN(request) ((uint16_t)((request)->header.message_size - sizeof(wk_port_request) + 4))


typedef struct {
  wk_msg_header header;
  uint8_t event;
  uint8_t event_attribute1;
  uint16_t event_attribute2;
  uint32_t value1;
  uint8_t data[4]; // variable length; can be 0 bytes
} wk_port_event;

#define WK_PORT_EVENT_ALLOC_SIZE(data_len) ((uint16_t)(sizeof(wk_port_event) - 4 + data_len))
#define WK_PORT_EVENT_DATA_LEN(event) ((uint16_t)((event)->header.message_size - sizeof(wk_port_event) + 4))


#ifdef __cplusplus
}
#endif

#endif
