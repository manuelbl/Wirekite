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

#define WK_PORT_ACTION_SET_VALUE 1
#define WK_PORT_ACTION_GET_VALUE 2

#define WK_CFG_PORT_TYPE_DIGI_PIN 1
#define WK_CFG_PORT_TYPE_ANALOG_IN 2
#define WK_CFG_PORT_TYPE_PWM 3

#define WK_CFG_MODULE_PWM_TIMER 1
#define WK_CFG_MODULE_PWM_CHANNEL 2

#define WK_RESULT_OK 0
#define WK_RESULT_INV_DATA 1

#define WK_EVENT_DODO 0
#define WK_EVENT_SINGLE_SAMPLE 1


typedef struct {
  uint16_t message_size;
  uint8_t message_type;
  uint8_t reserved0;
} wk_msg_header;


typedef struct {
  wk_msg_header header;
  uint8_t action;
  uint8_t port_type;
  uint16_t port_id;
  uint16_t request_id;
  uint16_t pin_config;
  uint32_t value1;
  uint16_t port_attributes;
} wk_config_request;


typedef struct {
  wk_msg_header header;
  uint16_t result;
  uint16_t port_id;
  uint16_t request_id;
  uint16_t optional1;
} wk_config_response;


typedef struct {
  wk_msg_header header;
  uint16_t port_id;
  uint8_t action;
  uint8_t action_attribute1;
  uint16_t action_attribute2;
  uint16_t request_id;
  uint8_t data[4]; // variable length, at leat 1 bytes
} wk_port_request;


typedef struct {
  wk_msg_header header;
  uint16_t port_id;
  uint8_t event;
  uint8_t event_attribute1;
  uint16_t request_id;
  uint8_t data[4]; // variable length, at least 1 bytes
} wk_port_event;


#ifdef __cplusplus
}
#endif

#endif
