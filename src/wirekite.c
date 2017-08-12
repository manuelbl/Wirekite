/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include <string.h>

#include "wirekite.h"
#include "proto.h"
#include "usb.h"
#include "yield.h"
#include "digital_pin.h"
#include "analog.h"
#include "buffers.h"
#include "kinetis.h"
#include "pwm.h"


#define PORT_GROUP_MASK 0xff00
#define PORT_GROUP_DETAIL_MASK 0x00ff
#define PORT_GROUP_DIGI_PIN 0x0100
#define PORT_GROUP_ANALOG_IN 0x0200
#define PORT_GROUP_PWM 0x0300


uint8_t wk_reset_flag = 0;

uint16_t remainder_size;
uint8_t remainder_buf[512];


static void handle_config_request(wk_config_request* request);
static void handle_port_request(wk_port_request* request);
static void handle_message(uint8_t* msg);
static void send_config_response(uint16_t result, uint16_t port_id, uint16_t request_id, uint16_t optional1);
static void send_port_event(uint16_t port_id, uint8_t event, uint16_t request_id, uint8_t* data, uint16_t data_len);
static void wk_reset();


void check_usb_rx()
{
    // check for global reset
    if (wk_reset_flag != 0) {
        wk_reset_flag = 0;
        wk_reset();
    }

    int16_t rx_size = endp2_get_rx_size();
    if (rx_size <= 0)
        return;
    
    uint8_t* rx_buf = (uint8_t*) endp2_get_rx_buffer();

    if (remainder_size > 0) {
        wk_msg_header* hdr = (wk_msg_header*) remainder_buf;
        uint16_t msg_size = hdr->message_size;
        if (rx_size + remainder_size >= msg_size) {
            uint16_t len = msg_size - remainder_size;
            memcpy(remainder_buf + remainder_size, rx_buf, len);
            handle_message(remainder_buf);
            rx_size -= len;
            rx_buf += len;
            remainder_size = 0;
        }
    }

    while (rx_size >= 2) {

        wk_msg_header* hdr = (wk_msg_header*) rx_buf;
        uint16_t msg_size = hdr->message_size;
        if (rx_size < msg_size)
            break;

        if (msg_size < 8) {
            // oops?!
            remainder_size = 0;
            endp2_consume_rx_buffer();
            return;
        }

        handle_message(rx_buf);

        rx_buf += msg_size;
        rx_size -= msg_size;
    }

    if (rx_size > 0) {
        memcpy(remainder_buf + remainder_size, rx_buf, rx_size);
        remainder_size += rx_size;
    }

    endp2_consume_rx_buffer();
}


void handle_message(uint8_t* msg)
{
    wk_msg_header* hdr = (wk_msg_header*)msg;
    if (hdr->message_type == WK_MSG_TYPE_CONFIG_REQUEST) {
        handle_config_request((wk_config_request*)msg);
    } else if (hdr->message_type == WK_MSG_TYPE_PORT_REQUEST) {
        handle_port_request((wk_port_request*)msg);
    }
}


void handle_config_request(wk_config_request* request)
{
    if (request->header.message_size == sizeof(wk_config_request)) {
        if (request->action == WK_CFG_ACTION_CONFIG_PORT) {
            uint16_t port_id = 0;
            uint16_t optional1 = 0;
            if (request->port_type == WK_CFG_PORT_TYPE_DIGI_PIN) {
                digital_pin pin = digital_pin_init(request->pin_config, request->port_attributes & 0x01, request->port_attributes & 0xfe);
                port_id = PORT_GROUP_DIGI_PIN | pin;
                if ((request->port_attributes & 0x01) == DIGI_PIN_INPUT)
                    optional1 = digital_pin_get_input(pin);
            } else if (request->port_type == WK_CFG_PORT_TYPE_ANALOG_IN) {
                analog_pin pin = analog_pin_init(request->pin_config, request->port_attributes, request->value1);
                if (pin != ANALOG_PIN_ERROR)
                    port_id = PORT_GROUP_ANALOG_IN | pin;
            } else if (request->port_type == WK_CFG_PORT_TYPE_PWM) {
                pwm_pin pin = pwm_pin_init(request->pin_config);
                if (pin != PWM_PIN_ERROR)
                    port_id = PORT_GROUP_PWM | pin;
            }
            if (port_id != 0) {
                send_config_response(WK_RESULT_OK, port_id, request->request_id, optional1);
                return;
            }

        } else if (request->action == WK_CFG_ACTION_RELEASE) {
            if ((request->port_id & PORT_GROUP_MASK) == PORT_GROUP_DIGI_PIN) {
                digital_pin pin = request->port_id & PORT_GROUP_DETAIL_MASK;
                digital_pin_release(pin);
                send_config_response(WK_RESULT_OK, request->port_id, request->request_id, 0);
                return;
            } else if ((request->port_id & PORT_GROUP_MASK) == PORT_GROUP_ANALOG_IN) {
                analog_pin pin = request->port_id & PORT_GROUP_DETAIL_MASK;
                analog_pin_release(pin);
                send_config_response(WK_RESULT_OK, request->port_id, request->request_id, 0);
                return;
            } else if ((request->port_id & PORT_GROUP_MASK) == PORT_GROUP_PWM) {
                pwm_pin pin = request->port_id & PORT_GROUP_DETAIL_MASK;
                pwm_pin_release(pin);
                send_config_response(WK_RESULT_OK, request->port_id, request->request_id, 0);
                return;
            }

        } else if (request->action == WK_CFG_ACTION_RESET) {
            wk_reset();
            send_config_response(WK_RESULT_OK, request->port_id, request->request_id, 0);
            return;
        
        } else if (request->action == WK_CFG_ACTION_CONFIG_MODULE) {
            if (request->port_type == WK_CFG_MODULE_PWM_TIMER) {
                pwm_timer_config(request->pin_config, request->value1, request->port_attributes);
                send_config_response(WK_RESULT_OK, request->port_id, request->request_id, 0);
                return;
            
            } else if (request->port_type == WK_CFG_MODULE_PWM_CHANNEL) {
                pwm_channel_config(request->pin_config, request->value1, request->port_attributes);
                send_config_response(WK_RESULT_OK, request->port_id, request->request_id, 0);
                return;
            }
        }
    }    
    
    send_config_response(WK_RESULT_INV_DATA, request->port_id, request->request_id, 0);
}


void handle_port_request(wk_port_request* request)
{
    if (request->header.message_size >= sizeof(wk_port_request) - 3) {

        uint16_t port_group = request->port_id & PORT_GROUP_MASK;
        if (request->action == WK_PORT_ACTION_SET_VALUE) {
            if (port_group == PORT_GROUP_DIGI_PIN) {
                digital_pin_set_output(request->port_id & PORT_GROUP_DETAIL_MASK, request->data[0]);
                return;
            } else if (port_group == PORT_GROUP_PWM) {
                int16_t* p = (int16_t*)&request->data;
                pwm_pin_set_value(request->port_id & PORT_GROUP_DETAIL_MASK, *p);
                return;
            }
        } else if (request->action == WK_PORT_ACTION_GET_VALUE) {
            if (port_group == PORT_GROUP_DIGI_PIN) {
                uint8_t value = digital_pin_get_input(request->port_id & PORT_GROUP_DETAIL_MASK);
                send_port_event(request->port_id, WK_EVENT_SINGLE_SAMPLE, request->request_id, &value, 1);
                return;
            } else if (port_group == PORT_GROUP_ANALOG_IN) {
                analog_request_conversion(request->port_id & PORT_GROUP_DETAIL_MASK);
                return;
            }
        }
    }
}


void send_config_response(uint16_t result, uint16_t port_id, uint16_t request_id, uint16_t optional1)
{
    wk_config_response* response = (wk_config_response*) buffers_get_buf();
    if (response == NULL)
        return; // drop data

    response->header.message_size = sizeof(wk_config_response);
    response->header.message_type = WK_MSG_TYPE_CONFIG_RESPONSE;
    response->result = result;
    response->port_id = port_id;
    response->request_id = request_id;
    response->optional1 = optional1;
    
    endp1_tx_msg(&response->header);
}


void send_port_event(uint16_t port_id, uint8_t evt, uint16_t request_id, uint8_t* data, uint16_t data_len)
{
    wk_port_event* event = (wk_port_event*) buffers_get_buf();
    if (event == NULL)
        return; // drop data

    event->header.message_size = sizeof(wk_port_event) - 4 + data_len;
    event->header.message_type = WK_MSG_TYPE_PORT_EVENT;
    event->port_id = port_id;
    event->event = evt;
    event->event_attribute1 = 0;
    event->request_id = request_id;
    for (int i = 0; i < data_len; i++)
        event->data[i] = data[i];
    
    endp1_tx_msg(&event->header);
}

/*
 * Interrupt handler for digital input pins on port A
 */
void porta_isr()
{
    portcd_isr();
}


/*
 * Interrupt hander for digital input pins on port C and D
 */
void portcd_isr()
{
    while (1) {
        digital_pin pin = digital_pin_get_interrupt_pin();
        if (pin == 0xff)
            break;

        uint8_t value = digital_pin_get_input(pin);
        send_port_event(PORT_GROUP_DIGI_PIN | pin, WK_EVENT_SINGLE_SAMPLE, 0, &value, 1);
    }
}


/*
 * Interrupt handler for analog-to-digital conversions
 */
void adc0_isr()
{
    int16_t value;
    analog_pin pin = analog_get_completed_pin(&value);

    if (pin == ANALOG_PIN_NONE)
        return; // spurious interrupt; don't know why
    
    if (pin == ANALOG_PIN_CALIB_COMPLETE)
        return;

    send_port_event(PORT_GROUP_ANALOG_IN | pin, WK_EVENT_SINGLE_SAMPLE, 0, (uint8_t*)&value, 2);
}


/*
 * Reset all configured ports
 */
void wk_reset()
{
    digital_pin_reset();
    analog_reset();
    pwm_reset();
}


extern volatile uint32_t systick_millis_count;

void systick_isr(void)
{
	systick_millis_count++;
    analog_timer_tick();
}
