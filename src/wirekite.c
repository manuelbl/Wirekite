/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include <string.h>
#include "kinetis.h"

#include "debug.h"
#include "wirekite.h"
#include "analog.h"
#include "mem.h"
#include "digital_pin.h"
#include "i2c.h"
#include "spi.h"
#include "proto.h"
#include "pwm.h"
#include "usb.h"


uint8_t wk_reset_flag = 0;

static uint16_t partial_size;
static uint16_t msg_size;
static wk_msg_header* partial_msg;


static void handle_config_request(wk_config_request* request);
static void handle_port_request(wk_port_request* request, uint8_t* deallocate_msg);
static void handle_message(wk_msg_header* msg, uint8_t take_ownership);
static void send_config_response(uint16_t result, uint16_t port_id, uint16_t request_id, uint16_t optional1, uint32_t value1);
static void wk_reset();
static void take_ownership_or_copy(wk_port_request** request, uint8_t* deallocate_msg);


void wk_check_usb_rx()
{
    // check for global reset
    if (wk_reset_flag != 0) {
        wk_reset_flag = 0;
        __disable_irq();        
        wk_reset();
        __enable_irq();
    }

    int16_t rx_size = endp2_get_rx_size();
    if (rx_size <= 0)
        return;
    
    __disable_irq();
    uint8_t* rx_buf = (uint8_t*) endp2_get_rx_buffer();

    if (partial_size > 0) {
        // there is a partial message from the last USB packet

        if (partial_size == 1) {
            // super special case: only half of the first word
            // was transmitted
            msg_size += ((uint16_t)rx_buf[0]) << 8;
            partial_msg = (wk_msg_header*)mm_alloc(msg_size);
            if (partial_msg != NULL)
                partial_msg->message_size = msg_size;
            else
                DEBUG_OUT("Insufficient memory for request");
            DEBUG_OUT("Super special case");
        }

        uint16_t len = rx_size;
        if (partial_size + len > msg_size)
            len = msg_size - partial_size;
        
        // append to partial message (buffer is big enough)
        if (partial_msg != NULL)
            memcpy(((uint8_t*)partial_msg) + partial_size, rx_buf, len);
        rx_buf += len;
        rx_size -= len;
        partial_size += len;

        // if message is complete handle it
        if (partial_size == msg_size) {
            if (partial_msg != NULL)
                handle_message(partial_msg, 1);
            partial_size = 0;
            msg_size = 0;
            partial_msg = NULL;
        }
    }

    while (rx_size >= 2) {

        wk_msg_header* hdr = (wk_msg_header*) rx_buf;
        uint16_t size = hdr->message_size;
        if (rx_size < size)
            break;

        if (size < 8) {
            // oops?!
            DEBUG_OUT("Ooops");
            endp2_consume_rx_buffer();
            __enable_irq();
            return;
        }

        handle_message(hdr, 0);

        rx_buf += size;
        rx_size -= size;
    }

    if (rx_size > 0) {
        // a partial message remains
        
        if (rx_size == 1) {
            // super special case: only 1 byte was transmitted;
            // we don't know the size of the message
            partial_size = 1;
            msg_size = rx_buf[0];

        } else {
            // allocate buffer
            wk_msg_header* hdr = (wk_msg_header*) rx_buf;
            msg_size = hdr->message_size;
            if (partial_msg != NULL)
                DEBUG_OUT("Lost it");
            partial_msg = (wk_msg_header*)mm_alloc(msg_size);
            partial_size = rx_size;
            if (partial_msg != NULL)
                memcpy(partial_msg, rx_buf, rx_size);
            else
                DEBUG_OUT("Insufficient memory for request");
        }
    }

    endp2_consume_rx_buffer();
    __enable_irq();
}


void handle_message(wk_msg_header* msg, uint8_t take_ownership)
{
    uint8_t deallocate_msg = take_ownership;
    if (msg->message_type == WK_MSG_TYPE_CONFIG_REQUEST) {
        handle_config_request((wk_config_request*)msg);
    } else if (msg->message_type == WK_MSG_TYPE_PORT_REQUEST) {
        handle_port_request((wk_port_request*)msg, &deallocate_msg);
    } else {
        DEBUG_OUT("Invalid msg");
    }
    if (deallocate_msg)
        mm_free(msg);
}


void handle_config_request(wk_config_request* request)
{
    uint16_t result = WK_RESULT_OK;

    if (request->header.message_size == sizeof(wk_config_request)) {

        if (request->action == WK_CFG_ACTION_CONFIG_PORT) {
            uint16_t port_id = 0;
            uint16_t optional1 = 0;
            if (request->port_type == WK_CFG_PORT_TYPE_DIGI_PIN) {
                digital_pin pin = digital_pin_init(request->pin_config, request->port_attributes1 & 0x01,
                        request->port_attributes1 & 0xfe, (uint8_t)request->value1);
                port_id = PORT_GROUP_DIGI_PIN | pin;
                if ((request->port_attributes1 & 0x01) == DIGI_PIN_INPUT)
                    optional1 = digital_pin_get_input(pin);
            } else if (request->port_type == WK_CFG_PORT_TYPE_ANALOG_IN) {
                analog_pin pin = analog_pin_init(request->pin_config, request->port_attributes1, request->value1);
                if (pin != ANALOG_PIN_ERROR)
                    port_id = PORT_GROUP_ANALOG_IN | pin;
            } else if (request->port_type == WK_CFG_PORT_TYPE_PWM) {
                pwm_pin pin = pwm_pin_init(request->pin_config, (int16_t)request->value1);
                if (pin != PWM_PIN_ERROR)
                    port_id = PORT_GROUP_PWM | pin;
            } else if (request->port_type == WK_CFG_PORT_TYPE_I2C) {
                i2c_port port = i2c_master_init(request->pin_config, request->port_attributes1, request->value1);
                if (port != I2C_PORT_ERROR)
                    port_id = PORT_GROUP_I2C | port;
            } else if (request->port_type == WK_CFG_PORT_TYPE_SPI) {
                spi_port port = spi_master_init(request->pin_config, request->port_attributes2, request->port_attributes1, request->value1);
                if (port != SPI_PORT_ERROR)
                    port_id = PORT_GROUP_SPI | port;
            }
            if (port_id != 0) {
                send_config_response(WK_RESULT_OK, port_id, request->header.request_id, optional1, 0);
                return;
            } else {
                result = WK_RESULT_INV_DATA;
            }

        } else if (request->action == WK_CFG_ACTION_RELEASE) {
            uint16_t port_group = request->header.port_id & PORT_GROUP_MASK;
            if (port_group == PORT_GROUP_DIGI_PIN) {
                digital_pin pin = request->header.port_id & PORT_GROUP_DETAIL_MASK;
                digital_pin_release(pin);
            } else if (port_group == PORT_GROUP_ANALOG_IN) {
                analog_pin pin = request->header.port_id & PORT_GROUP_DETAIL_MASK;
                analog_pin_release(pin);
            } else if (port_group == PORT_GROUP_PWM) {
                pwm_pin pin = request->header.port_id & PORT_GROUP_DETAIL_MASK;
                pwm_pin_release(pin);
            } else if (port_group == PORT_GROUP_I2C) {
                i2c_port port = request->header.port_id & PORT_GROUP_DETAIL_MASK;
                i2c_port_release(port);
            } else if (port_group == PORT_GROUP_SPI) {
                spi_port port = request->header.port_id & PORT_GROUP_DETAIL_MASK;
                spi_port_release(port);
            } else {
                result = WK_RESULT_INV_DATA;
            }

        } else if (request->action == WK_CFG_ACTION_RESET) {
            DEBUG_OUT("RESET");
            wk_reset();
        
        } else if (request->action == WK_CFG_ACTION_CONFIG_MODULE) {
            if (request->port_type == WK_CFG_MODULE_PWM_TIMER) {
                pwm_timer_config(request->pin_config, request->value1, request->port_attributes1);
            
            } else if (request->port_type == WK_CFG_MODULE_PWM_CHANNEL) {
                pwm_channel_config(request->pin_config, request->value1, request->port_attributes1);
            } else {
                result = WK_RESULT_INV_DATA;
            }

        } else if (request->action == WK_CFG_ACTION_QUERY) {
            uint32_t value = 0;
            if (request->port_type == WK_CFG_QUERY_MEM_AVAIL) {
                value = mm_avail();
            } else if (request->port_type == WK_CFG_QUERY_MEM_MAX_BLOCK) {
                value = mm_max_avail_block();
#ifdef _DEBUG
            } else if (request->port_type == WK_CFG_QUERY_MEM_INTEGRITY) {
                value = mm_check_integrity();
#endif
            } else if (request->port_type == WK_CFG_QUERY_MEM_MCU) {
#if defined(__MKL26Z64__)
                value = WK_CFG_MCU_TEENSY_LC;
#elif defined(__MK20DX256__)
                value = WK_CFG_MCU_TEENSY_3_2;
#endif
            } else if (request->port_type == WK_CFG_QUERY_VERSION) {
                value = 0x0050; // 0.50 in BCD
            } else {
                result = WK_RESULT_INV_DATA;
            }
            
            send_config_response(result, request->header.port_id, request->header.request_id, 0, value);
            return;
        
        } else {
            result = WK_RESULT_INV_DATA;
            DEBUG_OUT("Invalid config msg");
        }
    }    
    
    send_config_response(result, request->header.port_id, request->header.request_id, 0, 0);
}


void handle_port_request(wk_port_request* request, uint8_t* deallocate_msg)
{
    if (request->header.message_size >= WK_PORT_REQUEST_ALLOC_SIZE(0)) {

        uint16_t port_group = request->header.port_id & PORT_GROUP_MASK;
        if (request->action == WK_PORT_ACTION_SET_VALUE) {
            if (port_group == PORT_GROUP_DIGI_PIN) {
                if (request->action_attribute2 == 0) {
                    digital_pin_set_output(request->header.port_id & PORT_GROUP_DETAIL_MASK, (uint8_t)request->value1);
                } else if ((request->action_attribute2 & PORT_GROUP_MASK) == PORT_GROUP_SPI) {
                    take_ownership_or_copy(&request, deallocate_msg);
                    spi_set_digital_output(request);
                }
            } else if (port_group == PORT_GROUP_PWM) {
                pwm_pin_set_value(request->header.port_id & PORT_GROUP_DETAIL_MASK, (int32_t)request->value1);
            }

        } else if (request->action == WK_PORT_ACTION_GET_VALUE) {
            if (port_group == PORT_GROUP_DIGI_PIN) {
                uint8_t value = digital_pin_get_input(request->header.port_id & PORT_GROUP_DETAIL_MASK);
                wk_send_port_event(request->header.port_id, WK_EVENT_SINGLE_SAMPLE, request->header.request_id, value);

            } else if (port_group == PORT_GROUP_ANALOG_IN) {
                analog_request_conversion(request->header.port_id & PORT_GROUP_DETAIL_MASK);
            }

        } else if (request->action == WK_PORT_ACTION_TX_DATA) {
            if (port_group == PORT_GROUP_I2C) {
                take_ownership_or_copy(&request, deallocate_msg);
                i2c_master_start_send(request);                

            } else if (port_group == PORT_GROUP_SPI) {
                take_ownership_or_copy(&request, deallocate_msg);
                spi_master_start_send(request);                
            }

        } else if (request->action == WK_PORT_ACTION_RX_DATA) {
            if (port_group == PORT_GROUP_I2C) {
                i2c_master_start_recv(request);
            }

        } else if (request->action == WK_PORT_ACTION_TX_N_RX_DATA) {
            if (port_group == PORT_GROUP_I2C) {
                take_ownership_or_copy(&request, deallocate_msg);
                i2c_master_start_send(request);
            }

        }
    }
}


void take_ownership_or_copy(wk_port_request** request, uint8_t* deallocate_msg)
{
    if (*deallocate_msg) {
        // take ownership
        *deallocate_msg = 0;
    } else {
        // cannot take ownership; make copy
        wk_port_request* request2 = (wk_port_request*)mm_alloc((*request)->header.message_size);
        if (request2 != NULL) {
            memcpy(request2, *request, (*request)->header.message_size);
        } else {
            DEBUG_OUT("Insufficient memory to store request");
            wk_msg_header* hdr = &(*request)->header;
            if (hdr->message_type == WK_MSG_TYPE_CONFIG_REQUEST)
                send_config_response(WK_RESULT_OUT_OF_MEM, hdr->port_id, hdr->request_id, 0, 0);
            else
                wk_send_port_event(hdr->port_id, WK_EVENT_ERROR, hdr->request_id, WK_RESULT_OUT_OF_MEM);
        }
        *request = request2;
    }
}


void send_config_response(uint16_t result, uint16_t port_id, uint16_t request_id, uint16_t optional1, uint32_t value1)
{
    wk_config_response* response = (wk_config_response*) mm_alloc(sizeof(wk_config_response));
    if (response == NULL) {
        DEBUG_OUT("Insufficient memory for config response");
        return; // drop data
    }

    response->header.message_size = sizeof(wk_config_response);
    response->header.message_type = WK_MSG_TYPE_CONFIG_RESPONSE;
    response->result = result;
    response->header.port_id = port_id;
    response->header.request_id = request_id;
    response->optional1 = optional1;
    response->value1 = value1;
    
    endp1_tx_msg(&response->header);
}


void wk_send_port_event(uint16_t port_id, uint8_t evt, uint16_t request_id, uint32_t value)
{
    wk_send_port_event_2(port_id, evt, request_id, 0, 0, value, NULL, 0);
}


void wk_send_port_event_2(uint16_t port_id, uint8_t evt, uint16_t request_id, uint8_t attr1, uint16_t attr2, uint32_t value1, uint8_t* data, uint16_t data_len)
{
    uint32_t msg_size = WK_PORT_EVENT_ALLOC_SIZE(data_len);
    wk_port_event* event = (wk_port_event*) mm_alloc(msg_size);
    if (event == NULL) {
        DEBUG_OUT("Insufficient memory for port event");
        return; // drop data
    }

    event->header.message_size = msg_size;
    event->header.message_type = WK_MSG_TYPE_PORT_EVENT;
    event->header.port_id = port_id;
    event->event = evt;
    event->event_attribute1 = attr1;
    event->event_attribute2 = attr2;
    event->value1 = value1;
    event->header.request_id = request_id;
    if (data_len > 0)
        memcpy(event->data, data, data_len);
        
    endp1_tx_msg(&event->header);
}



/*
 * Reset all configured ports
 */
void wk_reset()
{
    DEBUG_OUT("RST");
    digital_pin_reset();
    analog_reset();
    pwm_reset();
    i2c_reset();
    spi_reset();
    endp1_clear_tx_buffer();
}


extern volatile uint32_t systick_millis_count;

void systick_isr(void)
{
	systick_millis_count++;
    analog_timer_tick();
}
