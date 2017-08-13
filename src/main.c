/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include "kinetis.h"

#include "analog.h"
#include "buffers.h"
#include "digital_pin.h"
#include "i2c.h"
#include "pwm.h"
#include "timer.h"
#include "usb.h"
#include "util.h"
#include "wirekite.h"
#include "debug.h"


extern void uart_echo();
void check_usb();

extern int main(void)
{
    buffers_init();
    analog_init();
    pwm_init();
    i2c_init();

#ifdef _DEBUG
    uart0_init(115200);
    uart0_set_recv_evt(uart_echo);
    DEBUG_OUT("START");
#endif

    // create serial number
    char serial_number[20];
    uint32_t v = SIM_UIDML;
    bytes_to_hex(serial_number, ((uint8_t*)&v) + 2, 2);
    v = SIM_UIDL;
    bytes_to_hex(serial_number + 4, (uint8_t*)&v, 4);
    serial_number[12] = 0;

    usb_init(serial_number);

    while (1) {

        wk_check_usb_rx();
        
    }
}


#ifdef _DEBUG
void uart_echo()
{
    char buf[1];
    while (1) {
        uart0_read(buf, 1);
        uart0_write(buf, 1);
        if (buf[0] == '\r')
            uart0_write("\n", 1);
        if (uart0_avail() <= 0)
            break;
    }
}
#endif
