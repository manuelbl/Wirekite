/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include "kinetis.h"

#include "analog.h"
#include "mem.h"
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
#ifdef _DEBUG
    uart0_init(115200);
    DEBUG_OUT("START");
#endif

    mm_init(NULL, 0);
    pwm_init();
    analog_init();
    i2c_init();

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


void rtc_set(unsigned long t)
{
}
