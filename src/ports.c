/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

 #include <stdlib.h>
 #include "kinetis.h"
 #include "ports.h"


volatile uint32_t* const PCR_ADDR[] = {
    &PORTA_PCR0,
    &PORTB_PCR0,
    &PORTC_PCR0,
    &PORTD_PCR0,
    &PORTE_PCR0
};


GPIO_PORT_t* const GPIO_PORT[] = {
    (GPIO_PORT_t*)&GPIOA_PDOR,
    (GPIO_PORT_t*)&GPIOB_PDOR,
    (GPIO_PORT_t*)&GPIOC_PDOR,
    (GPIO_PORT_t*)&GPIOD_PDOR,
    (GPIO_PORT_t*)&GPIOE_PDOR
};


