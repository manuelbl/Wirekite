/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include "digital_pin.h"
#include "kinetis.h"

#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3
#define PORT_E 4
#define PORT_ENC(t)     (t << 4)
#define PORT_IDX(n)     ((n >> 4) & 0x7)

#define INTERRUPT_EN    0x80

#define PIN_IDX_ENC(n)  | n
#define PIN_IDX(n)      (n & 0xf)

#define PIN_UNASSIGNED  0xff


static const uint8_t pin_map[] = {
    PORT_ENC(PORT_B)  PIN_IDX_ENC(16),
    PORT_ENC(PORT_B)  PIN_IDX_ENC(17),
    PORT_ENC(PORT_D)  PIN_IDX_ENC(0),
    PORT_ENC(PORT_A)  PIN_IDX_ENC(1),
    PORT_ENC(PORT_A)  PIN_IDX_ENC(2),
    PORT_ENC(PORT_D)  PIN_IDX_ENC(7),
    PORT_ENC(PORT_D)  PIN_IDX_ENC(4),
    PORT_ENC(PORT_D)  PIN_IDX_ENC(2),
    PORT_ENC(PORT_D)  PIN_IDX_ENC(3),
    PORT_ENC(PORT_C)  PIN_IDX_ENC(3),
    PORT_ENC(PORT_C)  PIN_IDX_ENC(4),
    PORT_ENC(PORT_C)  PIN_IDX_ENC(6),
    PORT_ENC(PORT_C)  PIN_IDX_ENC(7),
    PORT_ENC(PORT_C)  PIN_IDX_ENC(5),
    PORT_ENC(PORT_D)  PIN_IDX_ENC(1),
    PORT_ENC(PORT_C)  PIN_IDX_ENC(0),
    PORT_ENC(PORT_B)  PIN_IDX_ENC(0),
    PORT_ENC(PORT_B)  PIN_IDX_ENC(1),
    PORT_ENC(PORT_B)  PIN_IDX_ENC(3),
    PORT_ENC(PORT_B)  PIN_IDX_ENC(2),
    PORT_ENC(PORT_D)  PIN_IDX_ENC(5),
    PORT_ENC(PORT_D)  PIN_IDX_ENC(6),
    PORT_ENC(PORT_C)  PIN_IDX_ENC(1),
    PORT_ENC(PORT_C)  PIN_IDX_ENC(2),
    PORT_ENC(PORT_E)  PIN_IDX_ENC(20),
    PORT_ENC(PORT_E)  PIN_IDX_ENC(21),
    PORT_ENC(PORT_E)  PIN_IDX_ENC(30)
};

static uint8_t pins[27];
static int8_t num_init_pins = 0;


static volatile uint32_t* pcr_addr(uint8_t pin_info);
static volatile uint32_t* gpio_base_addr(uint8_t pin_info);


digital_pin digital_pin_init(uint8_t pin_idx, uint8_t direction, uint16_t attributes)
{
    // get free pin slot
    digital_pin p = 0;
    while (p < num_init_pins) {
        if (pins[p] == PIN_UNASSIGNED)
            break;
        p++;
    }
    if (p == num_init_pins)
        num_init_pins++;

    // enable interrupts
    if (p == 0) {
        NVIC_CLEAR_PENDING(IRQ_PORTA);
        NVIC_SET_PRIORITY(IRQ_PORTA, 128);
        NVIC_ENABLE_IRQ(IRQ_PORTA);

        NVIC_CLEAR_PENDING(IRQ_PORTCD);
        NVIC_SET_PRIORITY(IRQ_PORTCD, 128);
        NVIC_ENABLE_IRQ(IRQ_PORTCD);
    }

    // get pin information
    uint16_t pin_info = pin_map[pin_idx];

    // configure direction
    volatile uint32_t* gpio_ptr = gpio_base_addr(pin_info) + 5; // FGPIOx_PDDR
    uint32_t mask = 1 << (uint32_t) PIN_IDX(pin_info);
    if (direction == DIGI_PIN_INPUT) {
        *gpio_ptr &= ~mask;
    } else {
        *gpio_ptr |= mask;
    }

    // calculate pin configuration
    uint32_t pin_control = PORT_PCR_MUX(1);
    if (direction == DIGI_PIN_OUTPUT) {
        if (attributes & DIGI_PIN_OUT_HIGH_CURRENT)
            pin_control |= PORT_PCR_DSE;
    } else {
        if (attributes & DIGI_PIN_IN_PULLUP)
            pin_control |= PORT_PCR_PE | PORT_PCR_PS;
        else if (attributes & DIGI_PIN_IN_PULLDOWN)
            pin_control |= PORT_PCR_PE;
    }

    // assign physical pin
    volatile uint32_t* pcr_ptr = pcr_addr(pin_info);
    *pcr_ptr = pin_control;

    // configure interrupt if needed
    if (direction == DIGI_PIN_INPUT
            && (attributes & (DIGI_PIN_IN_TRIGGER_RAISING | DIGI_PIN_IN_TRIGGER_FALLING))) {
        pin_info |= INTERRUPT_EN;
        pin_control = 0;
        if (attributes & DIGI_PIN_IN_TRIGGER_RAISING)
            pin_control |= PORT_PCR_IRQC(0b1001);
        if (attributes & DIGI_PIN_IN_TRIGGER_FALLING)
            pin_control |= PORT_PCR_IRQC(0b1010);
        *pcr_ptr |= pin_control;
    }

    // save pin info
    pins[p] = pin_info;

    // return slot index
    return p;
}


void digital_pin_release(digital_pin pin)
{
    uint8_t pin_info = pins[pin];
    pins[pin] = PIN_UNASSIGNED;

    // disable pin
    volatile uint32_t* pcr_ptr = pcr_addr(pin_info);
    *pcr_ptr = PORT_PCR_ISF;

    uint32_t mask = 1 << (uint32_t) PIN_IDX(pin_info);
    volatile uint32_t* gpio_base_ptr = gpio_base_addr(pin_info); 

    // reset value
    *(gpio_base_ptr + 2) = mask; // FGPIOx_PCOR
    
    // reset direction
    *(gpio_base_ptr + 5) &= ~mask; // FGPIOx_PDDR
}


void digital_pin_set_output(digital_pin pin, uint8_t value)
{
    uint8_t pin_info = pins[pin];

    // compute register address
    volatile uint32_t* gpio_ptr = gpio_base_addr(pin_info);
    if (value)
        gpio_ptr += 1; // FGPIOx_PSOR
    else
        gpio_ptr += 2; // FGPIOx_PCOR

    // set value
    uint32_t mask = 1 << (uint32_t) PIN_IDX(pin_info);
    *gpio_ptr = mask;
}


uint8_t digital_pin_get_input(digital_pin pin)
{
    uint8_t pin_info = pins[pin];

    // compute register address
    volatile uint32_t* gpio_ptr = gpio_base_addr(pin_info) + 4;

    // get value
    uint32_t mask = 1 << (uint32_t) PIN_IDX(pin_info);
    return (*gpio_ptr & mask) ? DIGI_PIN_ON : DIGI_PIN_OFF;
}


volatile uint32_t* pcr_addr(uint8_t pin_info)
{
    uint8_t port = PORT_IDX(pin_info);

    // compute pin control register addr
    volatile uint32_t * pcr_ptr;
    switch (port) {
        case PORT_A:
            pcr_ptr = &PORTA_PCR0;
            break;
        case PORT_B:
            pcr_ptr = &PORTB_PCR0;
            break;
        case PORT_C:
            pcr_ptr = &PORTC_PCR0;
            break;
        case PORT_D:
            pcr_ptr = &PORTD_PCR0;
            break;
        default:
            pcr_ptr = &PORTE_PCR0;
            break;
    }
    uint8_t pin = PIN_IDX(pin_info);
    
    return pcr_ptr + pin;
}


volatile uint32_t* gpio_base_addr(uint8_t pin_info)
{
    uint8_t port = PORT_IDX(pin_info);

    // compute pin control register addr
    volatile uint32_t * pcr_ptr;
    switch (port) {
        case PORT_A:
            pcr_ptr = &GPIOA_PDOR;
            break;
        case PORT_B:
            pcr_ptr = &GPIOB_PDOR;
            break;
        case PORT_C:
            pcr_ptr = &GPIOC_PDOR;
            break;
        case PORT_D:
            pcr_ptr = &GPIOD_PDOR;
            break;
        default:
            pcr_ptr = &GPIOE_PDOR;
            break;
    }
    
    return pcr_ptr;
}


digital_pin digital_pin_get_interrupt_pin()
{
    // check for digital pins with enabled interrupt
    for (int32_t i = 0; i < num_init_pins; i++) {
        uint8_t pin_info = pins[i];
        if (pin_info & INTERRUPT_EN) {

            // check if the pin's interrupt flag is set
            volatile uint32_t* pcr = pcr_addr(pin_info);
            if (*pcr & PORT_PCR_ISF) {
                *pcr |= PORT_PCR_ISF; // clear the flag
                return i;
            }
        }
    }

    return 0xff;
}


void digital_pin_reset()
{
    for (int i = 0; i < num_init_pins; i++) {
        if (pins[i] != PIN_UNASSIGNED)
            digital_pin_release(i);
    }
    num_init_pins = 0;

    NVIC_DISABLE_IRQ(IRQ_PORTA);
    NVIC_CLEAR_PENDING(IRQ_PORTA);

    NVIC_DISABLE_IRQ(IRQ_PORTCD);
    NVIC_CLEAR_PENDING(IRQ_PORTCD);
}
