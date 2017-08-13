/*
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

typedef struct {
    uint8_t port : 3;
    uint8_t pin : 5;
} pin_map_t;

// pin map for Teensy LC
static const pin_map_t pin_map[] = {
    { PORT_B, 16 },
    { PORT_B, 17 },
    { PORT_D, 0 },
    { PORT_A, 1 },
    { PORT_A, 2 },
    { PORT_D, 7 },
    { PORT_D, 4 },
    { PORT_D, 2 },
    { PORT_D, 3 },
    { PORT_C, 3 },
    { PORT_C, 4 },
    { PORT_C, 6 },
    { PORT_C, 7 },
    { PORT_C, 5 },
    { PORT_D, 1 },
    { PORT_C, 0 },
    { PORT_B, 0 },
    { PORT_B, 1 },
    { PORT_B, 3 },
    { PORT_B, 2 },
    { PORT_D, 5 },
    { PORT_D, 6 },
    { PORT_C, 1 },
    { PORT_C, 2 },
    { PORT_E, 20 },
    { PORT_E, 21 },
    { PORT_E, 30 }
};

#define NUM_PINS (sizeof(pin_map)/sizeof(pin_map[0]))


static volatile uint32_t* PCR_ADDR[] = {
    &PORTA_PCR0,
    &PORTB_PCR0,
    &PORTC_PCR0,
    &PORTD_PCR0,
    &PORTE_PCR0
};

#define PCR_PTR(map) (PCR_ADDR[map.port] + map.pin)

static volatile uint32_t* GPIO_BASE_ADDR[] = {
    &GPIOA_PDOR,
    &GPIOB_PDOR,
    &GPIOC_PDOR,
    &GPIOD_PDOR,
    &GPIOE_PDOR
};

#define GPIO_BASE_PTR(map) (GPIO_BASE_ADDR[map.port])


// information about pins in use
typedef struct {
    pin_map_t map;
    uint8_t is_used : 1;
    uint8_t uses_interrupt : 1;
} pin_info_t;

static pin_info_t pins[NUM_PINS];
static int8_t num_init_pins = 0;


digital_pin digital_pin_init(uint8_t pin_idx, uint8_t direction, uint16_t attributes)
{
    if (pin_idx >= NUM_PINS)
        return DIGI_PIN_ERROR;

    // get free pin slot
    digital_pin p = 0;
    while (p < num_init_pins) {
        if (!pins[p].is_used)
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

    // initialize pin info
    pin_map_t map = pin_map[pin_idx];
    pins[p].map = map;
    pins[p].is_used = 1;
    pins[p].uses_interrupt = 0;
    
    // configure direction
    volatile uint32_t* gpio_ptr = GPIO_BASE_PTR(map) + 5; // FGPIOx_PDDR
    uint32_t mask = 1 << (uint32_t)map.pin;
    if (direction == DIGI_PIN_INPUT) {
        *gpio_ptr &= ~mask;
    } else {
        *gpio_ptr |= mask;
    }

    // calculate pin configuration
    uint32_t pin_control = PORT_PCR_MUX(1);
    if (direction == DIGI_PIN_OUTPUT) {
        pin_control |= PORT_PCR_SRE;
        if (attributes & DIGI_PIN_OUT_HIGH_CURRENT)
            pin_control |= PORT_PCR_DSE;
    } else {
        pin_control |= PORT_PCR_PFE;
        if (attributes & DIGI_PIN_IN_PULLUP)
            pin_control |= PORT_PCR_PE | PORT_PCR_PS;
        else if (attributes & DIGI_PIN_IN_PULLDOWN)
            pin_control |= PORT_PCR_PE;
    }

    // assign physical pin
    volatile uint32_t* pcr_ptr = PCR_PTR(map);
    *pcr_ptr = pin_control;

    // configure interrupt if needed
    if (direction == DIGI_PIN_INPUT
            && (attributes & (DIGI_PIN_IN_TRIGGER_RAISING | DIGI_PIN_IN_TRIGGER_FALLING))) {
        pins[p].uses_interrupt = 1;
        pin_control = 0;
        if (attributes & DIGI_PIN_IN_TRIGGER_RAISING)
            pin_control |= PORT_PCR_IRQC(0b1001);
        if (attributes & DIGI_PIN_IN_TRIGGER_FALLING)
            pin_control |= PORT_PCR_IRQC(0b1010);
        *pcr_ptr |= pin_control;
    }

    // return slot index
    return p;
}


void digital_pin_release(digital_pin pin)
{
    pin_map_t map = pins[pin].map;
    pins[pin].is_used = 0;

    // disable pin
    volatile uint32_t* pcr_ptr = PCR_PTR(map);
    *pcr_ptr = PORT_PCR_ISF;

    uint32_t mask = 1 << (uint32_t)map.pin;
    volatile uint32_t* gpio_base_ptr = GPIO_BASE_PTR(map); 

    // reset value
    *(gpio_base_ptr + 2) = mask; // FGPIOx_PCOR
    
    // reset direction
    *(gpio_base_ptr + 5) &= ~mask; // FGPIOx_PDDR
}


void digital_pin_set_output(digital_pin pin, uint8_t value)
{
    pin_map_t map = pins[pin].map;
    
    // compute register address
    volatile uint32_t* gpio_ptr = GPIO_BASE_PTR(map);
    if (value)
        gpio_ptr += 1; // FGPIOx_PSOR
    else
        gpio_ptr += 2; // FGPIOx_PCOR

    // set value
    uint32_t mask = 1 << (uint32_t)map.pin;
    *gpio_ptr = mask;
}


uint8_t digital_pin_get_input(digital_pin pin)
{
    pin_map_t map = pins[pin].map;
    
    // compute register address
    volatile uint32_t* gpio_ptr = GPIO_BASE_PTR(map) + 4;

    // get value
    uint32_t mask = 1 << (uint32_t)map.pin;
    return (*gpio_ptr & mask) ? DIGI_PIN_ON : DIGI_PIN_OFF;
}


digital_pin digital_pin_get_interrupt_pin()
{
    // check for digital pins with enabled interrupt
    for (int i = 0; i < num_init_pins; i++) {
        if (pins[i].uses_interrupt) {

            // check if the pin's interrupt flag is set
            volatile uint32_t* pcr_ptr = PCR_PTR(pins[i].map);
            if (*pcr_ptr & PORT_PCR_ISF) {
                *pcr_ptr |= PORT_PCR_ISF; // clear the flag
                return i;
            }
        }
    }

    return 0xff;
}


void digital_pin_reset()
{
    for (int i = 0; i < num_init_pins; i++) {
        if (pins[i].is_used)
            digital_pin_release(i);
    }
    num_init_pins = 0;

    NVIC_DISABLE_IRQ(IRQ_PORTA);
    NVIC_CLEAR_PENDING(IRQ_PORTA);

    NVIC_DISABLE_IRQ(IRQ_PORTCD);
    NVIC_CLEAR_PENDING(IRQ_PORTCD);
}
