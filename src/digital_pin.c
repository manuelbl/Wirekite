/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include "digital_pin.h"
#include "ports.h"
#include "proto.h"
#include "wirekite.h"
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

#if defined(__MKL26Z64__)
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

#elif defined(__MK20DX256__)

static const pin_map_t pin_map[] = {
    { PORT_B, 16 }, // PTB16
    { PORT_B, 17 }, // PTB17
    { PORT_D, 0 }, // PTD0
    { PORT_A, 1 }, // PTA12
    { PORT_A, 2 }, // PTA13
    { PORT_D, 7 }, // PTD7
    { PORT_D, 4 }, // PTD4
    { PORT_D, 2 }, // PTD2
    { PORT_D, 3 }, // PTD3
    { PORT_C, 3 }, // PTC3
    { PORT_C, 4 }, // PTC4
    { PORT_C, 6 }, // PTC6
    { PORT_C, 7 }, // PTC7
    { PORT_C, 5 }, // PTC5
    { PORT_D, 1 }, // PTD1
    { PORT_C, 0 }, // PTC0
    { PORT_B, 0 }, // PTB0
    { PORT_B, 1 }, // PTB1
    { PORT_B, 3 }, // PTB3
    { PORT_B, 2 }, // PTB2
    { PORT_D, 5 }, // PTD5
    { PORT_D, 6 }, // PTD6
    { PORT_C, 1 }, // PTC1
    { PORT_C, 2 }, // PTC2
    { PORT_A, 5 }, // PTA5
    { PORT_B, 19 }, // PTB19
    { PORT_E, 1 }, // PTE1
    { PORT_C, 9 }, // PTC9
    { PORT_C, 8 }, // PTC8
    { PORT_C, 10 }, // PTC10
    { PORT_C, 11 }, // PTC11
    { PORT_E, 0 }, // PTE0
    { PORT_B, 18 }, // PTB18
    { PORT_A, 4 } // PTA4
};

#endif


#define NUM_PINS (sizeof(pin_map)/sizeof(pin_map[0]))


// information about pins in use
typedef struct {
    pin_map_t map;
    uint8_t is_used : 1;
    uint8_t uses_interrupt : 1;
} pin_info_t;

static pin_info_t pins[NUM_PINS];
static int8_t num_init_pins = 0;


digital_pin digital_pin_init(uint8_t pin_idx, uint8_t direction, uint16_t attributes, uint8_t initial_value)
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
        NVIC_ENABLE_IRQ(IRQ_PORTA);

#if defined(__MKL26Z64__)
        // Teensy LC
        NVIC_CLEAR_PENDING(IRQ_PORTCD);
        NVIC_ENABLE_IRQ(IRQ_PORTCD);

#elif defined(__MK20DX256__)
        // Teensy 3.2
        NVIC_CLEAR_PENDING(IRQ_PORTB);
        NVIC_ENABLE_IRQ(IRQ_PORTB);

        NVIC_CLEAR_PENDING(IRQ_PORTC);
        NVIC_ENABLE_IRQ(IRQ_PORTC);

        NVIC_CLEAR_PENDING(IRQ_PORTD);
        NVIC_ENABLE_IRQ(IRQ_PORTD);

        NVIC_CLEAR_PENDING(IRQ_PORTE);
        NVIC_ENABLE_IRQ(IRQ_PORTE);
#endif
    }

    // initialize pin info
    pin_map_t map = pin_map[pin_idx];
    pins[p].map = map;
    pins[p].is_used = 1;
    pins[p].uses_interrupt = 0;
    
    // configure direction
    uint32_t mask = 1 << (uint32_t)map.pin;
    if (direction == DIGI_PIN_INPUT) {
        GPIO_PORT[map.port]->PDDR &= ~mask;
    } else {
        // set initial value
        digital_pin_set_output(p, initial_value);
        GPIO_PORT[map.port]->PDDR |= mask;
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
    volatile uint32_t* pcr_ptr = PCR_PTR(map.port, map.pin);
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
    if (pin >= num_init_pins)
        return;

    pin_map_t map = pins[pin].map;
    pins[pin].is_used = 0;

    // disable pin
    PCR(map.port, map.pin) = PORT_PCR_ISF;

    uint32_t mask = 1 << (uint32_t)map.pin;
    
    // reset direction
    GPIO_PORT[map.port]->PDDR &= ~mask;

    // reset value
    GPIO_PORT[map.port]->PCOR = mask;
}


void digital_pin_set_output(digital_pin pin, uint8_t value)
{
    if (pin >= num_init_pins)
        return;
        
    pin_map_t map = pins[pin].map;

    // set value
    uint32_t mask = 1 << (uint32_t)map.pin;
    if (value)
        GPIO_PORT[map.port]->PSOR = mask;
    else
        GPIO_PORT[map.port]->PCOR = mask;
}


uint8_t digital_pin_get_input(digital_pin pin)
{
    if (pin >= num_init_pins)
        return 0;
    
    pin_map_t map = pins[pin].map;
    
    // get value
    uint32_t mask = 1 << (uint32_t)map.pin;
    return (GPIO_PORT[map.port]->PDIR & mask) ? DIGI_PIN_ON : DIGI_PIN_OFF;
}


digital_pin digital_pin_get_interrupt_pin()
{
    // check for digital pins with enabled interrupt
    for (int i = 0; i < num_init_pins; i++) {
        if (pins[i].uses_interrupt) {

            // check if the pin's interrupt flag is set
            volatile uint32_t* pcr_ptr = PCR_PTR(pins[i].map.port, pins[i].map.pin);
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

#if defined(__MKL26Z64__)
    // Teensy LC
    NVIC_DISABLE_IRQ(IRQ_PORTCD);

#elif defined(__MK20DX256__)
    // Teensy 3.2
    NVIC_DISABLE_IRQ(IRQ_PORTB);

    NVIC_DISABLE_IRQ(IRQ_PORTC);

    NVIC_DISABLE_IRQ(IRQ_PORTD);

    NVIC_DISABLE_IRQ(IRQ_PORTE);

#endif
}


/*
 * Interrupt hander for digital input pins
 */
void porta_isr()
{
     while (1) {
         digital_pin pin = digital_pin_get_interrupt_pin();
         if (pin == 0xff)
             break;
 
         uint8_t value = digital_pin_get_input(pin);
         wk_send_port_event(PORT_GROUP_DIGI_PIN | pin, WK_EVENT_SINGLE_SAMPLE, 0, value);
     }
 }
 
 
 void portb_isr()
 {
     porta_isr();
 }
 
 
 void portc_isr()
 {
     porta_isr();
 }
 
 
 void portd_isr()
 {
     porta_isr();
 }
 
 
 void porte_isr()
 {
     porta_isr();
 }
 
 
 void portcd_isr()
 {
     porta_isr();
 }
 