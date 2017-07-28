/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include "kinetis.h"
#include "analog.h"
#include "analog_config.h"

#define PORT_CHANNEL_X      0x00
#define PORT_CHANNEL_A      0x00
#define PORT_CHANNEL_B      0x40
#define PORT_CHANNEL_MASK   0x40
#define ANALOG_PIN_CALIB  0xfc

#define REQUEST_NUM_BUFFERS 8
#define MAX_TRIGGERS 8

typedef struct {
    uint32_t treshold;
    uint32_t count;
    uint8_t pin;
} trigger_t;

static int8_t is_calibrated = 0;
static uint8_t running_conversion = ANALOG_PIN_NONE;
static uint8_t circ_request_buf[REQUEST_NUM_BUFFERS];
static uint8_t circ_request_head = 0;
static uint8_t circ_request_tail = 0;
static uint8_t num_triggers = 0;
static trigger_t triggers[MAX_TRIGGERS];


static void start_conversion(uint8_t pin);
static void start_calibration();
static void complete_calibration();
static void insert_trigger(uint8_t pin, uint32_t treshold);
static void remove_trigger(uint8_t pin);


// see table 3-35 ADC0 channel assignment
static uint8_t pin_map[] = {
    PORT_CHANNEL_B | 5,  // A0   ADC0_SE5b
    PORT_CHANNEL_X | 14, // A1   ADC0_SE14
    PORT_CHANNEL_X | 8,  // A2   ADC0_SE8
    PORT_CHANNEL_X | 9,  // A3   ADC0_SE9
    PORT_CHANNEL_X | 13, // A4   ADC0_SE13
    PORT_CHANNEL_X | 12, // A5   ADC0_SE12
    PORT_CHANNEL_B | 6,  // A6   ADC0_SE6b
    PORT_CHANNEL_B | 7,  // A7   ADC0_SE7b
    PORT_CHANNEL_X | 15, // A8   ADC0_SE15
    PORT_CHANNEL_X | 11, // A9   ADC0_SE11
    PORT_CHANNEL_X | 0,  // A10  ADC0_SE0
    PORT_CHANNEL_A | 4,  // A11  ADC0_SE4a
    PORT_CHANNEL_X | 23, // A12  ADC0_SE23
    PORT_CHANNEL_X | 29, // VREF
    PORT_CHANNEL_X | 26, // TEMP
    PORT_CHANNEL_X | 30, // VREFL
    PORT_CHANNEL_X | 27  // BANDGAP
};

#define VREF_IDX 13
#define BANDGAP_IDX 16

#define NUM_PINS (sizeof(pin_map) / sizeof(pin_map[0]))


void analog_init()
{
    // configure for 10 bits, long sample time
    ADC0_CFG1 = ADC_CFG1_10BIT + ADC_CFG1_MODE(2) + ADC_CFG1_ADLSMP;
    ADC0_CFG2 = ADC_CFG2_ADLSTS(3);
	
    // select reference voltage source
    ADC0_SC2 = ADC_SC2_REFSEL(1); // vcc

    // average 4 samples
	ADC0_SC3 = ADC_SC3_AVGE + ADC_SC3_AVGS(0);

    NVIC_CLEAR_PENDING(IRQ_ADC0);
    NVIC_SET_PRIORITY(IRQ_ADC0, 128);
    NVIC_ENABLE_IRQ(IRQ_ADC0);
}


analog_pin analog_pin_init(uint8_t pin_idx, uint16_t attributes, uint16_t trigger_ms)
{
    if (pin_idx >= ANALOG_PIN_VREF)
        pin_idx = pin_idx - ANALOG_PIN_VREF + VREF_IDX;

    if (!is_calibrated && running_conversion == ANALOG_PIN_NONE)
        start_calibration();

    // Enable band gap buffer
    if (pin_idx == BANDGAP_IDX)
        PMC_REGSC |= PMC_REGSC_BGBE;

    if (trigger_ms != 0)
        insert_trigger(pin_idx, trigger_ms);
    
    return pin_idx;
}


void analog_pin_release(analog_pin pin)
{
    remove_trigger(pin);

    // Disable band gap buffer
    if (pin == BANDGAP_IDX)
        PMC_REGSC &= ~PMC_REGSC_BGBE;

}


void analog_request_conversion(analog_pin pin)
{
    __disable_irq();

    if (running_conversion == ANALOG_PIN_NONE) {
        // no conversion on-going; start immediately
        running_conversion = pin;
        __enable_irq();
        start_conversion(pin);
        return;
    }

    // queue request
    uint8_t head = circ_request_head + 1;
    if (head >= REQUEST_NUM_BUFFERS)
        head = 0;
    if (head != circ_request_tail) {
        circ_request_buf[head] = pin;
        circ_request_head = head;
    } else {
        // buffer full; drop the request
    }

    __enable_irq();
}


void start_conversion(uint8_t pin)
{
    uint8_t pin_info = pin_map[pin];

    // select channel
    if (pin_info & PORT_CHANNEL_B)
        ADC0_CFG2 |= ADC_CFG2_MUXSEL;
    else
        ADC0_CFG2 &= ~ADC_CFG2_MUXSEL;
    
    ADC0_SC1A = ADC_SC1_AIEN | ADC_SC1_ADCH(pin_info & ~PORT_CHANNEL_MASK);
}


analog_pin analog_get_completed_pin(int16_t* value)
{
    uint16_t v = 0;
    uint8_t completed_conversion = running_conversion;

    if (completed_conversion == ANALOG_PIN_CALIB) {
        complete_calibration();
        completed_conversion = ANALOG_PIN_CALIB_COMPLETE;
    } else {
        v = ADC0_RA;
    }

    // check if new conversion should be started
    __disable_irq();

    if (circ_request_head != circ_request_tail) {
    
        circ_request_tail++;
        if (circ_request_tail >= REQUEST_NUM_BUFFERS)
            circ_request_tail = 0;
        
        uint8_t pin = circ_request_buf[circ_request_tail];
        running_conversion = pin;

        __enable_irq();

        start_conversion(pin);
    } else {
        running_conversion = ANALOG_PIN_NONE;
        __enable_irq();
    }
    // extend value from 10bits to 15bits
    *value = (v << 5) | (v >> 5);

    return completed_conversion;
}


void analog_reset()
{
    __disable_irq();
    is_calibrated = 0;
    running_conversion = ANALOG_PIN_NONE;
    circ_request_head = 0;
    circ_request_tail = 0;
    num_triggers = 0;
    ADC0_SC1A = ADC_SC1_ADCH(0b11111); // cancel conversion
    ADC0_SC3 = 0; // cancel calibration

    analog_init();
    __enable_irq();
}


void start_calibration()
{
    running_conversion = ANALOG_PIN_CALIB;
    ADC0_SC1A = ADC_SC1_AIEN | ADC_SC1_ADCH(0b11111);
    ADC0_SC3 |= ADC_SC3_CAL;
}


void complete_calibration()
{
    uint16_t v = ADC0_CLP0 + ADC0_CLP1 + ADC0_CLP2 + ADC0_CLP4 + ADC0_CLPS;
    v /= 2;
    v |= 0x8000;
    ADC0_PG = v;

    v = ADC0_CLM0 + ADC0_CLM1 + ADC0_CLM2 + ADC0_CLM4 + ADC0_CLMS;
    v /= 2;
    v |= 0x8000;
    ADC0_MG = v;

    is_calibrated = 1;
}


void analog_timer_tick()
{
    __disable_irq();
    for (int i = 0; i < num_triggers; i++) {
        uint32_t count = triggers[i].count;
        uint32_t treshold = triggers[i].treshold;
        count++;
        if (count >= treshold)
            count = 0;
        triggers[i].count = count;
        if (count == 0) {
            __enable_irq();
            analog_request_conversion(triggers[i].pin);
            __disable_irq();
        }
    }
    __enable_irq();
}


void insert_trigger(uint8_t pin, uint32_t treshold)
{
    __disable_irq();
    trigger_t* trigger = triggers + num_triggers;
    trigger->treshold = treshold;
    trigger->count = 0;
    trigger->pin = pin;
    num_triggers++;
    __enable_irq();
}


void remove_trigger(uint8_t pin)
{
    __disable_irq();

    int idx;
    for (idx = 0; idx < num_triggers; idx++) {
        if (triggers[idx].pin == pin) {
            for (int i = idx + 1; i < num_triggers; i++) {
                triggers[i - 1] = triggers[i];
            }
            num_triggers--;
        }
    }

    __enable_irq();
}


