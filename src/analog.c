/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include "kinetis.h"
#include "analog.h"
#include "analog_config.h"
#include "debug.h"
#include "proto.h"
#include "wirekite.h"


#define MUX_X   0
#define MUX_A   0
#define MUX_B   1
#define ANALOG_PIN_CALIB_0  0xfc
#define ANALOG_PIN_CALIB_1  0xfb

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
static void start_calibration(uint8_t pin);
static void complete_calibration(uint8_t pin);
static void insert_trigger(uint8_t pin, uint32_t treshold);
static void remove_trigger(uint8_t pin);


typedef struct {
    uint16_t adc : 2;
    uint16_t mux : 1;
    uint16_t channel : 5;
} pin_map_t;


#if defined(__MKL26Z64__)

// Teensy LC
// see table 3-35 ADC0 channel assignment
static pin_map_t pin_map[] = {
    { 0, MUX_B,  5 },  // A0   ADC0_SE5b
    { 0, MUX_X, 14 },  // A1   ADC0_SE14
    { 0, MUX_X,  8 },  // A2   ADC0_SE8
    { 0, MUX_X,  9 },  // A3   ADC0_SE9
    { 0, MUX_X, 13 },  // A4   ADC0_SE13
    { 0, MUX_X, 12 },  // A5   ADC0_SE12
    { 0, MUX_B,  6 },  // A6   ADC0_SE6b
    { 0, MUX_B,  7 },  // A7   ADC0_SE7b
    { 0, MUX_X, 15 },  // A8   ADC0_SE15
    { 0, MUX_X, 11 },  // A9   ADC0_SE11
    { 0, MUX_X,  0 },  // A10  ADC0_SE0
    { 0, MUX_A,  4 },  // A11  ADC0_SE4a
    { 0, MUX_X, 23 },  // A12  ADC0_SE23
    { 0, MUX_X, 29 },  // VREF
    { 0, MUX_X, 26 },  // TEMP
    { 0, MUX_X, 30 },  // VREFL
    { 0, MUX_X, 27 }   // BANDGAP
};

#define VREF_IDX 13
#define BANDGAP_IDX 16

#elif defined(__MK20DX256__)

// Teensy 3.2
// see table 3-35 ADC0 channel assignment
static pin_map_t pin_map[] = {
    { 0, MUX_B,  5 },  // A0   ADC0_SE5b
    { 0, MUX_X, 14 },  // A1   ADC0_SE14
    { 0, MUX_X,  8 },  // A2   ADC0_SE8 / ADC1_SE8
    { 0, MUX_X,  9 },  // A3   ADC0_SE9 / ADC1_SE9
    { 0, MUX_X, 13 },  // A4   ADC0_SE13
    { 0, MUX_X, 12 },  // A5   ADC0_SE12
    { 0, MUX_B,  6 },  // A6   ADC0_SE6b
    { 0, MUX_B,  7 },  // A7   ADC0_SE7b
    { 0, MUX_X, 15 },  // A8   ADC0_SE15
    { 0, MUX_B,  4 },  // A9   ADC0_SE4b
    { 0, MUX_X,  0 },  // A10  ADC0_DP0 / ADC1_DP3
    { 0, MUX_X, 19 },  // A11  ADC0_DM0 / ADC1_DM3
    { 0, MUX_X,  3 },  // A12  ADC1_DP0 / ADC0_DP3
    { 1, MUX_X, 19 },  // A13  ADC1_DM0 / ADC0_DM3
    { 0, MUX_X, 23 },  // A14  ADC0_SE23
    { 1, MUX_A,  5 },  // A15  ADC1_SE5a
    { 1, MUX_B,  5 },  // A16  ADC1_SE5b
    { 1, MUX_B,  4 },  // A17  ADC1_SE4b
    { 1, MUX_B,  6 },  // A18  ADC1_SE6b
    { 1, MUX_B,  7 },  // A19  ADC1_SE7b
    { 1, MUX_A,  4 },  // A20  ADC1_SE4a
    { 0, MUX_X, 22 },  // VREF
    { 0, MUX_X, 26 },  // TEMP
    { 0, MUX_X, 30 },  // VREFL
    { 0, MUX_X, 27 }   // BANDGAP
};

#define VREF_IDX 21
#define BANDGAP_IDX 24

#endif


#define NUM_PINS (sizeof(pin_map) / sizeof(pin_map[0]))


void analog_init()
{
    // configure for 10 bits, long sample time
    ADC0_CFG1 = ADC_CFG1_10BIT + ADC_CFG1_MODE(2) + ADC_CFG1_ADLSMP;
    ADC0_CFG2 = ADC_CFG2_ADLSTS(3);
	
    // select reference voltage source
#if defined(__MKL26Z64__)
    ADC0_SC2 = ADC_SC2_REFSEL(1); // vcc
#elif defined (__MK20DX256__)
    ADC0_SC2 = ADC_SC2_REFSEL(0); // vcc
#endif

    NVIC_CLEAR_PENDING(IRQ_ADC0);
    NVIC_ENABLE_IRQ(IRQ_ADC0);

#if defined(__MK20DX256__)

    // configure for 10 bits, long sample time
    ADC1_CFG1 = ADC_CFG1_10BIT + ADC_CFG1_MODE(2) + ADC_CFG1_ADLSMP;
    ADC1_CFG2 = ADC_CFG2_ADLSTS(3);
	
    // select reference voltage source
    ADC1_SC2 = ADC_SC2_REFSEL(1); // vcc

    NVIC_CLEAR_PENDING(IRQ_ADC1);
    NVIC_ENABLE_IRQ(IRQ_ADC1);

#endif
}


analog_pin analog_pin_init(uint8_t pin_idx, uint16_t attributes, uint16_t trigger_ms)
{
    if (pin_idx >= ANALOG_PIN_VREF)
        pin_idx = pin_idx - ANALOG_PIN_VREF + VREF_IDX;
    if (pin_idx >= NUM_PINS)
        return ANALOG_PIN_ERROR;

    // Enable band gap buffer
    if (pin_idx == BANDGAP_IDX)
        PMC_REGSC |= PMC_REGSC_BGBE;

    if (!is_calibrated && running_conversion == ANALOG_PIN_NONE)
        start_calibration(ANALOG_PIN_CALIB_0);

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
    if (pin >= NUM_PINS)
        return; // invalid pin
        
    if (running_conversion == ANALOG_PIN_NONE) {
        // no conversion on-going; start immediately
        running_conversion = pin;
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
}


void start_conversion(uint8_t pin)
{
    pin_map_t info = pin_map[pin];

#if defined(__MK20DX256__)
    if (info.adc == 0) {
#endif

        if (info.mux == MUX_B)
            ADC0_CFG2 |= ADC_CFG2_MUXSEL;
        else
            ADC0_CFG2 &= ~ADC_CFG2_MUXSEL;

        ADC0_SC1A = ADC_SC1_AIEN | ADC_SC1_ADCH(info.channel);

#if defined(__MK20DX256__)

    } else {
        if (info.mux == MUX_B)
            ADC1_CFG2 |= ADC_CFG2_MUXSEL;
        else
            ADC1_CFG2 &= ~ADC_CFG2_MUXSEL;

        ADC1_SC1A = ADC_SC1_AIEN | ADC_SC1_ADCH(info.channel);
    }

#endif    
    
}


analog_pin analog_get_completed_pin(int32_t* value)
{
    uint32_t v = 0;
    uint8_t completed_conversion = running_conversion;

    if (completed_conversion == ANALOG_PIN_CALIB_0 || completed_conversion == ANALOG_PIN_CALIB_1) {
        complete_calibration(completed_conversion);
        completed_conversion = ANALOG_PIN_CALIB_COMPLETE;
        if (running_conversion != ANALOG_PIN_NONE)
            return completed_conversion;

    } else {
    
#if defined(__MK20DX256__)
        pin_map_t info = pin_map[completed_conversion];
        if (info.adc == 0) {
#endif
            v = ADC0_RA;
            ADC0_SC1A = ADC_SC1_ADCH(0b11111);

#if defined(__MK20DX256__)
        } else {
            v = ADC1_RA;
            ADC1_SC1A = ADC_SC1_ADCH(0b11111);
        }
#endif
    }

    // check if new conversion should be started
    if (circ_request_head != circ_request_tail) {
    
        circ_request_tail++;
        if (circ_request_tail >= REQUEST_NUM_BUFFERS)
            circ_request_tail = 0;
        
        uint8_t pin = circ_request_buf[circ_request_tail];
        running_conversion = pin;

        start_conversion(pin);
    } else {
        running_conversion = ANALOG_PIN_NONE;
    }
    // extend value from 10bits to 31bits
    *value = (v << 21) | (v << 11) | (v << 1) | (v >> 9);

    return completed_conversion;
}


void analog_reset()
{
    NVIC_CLEAR_PENDING(IRQ_ADC0);

    ADC0_SC1A = ADC_SC1_ADCH(0b11111); // cancel conversion
    ADC0_SC3 = ADC_SC3_CALF; // cancel calibration

#if defined(__MK20DX256__)
    NVIC_CLEAR_PENDING(IRQ_ADC1);
    ADC1_SC1A = ADC_SC1_ADCH(0b11111); // cancel conversion
    ADC1_SC3 = ADC_SC3_CALF; // cancel calibration
#endif

    is_calibrated = 0;
    running_conversion = ANALOG_PIN_NONE;
    circ_request_head = 0;
    circ_request_tail = 0;
    num_triggers = 0;

    analog_init();
}


void start_calibration(uint8_t pin)
{
#if defined(__MK20DX256__)
    if (pin == ANALOG_PIN_CALIB_0) {
#endif
        ADC0_SC3 = ADC_SC3_CALF;
        ADC0_SC1A = ADC_SC1_AIEN | ADC_SC1_ADCH(0b11111);
        ADC0_SC3 = ADC_SC3_CAL | ADC_SC3_AVGE | ADC_SC3_AVGS(3);
        
#if defined(__MK20DX256__)

    } else {
        ADC1_SC3 = ADC_SC3_CALF;
        ADC1_SC1A = ADC_SC1_AIEN | ADC_SC1_ADCH(0b11111);
        ADC1_SC3 = ADC_SC3_CAL | ADC_SC3_AVGE | ADC_SC3_AVGS(3);
    }
#endif

    running_conversion = pin;
}



void complete_calibration(uint8_t pin)
{
#if defined(__MK20DX256__)
    if (pin == ANALOG_PIN_CALIB_0) {
#endif

        uint16_t v = ADC0_CLP0 + ADC0_CLP1 + ADC0_CLP2 + ADC0_CLP4 + ADC0_CLPS;
        v /= 2;
        v |= 0x8000;
        ADC0_PG = v;

        v = ADC0_CLM0 + ADC0_CLM1 + ADC0_CLM2 + ADC0_CLM4 + ADC0_CLMS;
        v /= 2;
        v |= 0x8000;
        ADC0_MG = v;

        ADC0_SC1A = ADC_SC1_ADCH(0b11111);
        // average 4 samples
        ADC0_SC3 = ADC_SC3_AVGE + ADC_SC3_AVGS(0);

#if defined(__MK20DX256__)
        start_calibration(ANALOG_PIN_CALIB_1);
#else
        running_conversion = ANALOG_PIN_NONE;
        is_calibrated = 1;
#endif
    
#if defined(__MK20DX256__)
    } else {

        uint16_t v = ADC1_CLP0 + ADC1_CLP1 + ADC1_CLP2 + ADC1_CLP4 + ADC1_CLPS;
        v /= 2;
        v |= 0x8000;
        ADC1_PG = v;
    
        v = ADC1_CLM0 + ADC1_CLM1 + ADC1_CLM2 + ADC1_CLM4 + ADC1_CLMS;
        v /= 2;
        v |= 0x8000;
        ADC1_MG = v;

        ADC1_SC1A = ADC_SC1_ADCH(0b11111);
        // average 4 samples
        ADC1_SC3 = ADC_SC3_AVGE + ADC_SC3_AVGS(0);

        running_conversion = ANALOG_PIN_NONE;
        is_calibrated = 1;
    }
#endif
}


void analog_timer_tick()
{
    for (int i = 0; i < num_triggers; i++) {
        uint32_t count = triggers[i].count;
        uint32_t treshold = triggers[i].treshold;
        count++;
        if (count >= treshold)
            count = 0;
        triggers[i].count = count;
        if (count == 0) {
            analog_request_conversion(triggers[i].pin);
        }
    }
}


void insert_trigger(uint8_t pin, uint32_t treshold)
{
    trigger_t* trigger = triggers + num_triggers;
    trigger->treshold = treshold;
    trigger->count = 0;
    trigger->pin = pin;
    num_triggers++;
}


void remove_trigger(uint8_t pin)
{
    int idx;
    for (idx = 0; idx < num_triggers; idx++) {
        if (triggers[idx].pin == pin) {
            for (int i = idx + 1; i < num_triggers; i++) {
                triggers[i - 1] = triggers[i];
            }
            num_triggers--;
        }
    }
}


/*
 * Interrupt handler for analog-to-digital conversions
 */
void adc0_isr()
{
    int32_t value;
    analog_pin pin = analog_get_completed_pin(&value);

    if (pin == ANALOG_PIN_NONE)
        return; // spurious interrupt; don't know why

    if (pin == ANALOG_PIN_CALIB_COMPLETE)
        return;

    wk_send_port_event(PORT_GROUP_ANALOG_IN | pin, WK_EVENT_SINGLE_SAMPLE, 0, value);
}
 
 
void adc1_isr()
{
    adc0_isr();
}
  