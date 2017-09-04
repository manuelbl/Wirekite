/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include <string.h>
#include "pwm.h"
#include "kinetis.h"
#include "pwm_config.h"
#include "debug.h"


#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3
#define PORT_E 4

typedef struct {
    uint16_t port : 3;
    uint16_t pin : 5;
    uint16_t alt : 3;
    uint16_t timer: 2;
    uint16_t channel : 3;
    uint8_t  digi_pin;
} pin_map_t;


#if defined(__MKL26Z64__)

// PWM pin map of Teensy LC
static pin_map_t pin_map[] = {
    { PORT_A,  1, 3, 2, 0,  3 },   //  Pin 3    PTA1    TPM2_CH0
    { PORT_A,  2, 3, 2, 1,  4 },   //  Pin 4    PTA2    TPM2_CH1
    { PORT_D,  4, 4, 0, 4,  6 },   //  Pin 6    PTD4    TPM0_CH4
    { PORT_C,  3, 4, 0, 2,  9 },   //  Pin 9    PTC3    TPM0_CH2
    { PORT_C,  4, 4, 0, 3, 10 },   //  Pin 10   PTC4    TPM0_CH3
    { PORT_B,  0, 3, 1, 0, 16 },   //  Pin 16   PTB0    TPM1_CH0
    { PORT_B,  1, 3, 1, 1, 17 },   //  Pin 17   PTB1    TPM1_CH1
    { PORT_D,  5, 4, 0, 5, 20 },   //  Pin 20   PTD5    TPM0_CH5
    { PORT_C,  1, 4, 0, 0, 22 },   //  Pin 22   PTC1    TPM0_CH0
    { PORT_C,  2, 4, 0, 1, 23 }    //  Pin 23   PTC2    TPM0_CH1
};

#elif defined(__MK20DX256__)

// PWM pin map of Teensy 3.2
static pin_map_t pin_map[] = {
    { PORT_A, 12, 3, 1, 0,  3 },   //  Pin 3    PTA12    FTM1_CH0
    { PORT_A, 13, 3, 1, 1,  4 },   //  Pin 4    PTA13    FTM1_CH1
    { PORT_D,  7, 4, 0, 7,  5 },   //  Pin 5    PTD7    FTM0_CH7
    { PORT_D,  4, 4, 0, 4,  6 },   //  Pin 6    PTD4    FTM0_CH4
    { PORT_C,  3, 4, 0, 2,  9 },   //  Pin 9    PTC3    FTM0_CH2
    { PORT_C,  4, 4, 0, 3, 10 },   //  Pin 10   PTC4    FTM0_CH3
    { PORT_D,  5, 4, 0, 5, 20 },   //  Pin 20   PTD5    FTM0_CH5
    { PORT_D,  6, 4, 0, 6, 21 },   //  Pin 21   PTD6    FTM0_CH6
    { PORT_C,  1, 4, 0, 0, 22 },   //  Pin 22   PTC1    FTM0_CH0
    { PORT_C,  2, 4, 0, 1, 23 },   //  Pin 23   PTC2    FTM0_CH1
    { PORT_B, 19, 3, 2, 1, 25 },   //  Pin 25   PTB19   FTM2_CH1
    { PORT_B, 18, 3, 2, 0, 32 }    //  Pin 32   PTB18   FTM2_CH0
};

#endif


#define NUM_PINS (sizeof(pin_map)/sizeof(pin_map[0]))

typedef struct __attribute__((packed, aligned(4))) {
    volatile uint32_t SC; // Channel x Status And Control
    volatile uint32_t V;  // Channel x Value
} FTM_CHAN_t;

typedef struct __attribute__((packed, aligned(4))) {
    volatile uint32_t SC;   // Status And Control
    volatile uint32_t CNT;  // Counter
    volatile uint32_t MOD;  // Modulo
    FTM_CHAN_t channel[8];
} FTM_t;

#if defined(__MKL26Z64__)
// Teensy LC timers

static FTM_t* FTM[] = {
    (FTM_t*)&TPM0_SC,
    (FTM_t*)&TPM1_SC,
    (FTM_t*)&TPM2_SC
};

#elif defined(__MK20DX256__)
// Teensy 3.2 timers

static FTM_t* FTM[] = {
    (FTM_t*)&FTM0_SC,
    (FTM_t*)&FTM1_SC,
    (FTM_t*)&FTM2_SC,
    (FTM_t*)&FTM3_SC
};

#endif

#define NUM_TIMERS (sizeof(FTM)/sizeof(FTM[0]))


static volatile uint32_t* PCR_ADDR[] = {
    &PORTA_PCR0,
    &PORTB_PCR0,
    &PORTC_PCR0,
    &PORTD_PCR0,
    &PORTE_PCR0
};

#define PCR(map) (*(PCR_ADDR[map.port] + map.pin))

static uint16_t modulos[NUM_TIMERS];
static int32_t  values[NUM_PINS];


void pwm_init()
{
    memset(values, 0xff, sizeof(values));

    pwm_timer_config(0, 488, 0);
    pwm_channel_config(0, 0, 0);
    pwm_channel_config(0, 1, 0);
    pwm_channel_config(0, 2, 0);
    pwm_channel_config(0, 3, 0);
    pwm_channel_config(0, 4, 0);
    pwm_channel_config(0, 5, 0);
#if defined(__MK20DX256__)
    pwm_channel_config(0, 6, 0);
    pwm_channel_config(0, 7, 0);
#endif

    pwm_timer_config(1, 488, 0);
    pwm_channel_config(1, 0, 0);
    pwm_channel_config(1, 1, 0);

    pwm_timer_config(2, 488, 0);
    pwm_channel_config(2, 0, 0);
    pwm_channel_config(2, 1, 0);
}


void pwm_timer_config(uint8_t timer, uint32_t frequency, uint16_t attributes)
{
    if (timer >= NUM_TIMERS)
        return;
    
    int clockSource;
    int clock;

#if defined(__MKL26Z64__)

    clockSource = 1;
    clock = F_TIMER;

#elif defined(__MK20DX256__)

    if (frequency < (F_TIMER >> 7) / 65536) {
        clockSource = 2;
        clock = 31250;
    } else {
        clockSource = 1;
        clock = F_TIMER;
    }

#endif

    // Compute prescale factor and modulus.
    // Select the lowest prescale factor
    // given the condition that modulus must be <= 65536
    uint8_t prescale = 0;
    uint32_t mod = (clock + (frequency >> 1)) / frequency;
    while (mod > 65536) {
        prescale++;
        mod >>= 1;
    }
    if (mod == 0)
        mod = 1;
    uint16_t modulo = mod - 1;
    modulos[timer] = modulo;

    uint32_t sc = FTM_SC_CLKS(clockSource) | FTM_SC_PS(prescale);
    if (attributes & PWM_TIMER_ATTR_CENTER)
        sc |= FTM_SC_CPWMS;

    FTM_t* ftm = FTM[timer];
    ftm->SC = 0;
    ftm->CNT = 0;
    ftm->MOD = modulo;
    ftm->SC = sc;

    // recalculate values
    for (int i = 0; i < NUM_PINS; i++) {
        if (values[i] != -1 && pin_map[i].timer == timer)
            pwm_pin_set_value(i, values[i]);
    }
}


void pwm_channel_config(uint8_t timer, uint8_t channel, uint16_t attributes)
{
    if (timer >= NUM_TIMERS)
        return;

#if defined(__MKL26Z64__)        
    if ((timer == 0 && channel >= 6) || (timer != 0 && channel >= 2))
        return;
#elif defined(__MK20DX256__)
    if ((timer == 0 && channel >= 8) || (timer != 0 && channel >= 2))
        return;
#endif

    uint32_t sc = (attributes & PWM_CHAN_ATTR_LOW_PULSE) ? 0b00100100 : 0b00101000;
    FTM_t* ftm = FTM[timer];
    ftm->channel[channel].SC = 0;
    ftm->channel[channel].SC = sc;
}


pwm_pin pwm_pin_init(uint8_t digi_pin, int32_t initial_value)
{
    pwm_pin pin = PWM_PIN_ERROR;
    for (int i = 0; i < NUM_PINS; i++) {
        if (pin_map[i].digi_pin == digi_pin) {
            pin = (pwm_pin)i;
            break;
        }
    }
    if (pin == PWM_PIN_ERROR)
        return pin;

    pwm_pin_set_value(pin, initial_value);

    pin_map_t map = pin_map[pin];
    PCR(map) = PORT_PCR_MUX(map.alt) | PORT_PCR_SRE;

    return pin;
}


void pwm_pin_release(pwm_pin port_id)
{
    if (port_id >= NUM_PINS)
        return;

    pin_map_t map = pin_map[port_id];
    PCR(map) = 0;
        
    pwm_pin_set_value(port_id, 0);
    values[port_id] = -1;
}


void pwm_pin_set_value(pwm_pin port_id, int32_t value)
{
    if (port_id >= NUM_PINS)
        return;

    pin_map_t map = pin_map[port_id];
    values[port_id] = value;
    
    // counter counts up to modulo + 1
    // input value is between 0 (0% duty cycle) and 2'147'483'647 (100% duty cycle)
    value >>= 16;
    // input value is between 0 (0% duty cycle) and 32'767 (100% duty cycle)
    uint32_t cval = 0;
    if (value != 0) {
        uint16_t modulo = modulos[map.timer];
        cval = value * (modulo + 1) / 32767;
    }

    FTM[map.timer]->channel[map.channel].V = cval;
}


void pwm_reset()
{
    for (int i = 0; i < NUM_PINS; i++)
        if (values[i] != -1)
            pwm_pin_release(i);
}
