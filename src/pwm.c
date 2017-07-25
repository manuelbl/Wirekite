/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include <string.h>
#include "pwm.h"
#include "kinetis.h"
#include "pwm_config.h"
#include "uart.h"


#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3
#define PORT_E 4
#define PORT_ENC(t)     (t << 4)
#define PORT_IDX(n)     ((n >> 4) & 0x7)

#define PIN_IDX_ENC(n)  | n
#define PIN_IDX(n)      (n & 0xf)

#define ALT_ENC(a)      | (a << 8)
#define ALT(n)          ((n >> 8) & 0x7)

#define TIMER_IDX_ENC(t)    | (t << 11)
#define TIMER_IDX(n)        ((n >> 11) & 0x3)

#define CHAN_IDX_ENC(c)     | (c << 13)
#define CHAN_IDX(n)         ((n >> 13) & 0x7)

static uint16_t pin_map[] = {
    PORT_ENC(PORT_A)  PIN_IDX_ENC(1)  ALT_ENC(3)  TIMER_IDX_ENC(2)  CHAN_IDX_ENC(0),   //  Pin 3    PTA1    TPM2_CH0
    PORT_ENC(PORT_A)  PIN_IDX_ENC(2)  ALT_ENC(3)  TIMER_IDX_ENC(2)  CHAN_IDX_ENC(1),   //  Pin 4    PTA2    TPM2_CH1
    PORT_ENC(PORT_D)  PIN_IDX_ENC(4)  ALT_ENC(4)  TIMER_IDX_ENC(0)  CHAN_IDX_ENC(4),   //  Pin 6    PTD4    TPM0_CH4
    PORT_ENC(PORT_C)  PIN_IDX_ENC(3)  ALT_ENC(4)  TIMER_IDX_ENC(0)  CHAN_IDX_ENC(2),   //  Pin 9    PTC3    TPM0_CH2
    PORT_ENC(PORT_C)  PIN_IDX_ENC(4)  ALT_ENC(4)  TIMER_IDX_ENC(0)  CHAN_IDX_ENC(3),   //  Pin 10   PTC4    TPM0_CH3
    PORT_ENC(PORT_B)  PIN_IDX_ENC(0)  ALT_ENC(3)  TIMER_IDX_ENC(1)  CHAN_IDX_ENC(0),   //  Pin 16   PTB0    TPM1_CH0
    PORT_ENC(PORT_B)  PIN_IDX_ENC(1)  ALT_ENC(3)  TIMER_IDX_ENC(1)  CHAN_IDX_ENC(1),   //  Pin 17   PTB1    TPM1_CH1
    PORT_ENC(PORT_D)  PIN_IDX_ENC(5)  ALT_ENC(4)  TIMER_IDX_ENC(0)  CHAN_IDX_ENC(5),   //  Pin 20   PTD5    TPM0_CH5
    PORT_ENC(PORT_C)  PIN_IDX_ENC(1)  ALT_ENC(4)  TIMER_IDX_ENC(0)  CHAN_IDX_ENC(0),   //  Pin 22   PTC1    TPM0_CH0
    PORT_ENC(PORT_C)  PIN_IDX_ENC(2)  ALT_ENC(4)  TIMER_IDX_ENC(0)  CHAN_IDX_ENC(1)    //  Pin 23   PTC2    TPM0_CH1
};

#define NUM_PINS (sizeof(pin_map)/sizeof(pin_map[0]))


static volatile uint32_t* TPM_BASE_ADDR[] = {
    &TPM0_SC,
    &TPM1_SC,
    &TPM2_SC
};

#define NUM_TIMERS (sizeof(TPM_BASE_ADDR)/sizeof(TPM_BASE_ADDR[0]))


static volatile uint32_t* PCR_ADDR[] = {
    &PORTA_PCR0,
    &PORTB_PCR0,
    &PORTC_PCR0,
    &PORTD_PCR0,
    &PORTE_PCR0
};

static uint16_t modulos[] = { 0, 0, 0};
static int16_t  values[NUM_PINS];


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

    // Compute prescale factor and modulus.
    // Select the lowest prescale factor
    // given the condition that modulus must be <= 65536
    uint8_t prescale = 0;
    uint32_t mod = (F_TIMER + (frequency >> 1)) / frequency;
    while (mod > 65536) {
        prescale++;
        mod >>= 1;
    }
    if (mod == 0)
        mod = 1;
    uint16_t modulo = mod - 1;
    modulos[timer] = modulo;

    uint32_t sc = FTM_SC_CLKS(1) | FTM_SC_PS(prescale);
    if (attributes & PWM_TIMER_ATTR_CENTER)
        sc |= FTM_SC_CPWMS;

    volatile uint32_t* tpm_base = TPM_BASE_ADDR[timer];
    *(tpm_base + 0) = 0; // TPMx_SC
    *(tpm_base + 1) = 0; // TPMx_CNT
    *(tpm_base + 2) = modulo; // TPMx_MOD
    *(tpm_base + 0) = sc; // TPMx_SC

    // recalculate values
    for (int i = 0; i < NUM_PINS; i++) {
        if (values[i] != -1 && TIMER_IDX(pin_map[i]) == timer)
            pwm_pin_set_value(i, values[i]);
    }
}


void pwm_channel_config(uint8_t timer, uint8_t channel, uint16_t attributes)
{
    if (timer >= NUM_TIMERS)
        return;
    if ((timer == 0 && channel >= 6) || (timer != 0 && channel >= 2))
        return;

    uint32_t sc = (attributes & PWM_CHAN_ATTR_LOW_PULSE) ? 0b00100100 : 0b00101000;
    volatile uint32_t* tpm_base = TPM_BASE_ADDR[timer];
    *(tpm_base + 3 + channel * 2) = 0; // TPMx_CnSC
    *(tpm_base + 3 + channel * 2) = sc; // TPMx_CnSC
}


pwm_pin pwm_pin_init(uint8_t pin)
{
    if (pin >= NUM_PINS)
        return PWM_PIN_ERROR;

    pwm_pin_set_value(pin, 0);

    uint16_t pin_info = pin_map[pin];
    *(PCR_ADDR[PORT_IDX(pin_info)] + PIN_IDX(pin_info)) = PORT_PCR_MUX(ALT(pin_info)) | PORT_PCR_DSE | PORT_PCR_SRE;

    return pin;
}


void pwm_pin_release(pwm_pin port_id)
{
    if (port_id >= NUM_PINS)
        return;
        
    pwm_pin_set_value(port_id, 0);
    values[port_id] = -1;

    uint16_t pin_info = pin_map[port_id];
    *(PCR_ADDR[PORT_IDX(pin_info)] + PIN_IDX(pin_info)) = 0;
}


void pwm_pin_set_value(pwm_pin port_id, int16_t value)
{
    if (port_id >= NUM_PINS)
        return;

    uint16_t pin_info = pin_map[port_id];

    // counter counts up to modulo + 1
    // input value is between 0 (0% duty cycle) and 32767 (100% duty cycle)
    uint32_t cval = 0;
    if (value != 0) {
        uint16_t modulo = modulos[TIMER_IDX(pin_info)];
        cval = ((uint32_t)value) * (((uint32_t)modulo) + 1) / 32767;
    }
    values[port_id] = cval;

    volatile uint32_t* value_ptr = TPM_BASE_ADDR[TIMER_IDX(pin_info)] + 4 + 2 * CHAN_IDX(pin_info);
    *value_ptr = cval;
}


void pwm_reset()
{
    for (int i = 0; i < NUM_PINS; i++)
        if (values[i] != -1)
            pwm_pin_release(i);
}
