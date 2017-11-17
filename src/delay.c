/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include <string.h>
#include "kinetis.h"
#include "delay.h"
#include "debug.h"


typedef struct {
    uint32_t ticks;
    delay_callback_t callback;
    uint32_t param;
} delay_slot_t;


#define NUM_SLOTS 4

static delay_slot_t slots[NUM_SLOTS];
static uint32_t current_ticks;

#define TIMER_MAX_VALUE 0x7fffffff


static void start_timer();


void delay_init()
{
    SIM_SCGC6 |= SIM_SCGC6_PIT; // enable PIT clocking
    PIT_MCR = 0; // enable periodic timer module
    PIT_TFLG0 = PIT_TFLG_TIF;

    NVIC_CLEAR_PENDING(IRQ_PIT_CH0);
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
}


void delay_reset()
{
    PIT_TFLG0 = PIT_TFLG_TIF;
    NVIC_CLEAR_PENDING(IRQ_PIT_CH0);

    for (int i = 0; i < NUM_SLOTS; i++)
        slots[i].ticks = 0;

    current_ticks = 0;
}


void delay_wait(uint32_t us, delay_callback_t callback, uint32_t param)
{
    uint32_t ticks = us * (F_BUS / 1000000);

    // find unused slot
    for (int i = 0; i < NUM_SLOTS; i++) {
        if (slots[i].ticks == 0) {
            slots[i].ticks = ticks;
            slots[i].callback = callback;
            slots[i].param = param;

            start_timer();
            return;
        }
    }

    DEBUG_OUT("No available timer slot");
}


void pit0_isr()
{
    // add delay since timer interrupt
    uint32_t cval = PIT_CVAL0;
    uint32_t passed_ticks;
    if (cval != 0)
        passed_ticks = TIMER_MAX_VALUE - cval + current_ticks;
    else
        passed_ticks = current_ticks;

repeat:
    for (int i = 0; i < NUM_SLOTS; i++) {
        uint32_t ticks = slots[i].ticks;
        if (ticks != 0) {
            if (ticks <= passed_ticks) {
                slots[i].callback(slots[i].param);
                slots[i].ticks = 0;
                passed_ticks = TIMER_MAX_VALUE - cval + current_ticks;
                goto repeat;
            } else {
                slots[i].ticks = ticks - passed_ticks;
            }
        }
    }

    current_ticks = 0;
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN; // disable timer
    PIT_TFLG0 = PIT_TFLG_TIF; // clear interrupt flag
    start_timer();
}


void start_timer()
{
    if (current_ticks != 0)
        return; // timer already running

    // find shortest delay
    uint32_t min_ticks = TIMER_MAX_VALUE;
    for (int i = 0; i < NUM_SLOTS; i++) {
        uint32_t ticks = slots[i].ticks;
        if (ticks != 0 && ticks < min_ticks)
            min_ticks = ticks;
    }

    if (min_ticks == TIMER_MAX_VALUE)
        return; // nothing to do

    PIT_LDVAL0 = min_ticks;
    PIT_TCTRL0 = PIT_TCTRL_TIE; // enable interrupt
    PIT_TCTRL0 |= PIT_TCTRL_TEN; // start timer
    current_ticks = min_ticks;

    // Load large value for next timer countdown
    // so if the timer isn't immediately served
    // we can calculate the delay.
    PIT_LDVAL0 = TIMER_MAX_VALUE;
}
