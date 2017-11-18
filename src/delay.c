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

#define STATE_IDLE 0
#define STATE_COUNTDOWN 1
#define STATE_CALLBACKS 2


static delay_slot_t slots[NUM_SLOTS];
static uint32_t countdown_ticks;
static int delay_state;

#define TIMER_MAX_VALUE 0xffffffff


static void start_timer();
static int serve_callbacks(uint32_t expired_ticks);


void delay_init()
{
    SIM_SCGC6 |= SIM_SCGC6_PIT; // enable PIT clocking
    PIT_MCR = 0; // enable periodic timer module
    PIT_TFLG0 = PIT_TFLG_TIF;

#if defined(__MKL26Z64__)
    NVIC_CLEAR_PENDING(IRQ_PIT);
    NVIC_ENABLE_IRQ(IRQ_PIT);
#elif defined(__MK20DX256__)
    NVIC_CLEAR_PENDING(IRQ_PIT_CH0);
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
#endif
}


void delay_reset()
{
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN; // disable timer
    PIT_TFLG0 = PIT_TFLG_TIF; // clear flag

#if defined(__MKL26Z64__)
    NVIC_CLEAR_PENDING(IRQ_PIT);
#elif defined(__MK20DX256__)
    NVIC_CLEAR_PENDING(IRQ_PIT_CH0);
#endif

    for (int i = 0; i < NUM_SLOTS; i++)
        slots[i].ticks = 0;

    countdown_ticks = 0;
    delay_state = STATE_IDLE;
}


void delay_wait(uint32_t us, delay_callback_t callback, uint32_t param)
{
    if (delay_state == STATE_COUNTDOWN) {
        // stop timer and recalculate remaining time
        PIT_TCTRL0 &= ~PIT_TCTRL_TEN; // disable timer
        uint32_t expired_ticks;
        if ((PIT_TFLG0 & PIT_TFLG_TIF) == 0)
            expired_ticks = countdown_ticks - PIT_CVAL0;
        else
            expired_ticks = TIMER_MAX_VALUE - PIT_CVAL0 + countdown_ticks;
        PIT_TFLG0 = PIT_TFLG_TIF; // clear interrupt flag

        delay_state = STATE_CALLBACKS;
        serve_callbacks(expired_ticks);
        delay_state = STATE_IDLE;
    }

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
    start_timer();
}


int serve_callbacks(uint32_t expired_ticks)
{
    int served = 0;

    // mark expired delays so we can distinguish them from
    // new delays that are inserted from callbacks
    for (int i = 0; i < NUM_SLOTS; i++) {
        uint32_t ticks = slots[i].ticks;
        if (ticks != 0) {
            if (ticks <= expired_ticks) {
                slots[i].ticks = TIMER_MAX_VALUE;
                served = 1;
            } else {
                slots[i].ticks -= expired_ticks;
            }
        }
    }

    // call callback for expired delays
    for (int i = 0; i < NUM_SLOTS; i++) {
        if (slots[i].ticks == TIMER_MAX_VALUE) {
            slots[i].ticks = 0;
            slots[i].callback(slots[i].param);
        }
    }

    return served;
}


void pit0_isr()
{
    delay_state = STATE_CALLBACKS;

    // Compute ticks since delayed exuection was posted
    // incl. delay since timer interrupt occurred
    uint32_t cval = PIT_CVAL0;
    uint32_t expired_ticks = TIMER_MAX_VALUE - cval + countdown_ticks;

    int served;
    do {
        served = serve_callbacks(expired_ticks);
        uint32_t cv = PIT_CVAL0;
        expired_ticks = cv - cval;
        cval = cv;
    } while (served != 0);

    PIT_TCTRL0 &= ~PIT_TCTRL_TEN; // disable timer
    PIT_TFLG0 = PIT_TFLG_TIF; // clear interrupt flag
    delay_state = STATE_IDLE;

    start_timer();
}


#if defined(__MKL26Z64__)

void pit_isr()
{
    if (PIT_TFLG0 & PIT_TFLG_TIF)
        pit0_isr();
}

#endif


void start_timer()
{
    if (delay_state != STATE_IDLE)
        return; // timer already running or serving callbacks

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
    countdown_ticks = min_ticks;
    delay_state = STATE_COUNTDOWN;

    // Load large value for next timer countdown
    // so if the timer isn't immediately served
    // we can calculate the delay.
    PIT_LDVAL0 = TIMER_MAX_VALUE;
}
