/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include "yield.h"

#define YIELD_TAB_SIZE 8
static volatile yield_func_t yield_tab[YIELD_TAB_SIZE];
static volatile uint8_t yield_tab_size = 0;

void yield(void)
{
    static uint8_t is_yielding = 0;

    if (is_yielding)
        return;
    is_yielding = 1;

    for (uint8_t i = 0; i < yield_tab_size; i++)
        yield_tab[i]();

    is_yielding = 0;
}


void yield_add_func(yield_func_t f)
{
    yield_tab[yield_tab_size] = f;
    yield_tab_size++;
}


void yield_remove_func(yield_func_t f)
{
    uint8_t idx = yield_tab_size;
    for (uint8_t i = 0; i < yield_tab_size; i++) {
        if (yield_tab[i] == f) {
            idx = i;
            break;
        }
    }

    if (idx < yield_tab_size)
        yield_tab_size--;

    for (uint8_t i = idx; i < yield_tab_size; i++) {
        yield_tab[i] = yield_tab[i + 1];
    }

}