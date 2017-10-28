/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __ports_h__
#define __ports_h__

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t* const PCR_ADDR[];

#define PCR(port, pin) (*(PCR_ADDR[port] + pin))
#define PCR_PTR(port, pin) ((PCR_ADDR[port] + pin))

typedef struct __attribute__((packed, aligned(4))) {
    volatile uint32_t PDOR; // Port Data Output Register
    volatile uint32_t PSOR; // Port Set Output Register
    volatile uint32_t PCOR; // Port Clear Output Register
    volatile uint32_t PTOR; // Port Toggle Output Register
    volatile uint32_t PDIR; // Port Data Input Register
    volatile uint32_t PDDR; // Port Data Direction Register
} GPIO_PORT_t;

extern GPIO_PORT_t* const GPIO_PORT[];


#ifdef __cplusplus
}
#endif


#endif

