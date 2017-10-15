/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __debug_h__
#define __debug_h__

// Switch to enable debugging output
// Alternative: define it on the command line: -D_DEBUG
#ifndef _DEBUG
//#define _DEBUG
#endif

#ifdef _DEBUG

#include "uart.h"
#include "util.h"

#define DEBUG_OUT(s) do { uart0_println(s); } while(0)

#else

#define DEBUG_OUT(s) do { } while(0)

#endif

#endif

