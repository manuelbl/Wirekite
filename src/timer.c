/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include "timer.h"
#include "kinetis.h"
#include "yield.h"

volatile uint32_t systick_millis_count = 0;


uint32_t micros(void)
{
	uint32_t count, current, istatus;

	__disable_irq();
	current = SYST_CVR;
	count = systick_millis_count;
	istatus = SCB_ICSR;	// bit 26 indicates if systick exception pending
	__enable_irq();
	if ((istatus & SCB_ICSR_PENDSTSET) && current > 50) count++;
	current = ((F_CPU / 1000) - 1) - current;
	return count * 1000 + ((current * (uint32_t)87381) >> 22);
}

void delay(uint32_t ms)
{
	if (ms <= 0)
		return;

	uint32_t start = micros();

	while (1) {
		while (micros() - start >= 1000) {
			ms--;
			if (ms == 0) return;
			start += 1000;
		}
		yield();
	}
}
