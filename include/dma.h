/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#ifndef __dma_h__
#define __dma_h__

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


#define DMA_NUM_CHANNELS   4

#define DMA_CHANNEL_ERROR 0xff


uint8_t dma_acquire_channel();
void dma_release_channel(uint8_t channel);

void dma_enable(uint8_t channel);
void dma_disable(uint8_t channel);
uint8_t dma_is_complete(uint8_t channel);
void dma_clear_complete(uint8_t channel);
uint8_t dma_is_error(uint8_t channel);
void dma_clear_error(uint8_t channel);
uint32_t dma_bytes_remaining(uint8_t channel);

void dma_attach_interrupt(uint8_t channel, void (*isr)(void));
void dma_detach_interrupt(uint8_t channel);
void dma_clear_interrupt(uint8_t channel);

void dma_trigger_at_hw_evt(uint8_t channel, uint8_t source);
void dma_interrupt_on_completion(uint8_t channel);
void dma_disable_on_completion(uint8_t channel);

void dma_source_byte(uint8_t channel, volatile uint8_t* addr);
void dma_source_byte_buffer(uint8_t channel, const uint8_t* buf, uint16_t len);
void dma_dest_byte(uint8_t channel, volatile uint8_t* addr);
void dma_dest_byte_buffer(uint8_t channel, uint8_t* buf, uint16_t len);


#ifdef __cplusplus
}
#endif

#endif
