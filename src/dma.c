/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */


#include "kinetis.h"
#include "dma.h"

static uint32_t dma_channels_in_use;


#if defined(__MKL26Z64__)
// Teensy LC


typedef struct __attribute__((packed, aligned(4))) {
    volatile const void * volatile SAR;
    volatile void * volatile       DAR;
    volatile uint32_t              DSR_BCR;
    volatile uint32_t              DCR;
} DMA_CFG_t;


static volatile uint8_t* dma_channel_cfg(uint8_t channel);
static DMA_CFG_t* dma_cfg(uint8_t channel);


uint8_t dma_acquire_channel()
{
    if (dma_channels_in_use == 0) {
        SIM_SCGC7 |= SIM_SCGC7_DMA;
        SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
    }

    uint32_t mask = 1;
    for (int channel = 0; channel < DMA_NUM_CHANNELS; channel++) {
        if ((dma_channels_in_use & mask) == 0) {
            dma_channels_in_use |= mask;
            dma_cfg(channel)->DSR_BCR = DMA_DSR_BCR_DONE;
            dma_cfg(channel)->DCR = DMA_DCR_CS;
            dma_cfg(channel)->SAR = NULL;
            dma_cfg(channel)->DAR = NULL;
            return (uint8_t)channel;
        }
        mask <<= 1;
    }

    return DMA_CHANNEL_ERROR;
}


void dma_release_channel(uint8_t channel)
{
    dma_cfg(channel)->DSR_BCR = DMA_DSR_BCR_DONE;
    dma_cfg(channel)->DCR = DMA_DCR_CS;
    dma_cfg(channel)->SAR = NULL;
    dma_cfg(channel)->DAR = NULL;
    *dma_channel_cfg(channel) = DMAMUX_DISABLE;

    uint32_t mask = 1 << channel;
    dma_channels_in_use &= ~mask;

    if (dma_channels_in_use == 0) {
        SIM_SCGC7 &= ~SIM_SCGC7_DMA;
        SIM_SCGC6 &= ~SIM_SCGC6_DMAMUX;
    }
    
}


void dma_enable(uint8_t channel)
{
    dma_cfg(channel)->DCR |= DMA_DCR_ERQ;
}

void dma_disable(uint8_t channel)
{
    dma_cfg(channel)->DCR &= ~DMA_DCR_ERQ;
}

uint8_t dma_is_complete(uint8_t channel)
{
    return (dma_cfg(channel)->DSR_BCR & DMA_DSR_BCR_DONE) != 0;
}

void dma_clear_complete(uint8_t channel)
{
    dma_cfg(channel)->DSR_BCR = DMA_DSR_BCR_DONE;
}

uint8_t dma_is_error(uint8_t channel)
{
    return (dma_cfg(channel)->DSR_BCR & (DMA_DSR_BCR_CE | DMA_DSR_BCR_BES | DMA_DSR_BCR_BED)) != 0;
}

void dma_clear_error(uint8_t channel)
{
    dma_cfg(channel)->DSR_BCR = DMA_DSR_BCR_DONE;
}

uint32_t dma_bytes_remaining(uint8_t channel)
{
    return DMA_DSR_BCR_BCR(dma_cfg(channel)->DSR_BCR);
}

void dma_attach_interrupt(uint8_t channel, void (*isr)(void)) {
    _VectorsRam[IRQ_DMA_CH0 + 16 + channel] = isr;
    NVIC_ENABLE_IRQ(IRQ_DMA_CH0 + channel);
}


void dma_detach_interrupt(uint8_t channel)
{
    NVIC_DISABLE_IRQ(IRQ_DMA_CH0 + channel);
}


void dma_clear_interrupt(uint8_t channel)
{
    dma_cfg(channel)->DSR_BCR = DMA_DSR_BCR_DONE;
}


void dma_interrupt_on_completion(uint8_t channel)
{
    dma_cfg(channel)->DCR |= DMA_DCR_EINT;
}


void dma_disable_on_completion(uint8_t channel)
{
    dma_cfg(channel)->DCR |= DMA_DCR_D_REQ;
}


void dma_trigger_at_hw_evt(uint8_t channel, uint8_t source)
{
    dma_cfg(channel)->DCR |= DMA_DCR_CS;
    volatile uint8_t* chcfg = dma_channel_cfg(channel);
    *chcfg = DMAMUX_DISABLE;
    *chcfg = source | DMAMUX_ENABLE;
}


void dma_source_byte(uint8_t channel, volatile uint8_t* addr)
{
    DMA_CFG_t* cfg = dma_cfg(channel);
    cfg->SAR = addr;
    cfg->DCR = (cfg->DCR & 0xF08E0FFF) | DMA_DCR_SSIZE(1);
}


void dma_source_byte_buffer(uint8_t channel, const uint8_t* buf, uint16_t len)
{
    DMA_CFG_t* cfg = dma_cfg(channel);
    cfg->SAR = buf;
    cfg->DCR = (cfg->DCR & 0xF08E0FFF) | DMA_DCR_SSIZE(1) | DMA_DCR_SINC;
    cfg->DSR_BCR = len;
}


void dma_dest_byte(uint8_t channel, volatile uint8_t* addr)
{
    DMA_CFG_t* cfg = dma_cfg(channel);
    cfg->DAR = addr;
    cfg->DCR = (cfg->DCR & 0xF0F0F0FF) | DMA_DCR_DSIZE(1);
}


void dma_dest_byte_buffer(uint8_t channel, uint8_t* buf, uint16_t len)
{
    DMA_CFG_t* cfg = dma_cfg(channel);
    cfg->DAR = buf;
    cfg->DCR = (cfg->DCR & 0xF0F0F0FF) | DMA_DCR_DSIZE(1) | DMA_DCR_DINC;
    cfg->DSR_BCR = len;
}


volatile uint8_t* dma_channel_cfg(uint8_t channel)
{
    return &DMAMUX0_CHCFG0 + channel;
}


DMA_CFG_t* dma_cfg(uint8_t channel)
{
    return ((DMA_CFG_t*)&DMA_SAR0) + channel;
}


#elif defined(__MK20DX256__)
// Teensy 3.2


uint8_t dma_acquire_channel()
{
    return DMA_CHANNEL_ERROR;
}


void dma_release_channel(uint8_t channel)
{
}


void dma_enable(uint8_t channel)
{
}

void dma_disable(uint8_t channel)
{
}

uint8_t dma_is_complete(uint8_t channel)
{
    return 0;
}

void dma_clear_complete(uint8_t channel)
{
}

uint8_t dma_is_error(uint8_t channel)
{
    return 0;
}

void dma_clear_error(uint8_t channel)
{
}

uint32_t dma_bytes_remaining(uint8_t channel)
{
    return 0;
}

void dma_attach_interrupt(uint8_t channel, void (*isr)(void)) {
}


void dma_detach_interrupt(uint8_t channel)
{
}


void dma_clear_interrupt(uint8_t channel)
{
}


void dma_interrupt_on_completion(uint8_t channel)
{
}


void dma_disable_on_completion(uint8_t channel)
{
}


void dma_trigger_at_hw_evt(uint8_t channel, uint8_t source)
{
}


void dma_source_byte(uint8_t channel, volatile uint8_t* addr)
{
}


void dma_source_byte_buffer(uint8_t channel, const uint8_t* buf, uint16_t len)
{
}


void dma_dest_byte(uint8_t channel, volatile uint8_t* addr)
{
}


void dma_dest_byte_buffer(uint8_t channel, uint8_t* buf, uint16_t len)
{
}


#endif
