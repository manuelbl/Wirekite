/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */


#include <string.h>
#include "kinetis.h"

#include "spi.h"
#include "digital_pin.h"
#include "dma.h"
#include "mem.h"
#include "usb.h"
#include "wirekite.h"
#include "debug.h"

#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3
#define PORT_E 4


typedef struct {
    uint32_t t_pin : 6;
    uint32_t port : 3;
    uint32_t pin : 5;
    uint32_t alt : 3;
    uint32_t spi : 2;
} port_map_t;


static const port_map_t SCK_map[] = {
    { 13, PORT_C,  5, 2, 0 }, // PTC5
    { 14, PORT_D,  1, 2, 0 }, // PTD1
    { 20, PORT_D,  5, 2, 1 }  // PTD5
};

#define NUM_SCK (sizeof(SCK_map)/sizeof(SCK_map[0]))


static const port_map_t MOSI_map[] = {
    { 11, PORT_C,  6, 2, 0 }, // PTC6
    {  7, PORT_D,  2, 2, 0 }, // PTD2
    {  0, PORT_B, 16, 2, 1 }, // PTB16
    { 21, PORT_D,  6, 2, 1 }  // PTD6
};    

#define NUM_MOSI (sizeof(MOSI_map)/sizeof(MOSI_map[0]))


static const port_map_t MISO_map[] = {
    { 12, PORT_C,  7, 2, 0 }, // PTC7
    {  8, PORT_D,  3, 2, 0 }, // PTD3
    {  1, PORT_B, 17, 2, 1 }, // PTB17
    {  5, PORT_D,  7, 2, 1 }  // PTD7
};    

#define NUM_MISO (sizeof(MISO_map)/sizeof(MISO_map[0]))


typedef struct {
    uint8_t f_div;
    uint16_t divider;
} freq_div_t;


static volatile uint32_t* PCR_ADDR[] = {
    &PORTA_PCR0,
    &PORTB_PCR0,
    &PORTC_PCR0,
    &PORTD_PCR0,
    &PORTE_PCR0
};

#define PCR(port, pin) (*(PCR_ADDR[port] + pin))


static const freq_div_t freq_divs[] = {
    { 0x0, 3 },
    { 0x1, 5 },
    { 0x20, 7 },
    { 0x2, 9 },
    { 0x40, 11 },
    { 0x21, 13 },
    { 0x60, 15 },
    { 0x3, 18 },
    { 0x41, 22 },
    { 0x22, 26 },
    { 0x61, 30 },
    { 0x4, 36 },
    { 0x42, 44 },
    { 0x23, 52 },
    { 0x62, 60 },
    { 0x5, 72 },
    { 0x43, 88 },
    { 0x24, 104 },
    { 0x63, 120 },
    { 0x6, 143 },
    { 0x44, 175 },
    { 0x25, 207 },
    { 0x64, 239 },
    { 0x7, 286 },
    { 0x45, 351 },
    { 0x26, 415 },
    { 0x65, 479 },
    { 0x8, 572 },
    { 0x46, 701 },
    { 0x27, 830 },
    { 0x66, 958 },
    { 0x18, 1145 },
    { 0x47, 1402 },
    { 0x28, 1659 },
    { 0x67, 1916 },
    { 0x38, 2290 },
    { 0x48, 2804 },
    { 0x58, 3318 },
    { 0x68, 3831 },
    { 0x78, 0 }
};

#define NUM_F_DIVS (sizeof(freq_divs)/sizeof(freq_divs[0]))


#define STATE_INACTIVE 0
#define STATE_WAITING  1
#define STATE_TX 2
#define STATE_RX 3
#define STATE_ERROR 4

#define SUB_STATE_IDLE 0
#define SUB_STATE_DATA 1
#define SUB_STATE_DMA 2
#define SUB_STATE_DMA_LAST_BYTE 3


#define CIRC_QUEUE_SIZE 20


typedef struct {
    union {
        wk_port_request* request;
        wk_port_event* response;
    };
    uint8_t state;
    uint8_t sub_state;
    uint16_t processed;
    uint8_t circ_request_head;
    uint8_t circ_request_tail;
    uint8_t dma_tx;
    uint8_t dma_rx;
    uint8_t sck;
    uint8_t mosi;
    uint8_t miso;
    wk_port_request* circ_request_buf[CIRC_QUEUE_SIZE];
} spi_port_info_t;

#define NUM_SPI_PORTS 2

static spi_port_info_t port_info[NUM_SPI_PORTS];


static KINETISL_SPI_t* get_spi_ctrl(spi_port port);
static void dma_spi0_tx_isr();
static void dma_spi0_rx_isr();
static void dma_spi1_tx_isr();
static void dma_spi1_rx_isr();
static void set_frequency(KINETISL_SPI_t* spi, uint32_t bus_rate, uint32_t frequency);
static void master_start_send_2(wk_port_request* request);
static void set_digital_output_2(wk_port_request* request);
static void write_complete(spi_port port, uint8_t status, uint16_t len);
static wk_port_request* get_next_request(spi_port port);
static void append_request(spi_port port, wk_port_request* request);
static void check_queue(spi_port port);
static void clear_request_queue(spi_port port);
static void spi_isr_handler(uint8_t port);


void spi_init()
{
    // nothing to do
}

void spi_reset()
{
    for (int i = 0; i < NUM_SPI_PORTS; i++)
        if (port_info[i].state != STATE_INACTIVE)
            spi_port_release(i);

    SIM_SCGC4 &= ~(SIM_SCGC4_SPI0 | SIM_SCGC4_SPI1);
}

spi_port spi_master_init(uint16_t sck_mosi, uint16_t miso, uint16_t attributes, uint32_t frequency)
{
    // resolve SCK
    uint8_t sck_pin = sck_mosi & 0xff;
    uint8_t sck_port = 0xff;
    for (int i = 0; i < NUM_SCK; i++) {
        if (SCK_map[i].t_pin == sck_pin)
        sck_port = i;
    }

    if (sck_port == 0xff)
        return SPI_PORT_ERROR;

    uint8_t spi_port = SCK_map[sck_port].spi;

    // resolve MOSI
    uint8_t mosi_pin = (sck_mosi >> 8) & 0xff;
    uint8_t mosi_port = 0xff;
    for (int i = 0; i < NUM_MOSI; i++) {
        if (MOSI_map[i].t_pin == mosi_pin)
        mosi_port = i;
    }

    if (mosi_port == 0xff)
        return SPI_PORT_ERROR;
    if (spi_port != MOSI_map[mosi_port].spi)
        return SPI_PORT_ERROR;

    // resolve optional MISO
    uint8_t miso_pin = miso & 0xff;
    uint8_t miso_port = 0xff;
    if (miso_pin != 0xff) {
        for (int i = 0; i < NUM_MISO; i++) {
            if (MISO_map[i].t_pin == miso_pin)
                miso_port = i;
        }

        if (miso_port == 0xff)
            return SPI_PORT_ERROR;
        if (spi_port != MISO_map[miso_port].spi)
            return SPI_PORT_ERROR;
    }

    spi_port_info_t* pi = &port_info[spi_port];
    if (pi->state != STATE_INACTIVE)
        return SPI_PORT_ERROR;
    
    // initialize state
    pi->state = STATE_WAITING;
    pi->sck = sck_port;
    pi->mosi = mosi_port;
    pi->miso = miso_port;

    // enable clock
    if (spi_port == 0) {
        SIM_SCGC4 |= SIM_SCGC4_SPI0;
    } else if (spi_port == 1) {
        SIM_SCGC4 |= SIM_SCGC4_SPI1;
    }

    // initialize SPI module
    KINETISL_SPI_t* spi = get_spi_ctrl(spi_port);

    uint8_t c1 = SPI_C1_MSTR | SPI_C1_SPE;
    if ((attributes & SPI_CONFIG_MODE_MASK) == SPI_CONFIG_MODE2 || (attributes & SPI_CONFIG_MODE_MASK) == SPI_CONFIG_MODE3)
        c1 |= SPI_C1_CPOL;
    if ((attributes & SPI_CONFIG_MODE_MASK) == SPI_CONFIG_MODE1 || (attributes & SPI_CONFIG_MODE_MASK) == SPI_CONFIG_MODE3)
        c1 |= SPI_C1_CPHA;    
    spi->C1 = c1;

    uint8_t c2 = 0;
    if ((attributes & SPI_CONFIG_BITS_MASK) == SPI_CONFIG_16_BIT)
        c2 |= SPI_C2_SPIMODE;
    spi->C2 = c2;

    set_frequency(spi, spi_port == 0 ? F_BUS : F_CPU, frequency);
    
    // configure SCK
    PCR(SCK_map[sck_port].port, SCK_map[sck_port].pin) = PORT_PCR_MUX(SCK_map[sck_port].alt) | PORT_PCR_DSE;
    
    // configure MOSI
    PCR(MOSI_map[mosi_port].port, MOSI_map[mosi_port].pin) = PORT_PCR_MUX(MOSI_map[mosi_port].alt) | PORT_PCR_DSE;
    
    // configure MISO
    if (miso_port != 0xff)
        PCR(MISO_map[miso_port].port, MISO_map[miso_port].pin) = PORT_PCR_MUX(MISO_map[miso_port].alt);
    
    // Enable interrupt
    NVIC_ENABLE_IRQ(spi_port == 0 ? IRQ_SPI0 : IRQ_SPI1);

    // configure DMAs
    uint8_t dma_tx = dma_acquire_channel();
    pi->dma_tx = dma_tx;
    if (dma_tx != DMA_CHANNEL_ERROR) {
        dma_disable_on_completion(dma_tx);
        dma_attach_interrupt(dma_tx, spi_port == 0 ? dma_spi0_tx_isr : dma_spi1_tx_isr);
        dma_interrupt_on_completion(dma_tx);
        dma_trigger_at_hw_evt(dma_tx, spi_port == 0 ? DMAMUX_SOURCE_SPI0_TX : DMAMUX_SOURCE_SPI1_TX);
    }

    uint8_t dma_rx = DMA_CHANNEL_ERROR;
    if (dma_tx != DMA_CHANNEL_ERROR) {
        dma_rx = dma_acquire_channel();
        pi->dma_rx = dma_rx;
        if (dma_rx != DMA_CHANNEL_ERROR) {
            dma_disable_on_completion(dma_rx);
            dma_attach_interrupt(dma_rx, spi_port == 0 ? dma_spi0_rx_isr : dma_spi1_rx_isr);
            dma_interrupt_on_completion(dma_rx);
            dma_trigger_at_hw_evt(dma_rx, spi_port == 0 ? DMAMUX_SOURCE_SPI0_RX : DMAMUX_SOURCE_SPI1_RX);
        }
    } else {
        // both DMA channels or none
        dma_release_channel(dma_tx);
        pi->dma_tx = DMA_CHANNEL_ERROR;
    }

    return spi_port;
}


void spi_master_start_send(wk_port_request* request)
{
    // take ownership of request; release it when the transmission is done
    spi_port port = (uint8_t) request->header.port_id;

    // if SPI port busy then queue request
    if (port_info[port].state != STATE_WAITING) {
        append_request(port, request);
        return;
    }

    master_start_send_2(request);
}


void master_start_send_2(wk_port_request* request)
{
    // takes ownership of request; release it when the transmission is done

    // chip select
    digital_pin cs = request->action_attribute2;
    if (cs != DIGI_PIN_ERROR)
        digital_pin_set_output(cs, 0);
    
    // initialize state
    spi_port port = (uint8_t) request->header.port_id;
    spi_port_info_t* pi = &port_info[port];
    pi->state = STATE_TX;
    pi->sub_state = SUB_STATE_DATA;
    pi->request = request;
    pi->processed = 0;
    
    // setup DMA
    KINETISL_SPI_t* spi = get_spi_ctrl(port);
    uint8_t dma_tx = pi->dma_tx;
    uint8_t dma_rx = pi->dma_rx;
    
    uint16_t data_len = WK_PORT_REQUEST_DATA_LEN(pi->request);
    uint8_t use_dma = dma_tx != DMA_CHANNEL_ERROR && data_len > 3;

    if (use_dma) {
        // switch to DMA for bulk of data
        dma_source_byte_buffer(dma_tx, request->data + 1, data_len - 1);
        dma_dest_byte(dma_tx, &spi->DL);
        dma_source_byte(dma_rx, &spi->DL);
        dma_dest_byte_buffer(dma_rx, request->data, data_len);
        pi->sub_state = SUB_STATE_DMA;
        // DMA will start after first byte    
    } else {
        spi->C1 |= SPI_C1_SPIE;
    }

    // transmit first byte
    while ((spi->S & SPI_S_SPTEF) == 0);
    spi->DL = request->data[0];

    if (use_dma) {
        spi->C2 |= SPI_C2_TXDMAE | SPI_C2_RXDMAE;
        dma_enable(dma_rx);
        dma_enable(dma_tx);
    }
}


void spi_set_digital_output(wk_port_request* request)
{
    // take ownership of request; release it when the transmission is done
    spi_port port = (uint8_t) request->action_attribute2;

    // if SPI port busy then queue request
    if (port_info[port].state != STATE_WAITING) {
        append_request(port, request);
        return;
    }

    set_digital_output_2(request);
}


void set_digital_output_2(wk_port_request* request)
{
    spi_port port = (uint8_t) request->action_attribute2;
    digital_pin pin = request->header.port_id & PORT_GROUP_DETAIL_MASK;
    digital_pin_set_output(pin, (uint8_t)request->value1);

    // free request
    uint16_t request_id = request->header.request_id;
    uint16_t port_id = request->header.port_id;
    mm_free(request);

    // send completion message
    wk_send_port_event_2(port_id, WK_EVENT_SET_DONE, request_id, 0, 0, 0, NULL, 0);

    check_queue(port);
}


void set_frequency(KINETISL_SPI_t* spi, uint32_t bus_rate, uint32_t frequency)
{
    uint16_t target_div = (bus_rate + frequency / 2) / frequency;

    // binary search
    int lower = 0;
    int upper = NUM_F_DIVS - 1;
    while (lower < upper) {
        int mid = (upper + lower) / 2;
        if (freq_divs[mid].divider < target_div)
            lower = mid + 1;
        else
            upper = mid;
    }
    // result:
    //    forall i  < upper: freq_divs[i].divider <  target_div
    //    forall i => upper: freq_divs[i].divider >= target_div

    spi->BR = freq_divs[upper].f_div;
}


void spi_port_release(spi_port port)
{
    if (port >= NUM_SPI_PORTS)
        return;

    spi_port_info_t* pi = &port_info[port];
    if (pi->state == STATE_INACTIVE)
        return;
    
    pi->state = STATE_INACTIVE;
    pi->sub_state = SUB_STATE_IDLE;
        
    KINETISL_SPI_t* spi = get_spi_ctrl(port);
    spi->C1 = 0;
    
    if (pi->dma_tx != DMA_CHANNEL_ERROR)
        dma_release_channel(pi->dma_tx);
    if (pi->dma_rx != DMA_CHANNEL_ERROR)
        dma_release_channel(pi->dma_rx);

    PCR(SCK_map[pi->sck].port, SCK_map[pi->sck].pin) = 0;
    PCR(MOSI_map[pi->mosi].port, MOSI_map[pi->mosi].pin) = 0;
    if (pi->miso != 0xff)
        PCR(MISO_map[pi->miso].port, MISO_map[pi->miso].pin) = 0;
    
    clear_request_queue(port);
}


void dma_tx_isr_handler(uint8_t port)
{
    KINETISL_SPI_t* spi = get_spi_ctrl(port);
    spi_port_info_t* pi = &port_info[port];
    uint8_t dma = pi->dma_tx;

    if (dma == DMA_CHANNEL_ERROR)
        return; // oops

    if (dma_is_error(dma)) {
        // If this error handling code is ever executed, it is most likely a software bug.
        // In SPI, there are no ACKs. And the master cannot recognize if the
        // slave has received or transmitted something or even exists.
        DEBUG_OUT("SPI DMA TX error");

        uint32_t processed = WK_PORT_REQUEST_DATA_LEN(pi->request) - dma_bytes_remaining(dma) + 1;
        dma_clear_error(dma);

        // disable RX DMAs
        uint8_t dma_rx = pi->dma_rx;
        if (dma_rx != DMA_CHANNEL_ERROR) {
            dma_disable(dma_rx);
            dma_clear_error(dma_rx);
            dma_clear_interrupt(dma_rx);
        }
        
        // disable DMA
        spi->C2 &= ~(SPI_C2_TXDMAE | SPI_C2_RXDMAE);
        
        uint8_t status = SPI_STATUS_UNKNOWN;
        write_complete(port, status, processed);

    // DMA has written last byte to data register; SPI starts to send last byte
    } else if (dma_is_complete(dma)) {
        // disable TX DMA
        spi->C2 &= ~SPI_C2_TXDMAE;
        pi->sub_state = SUB_STATE_DMA_LAST_BYTE;
        dma_clear_complete(dma);        
        // no further action; the last byte is still being received

    } else {
        DEBUG_OUT("Spurious SPI DMA TX interrupt");
        return; // oops
    }

    // disable TX DMA
    dma_disable(dma);
    dma_clear_interrupt(dma);
}


void dma_rx_isr_handler(uint8_t port)
{
    KINETISL_SPI_t* spi = get_spi_ctrl(port);
    spi_port_info_t* pi = &port_info[port];
    uint8_t dma = pi->dma_rx;

    if (dma == DMA_CHANNEL_ERROR)
        return; // oops

    uint8_t status;
    uint32_t processed = WK_PORT_REQUEST_DATA_LEN(pi->request) - dma_bytes_remaining(dma);
        
    if (dma_is_error(dma)) {
        // If this error handling code is ever executed, it is most likely a software bug.
        // In SPI, there are no ACKs. And the master cannot recognize if the
        // slave has received or transmitted something or even exists.
        DEBUG_OUT("SPI DMA RX error");

        dma_clear_error(dma);

        // disable TX DMA
        uint8_t dma_tx = pi->dma_tx;
        if (dma_tx != DMA_CHANNEL_ERROR) {
            dma_disable(dma_tx);
            dma_clear_interrupt(dma_tx);
            dma_clear_error(dma_tx);
        }

        status = SPI_STATUS_UNKNOWN;

    // DMA has read last byte from data register
    } else if (dma_is_complete(dma)) {
        dma_clear_complete(dma);        
        status = SPI_STATUS_OK;
    
    } else {
        DEBUG_OUT("Spurious SPI DMA RX interrupt");
        return; // oops
    }

    // disable DMA
    spi->C2 &= ~(SPI_C2_TXDMAE | SPI_C2_RXDMAE);
    dma_disable(dma);
    dma_clear_interrupt(dma);

    write_complete(port, status, processed);        
}


void dma_spi0_tx_isr()
{
    dma_tx_isr_handler(0);
}

void dma_spi0_rx_isr()
{
    dma_rx_isr_handler(0);
}

void dma_spi1_tx_isr()
{
    dma_tx_isr_handler(1);
}

void dma_spi1_rx_isr()
{
    dma_rx_isr_handler(1);
}

void spi0_isr()
{
    spi_isr_handler(0);
}

void spi1_isr()
{
    spi_isr_handler(1);
}


void write_complete(spi_port port, uint8_t status, uint16_t len)
{
    // save relevant values
    spi_port_info_t* pi = &port_info[port];
    wk_port_request* request = pi->request;
    uint16_t port_id = request->header.port_id;
    uint16_t request_id = request->header.request_id;

    // chip select
    digital_pin cs = request->action_attribute2;
    if (cs != DIGI_PIN_ERROR)
        digital_pin_set_output(cs, 1);
    
    // free request
    mm_free(request);
    pi->request = NULL;
    pi->state = STATE_WAITING;
    pi->sub_state = SUB_STATE_IDLE;
    pi->processed = 0;

    // send completion message
    wk_send_port_event_2(port_id, WK_EVENT_TX_COMPLETE, request_id, status, len, 0, NULL, 0);

    check_queue(port);
}


// check for pending request in queue
void check_queue(spi_port port)
{
    wk_port_request* request = get_next_request(port);

    if (request == NULL)
        return;

    if (request->action == WK_PORT_ACTION_TX_DATA) {
        master_start_send_2(request);
    } else if (request->action == WK_PORT_ACTION_SET_VALUE) {
        set_digital_output_2(request);
    } else {
        DEBUG_OUT("Invalid request in SPI queue");
    }
//    } else if (request->action == WK_PORT_ACTION_RX_DATA) {
//        master_start_recv_2(request);
//        mm_free(request);
//    }
}


// --- TX queue


wk_port_request* get_next_request(spi_port port)
{    
    spi_port_info_t* pi = &port_info[port];
    if (pi->circ_request_head == pi->circ_request_tail)
        return NULL;
    
    pi->circ_request_tail++;
    if (pi->circ_request_tail >= CIRC_QUEUE_SIZE)
        pi->circ_request_tail = 0;
    
    return pi->circ_request_buf[pi->circ_request_tail];
}


void append_request(spi_port port, wk_port_request* request)
{
    spi_port_info_t* pi = &port_info[port];
    uint8_t head = pi->circ_request_head + 1;
    if (head >= CIRC_QUEUE_SIZE)
        head = 0;
    if (head != pi->circ_request_tail) {
        pi->circ_request_buf[head] = request;
        pi->circ_request_head = head;
        return;
    }
    
    DEBUG_OUT("SPI request overflow");
    mm_free(request);
}


void clear_request_queue(spi_port port)
{
    while (1) {
        wk_port_request* request = get_next_request(port);
        if (request == NULL)
            break;
        mm_free(request);
    }
}


// SPI interrupt
// Triggered when a new byte has been received
// (real or dummy)

void spi_isr_handler(uint8_t port)
{
    spi_port_info_t* pi = &port_info[port];
    KINETISL_SPI_t* spi = get_spi_ctrl(port);
    uint8_t sub_state = pi->sub_state;

    if (sub_state == SUB_STATE_DATA) {

        wk_port_request* request = pi->request;

        // read received data byte;
        // must first read status register even though we now
        // data byte is ready as interrupt was triggered;
        // replaces: while ((spi->S & SPI_S_SPRF) == 0);
        uint8_t __attribute__((unused)) dummy = spi->S;
        int processed = pi->processed;
        request->data[processed] = spi->DL;
        processed++;
        pi->processed = (uint16_t)processed;
        
        int data_len = WK_PORT_REQUEST_DATA_LEN(pi->request);
        if (processed == data_len) {
            // request is complete
            spi->C1 &= ~SPI_C1_SPIE;
            write_complete(port, SPI_STATUS_OK, pi->processed);    
            
        } else {
            // transmit next data byte;
            // must first read status register even though we know register is
            // ready to receive data as writes are interleaved with reads;
            // replaces: while ((spi->S & SPI_S_SPTEF) == 0);
            uint8_t __attribute__((unused)) dummy = spi->S;
            spi->DL = request->data[processed];
        }

    } else {
        DEBUG_OUT("Spurious SPI interrupt");
    }
}


static KINETISL_SPI_t* SPI_CTRL[] = {
    &KINETISL_SPI0,
    &KINETISL_SPI1
};

KINETISL_SPI_t* get_spi_ctrl(uint8_t port)
{
    return SPI_CTRL[port];
}
