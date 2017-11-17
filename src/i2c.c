/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

 /*
  * This code heavily benefited from the Teensy I2C library
  * (https://github.com/nox771/i2c_t3) by Brian (nox771).
  *
  * Further sources: https://community.nxp.com/docs/DOC-330946
  */

#include <string.h>
#include "kinetis.h"

#include "i2c.h"
#include "dma.h"
#include "mem.h"
#include "delay.h"
#include "ports.h"
#include "usb.h"
#include "wirekite.h"
#include "debug.h"


// Hopefully, these defines will be in kinetis.h one day...
#define I2C_FLT_SSIE    ((uint8_t)0x20)         // Start/Stop Interrupt Enable
#define I2C_FLT_STARTF  ((uint8_t)0x10)         // Start Detect Flag


#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3
#define PORT_E 4

typedef struct {
    uint32_t i2c_port : 3;
    uint32_t scl_port : 3;
    uint32_t sda_port : 3;
    uint32_t scl_pin : 5;
    uint32_t sda_pin : 5;
    uint32_t scl_alt : 3;
    uint32_t sda_alt : 3;
} port_map_t;

#if defined(__MKL26Z64__)

// Pin/port mapping for Teensy LC
static const port_map_t port_map[] = {
    { 0, PORT_B, PORT_B,  0,  1, 2, 2 },  // I2C0 pin 16/17 PTB0/PTB1
    { 0, PORT_B, PORT_B,  2,  3, 2, 2 },  // I2C0 pin 19/18 PTB2/PTB3
    { 1, PORT_C, PORT_C,  1,  2, 2, 2 }   // I2C1 pin 22/23 PTC1/PTC2
};

#elif defined(__MK20DX256__)

// Pin/port mapping for Teensy 3.2
static const port_map_t port_map[] = {
    { 0, PORT_B, PORT_B,  0,  1, 2, 2 },  // I2C0 pin 16/17 PTB0/PTB1
    { 0, PORT_B, PORT_B,  2,  3, 2, 2 },  // I2C0 pin 19/18 PTB2/PTB3
    { 1, PORT_C, PORT_C, 10, 11, 2, 2 }   // I2C1 pin 29/30 PTC10/PTC11
};

#endif


static const freq_div_t freq_divs[] = {
    { 0x00, 21},
    { 0x01, 23},
    { 0x02, 25},
    { 0x03, 27},
    { 0x08, 29},
    { 0x05, 31},
    { 0x09, 33},
    { 0x06, 35},
    { 0x0A, 38},
    { 0x07, 42},
    { 0x0C, 46},
    { 0x0D, 50},
    { 0x43, 54},
    { 0x0E, 58},
    { 0x45, 62},
    { 0x12, 66},
    { 0x0F, 70},
    { 0x13, 76},
    { 0x14, 84},
    { 0x15, 92},
    { 0x19, 100},
    { 0x16, 108},
    { 0x1A, 116},
    { 0x85, 124},
    { 0x17, 132},
    { 0x4F, 140},
    { 0x1C, 152},
    { 0x1D, 168},
    { 0x55, 184},
    { 0x1E, 200},
    { 0x56, 216},
    { 0x22, 232},
    { 0x1F, 248},
    { 0x23, 264},
    { 0x8F, 280},
    { 0x24, 304},
    { 0x25, 336},
    { 0x95, 368},
    { 0x26, 400},
    { 0x96, 432},
    { 0x2A, 464},
    { 0x27, 496},
    { 0x2B, 543},
    { 0x2C, 607},
    { 0x2D, 701},
    { 0x2E, 830},
    { 0x32, 927},
    { 0x2F, 991},
    { 0x33, 1086},
    { 0x34, 1214},
    { 0x35, 1402},
    { 0x36, 1659},
    { 0x3A, 1855},
    { 0x37, 1983},
    { 0x3B, 2172},
    { 0x3C, 2429},
    { 0x3D, 2804},
    { 0x3E, 3318},
    { 0x7A, 3710},
    { 0x3F, 3966},
    { 0x7B, 4344},
    { 0x7C, 4857},
    { 0x7D, 5609},
    { 0x7E, 6636},
    { 0xBA, 7420},
    { 0x7F, 7932},
    { 0xBB, 8689},
    { 0xBC, 9715},
    { 0xBD, 11217},
    { 0xBE, 13738},
    { 0xBF, 0}
};

#define NUM_F_DIVS (sizeof(freq_divs)/sizeof(freq_divs[0]))


#define STATE_INACTIVE 0
#define STATE_IDLE  1
#define STATE_TX 2
#define STATE_RX 3
#define STATE_COOL_DOWN 5
#define STATE_RESET_BUS 4

#define SUB_STATE_NONE 0
#define SUB_STATE_ADDR 1
#define SUB_STATE_DATA 2
#define SUB_STATE_DMA 3


#define CIRC_QUEUE_SIZE 20


typedef struct {
    uint8_t state;
    uint8_t sub_state;
    uint16_t processed;
    union {
        wk_port_request* request;
        wk_port_event* response;
    };
    wk_port_request* circ_request_buf[CIRC_QUEUE_SIZE];
    uint8_t circ_request_head;
    uint8_t circ_request_tail;
    uint8_t dma;
    uint8_t pins;
    uint32_t trx_delay;
} port_info_t;

#define NUM_I2C_PORTS 2

static port_info_t port_info[NUM_I2C_PORTS];


static void master_start_tx_now(wk_port_request* request);
static void master_start_rx_now(wk_port_request* request);
static KINETIS_I2C_t* get_i2c_ctrl(i2c_port port);
static void set_frequency(KINETIS_I2C_t* i2c, uint32_t bus_rate, uint32_t frequency);
static uint8_t acquire_bus(i2c_port port);
static wk_port_event* create_response(uint16_t port_id, uint16_t request_id, uint16_t rx_size);
static void switch_to_rx(i2c_port port);
static void master_start_rx_3(i2c_port port, uint16_t slave_addr);
static void dma_i2c0_isr();
static void dma_i2c1_isr();
static void dma_isr_handler(uint8_t port);
static void write_complete(i2c_port port, uint8_t status, uint16_t len);
static void read_complete(i2c_port port, uint8_t status, uint16_t len);
static wk_port_request* get_next_request(i2c_port port);
static void append_request(i2c_port port, wk_port_request* msg);
static void clear_request_queue(i2c_port port);
static void reset_bus_tick(uint32_t param);
static void reset_bus_now(wk_port_request* request);
static void cool_down(i2c_port port);
static void cool_down_done(uint32_t param);


void i2c_init()
{
    // nothing to do
}

void i2c_reset()
{
    for (int i = 0; i < NUM_I2C_PORTS; i++)
        i2c_port_release(i);
}

i2c_port i2c_master_init(uint8_t pins, uint16_t attributes, uint32_t frequency)
{
    if (pins > I2C_PINS_MAX)
        return I2C_PORT_ERROR;

    uint8_t port = port_map[pins].i2c_port;
    port_info_t* pi = &port_info[port];
    if (pi->state != STATE_INACTIVE)
        return I2C_PORT_ERROR;

    pi->state = STATE_IDLE;
    pi->trx_delay = 9000000 / frequency; // delay between transactions: 9 cycles

    SIM_SCGC4 |= port == 0 ? SIM_SCGC4_I2C0 : SIM_SCGC4_I2C1;

    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);

    // configure as master
    i2c->C2 = I2C_C2_HDRS; // no effect on LC
    i2c->A1 = 0;
    i2c->RA = 0;

    set_frequency(i2c, port == 0 ? F_BUS : F_CPU, frequency);
    
    i2c->C1 = I2C_C1_IICEN;
    i2c->S = I2C_S_IICIF | I2C_S_ARBL;

    NVIC_CLEAR_PENDING(port == 0 ? IRQ_I2C0 : IRQ_I2C1);
    NVIC_ENABLE_IRQ(port == 0 ? IRQ_I2C0 : IRQ_I2C1);
    
    // configure SCL pin
    const port_map_t* map = port_map + pi->pins;
    PCR(map->scl_port, map->scl_pin) = PORT_PCR_MUX(map->scl_alt) | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE;
    
    // configure SDA pin
    PCR(map->sda_port, map->sda_pin) = PORT_PCR_MUX(map->sda_alt) | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE;

    // configure DMA
    uint8_t dma = dma_acquire_channel();
    pi->dma = dma;
    if (dma != DMA_CHANNEL_ERROR) {
        dma_disable_on_completion(dma);
        dma_attach_interrupt(dma, port == 0 ? dma_i2c0_isr : dma_i2c1_isr);
        dma_interrupt_on_completion(dma);
        dma_trigger_at_hw_evt(dma, port == 0 ? DMAMUX_SOURCE_I2C0 : DMAMUX_SOURCE_I2C1);
    }

    return port;
}

void i2c_port_release(i2c_port port)
{
    if (port >= NUM_I2C_PORTS)
        return;

    port_info_t* pi = &port_info[port];
    if (pi->state == STATE_INACTIVE)
        return;
    
    const port_map_t* map = port_map + pi->pins;
    if (pi->state == STATE_RESET_BUS) {
        // restore pin settings
        uint32_t scl_mask = 1 << (uint32_t)map->scl_pin;
        GPIO_PORT[map->scl_pin]->PCOR = scl_mask; // set SCL to low
        GPIO_PORT[map->scl_port]->PDDR &= ~scl_mask; // SCL direction: input
    }
    pi->state = STATE_INACTIVE;
    pi->sub_state = SUB_STATE_NONE;
    
    NVIC_DISABLE_IRQ(port == 0 ? IRQ_I2C0 : IRQ_I2C1);
    
    if (pi->dma != DMA_CHANNEL_ERROR)
        dma_release_channel(pi->dma);
    
    PCR(map->scl_port, map->scl_pin) = 0;
    PCR(map->sda_port, map->sda_pin) = 0;

    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    i2c->C1 = 0;
    i2c->FLT = I2C_FLT_STOPF | I2C_FLT_STARTF;
    i2c->S = I2C_S_IICIF | I2C_S_ARBL;

    mm_free(pi->request);
    pi->request = NULL;

    SIM_SCGC4 &= port == 0 ? ~SIM_SCGC4_I2C0 : ~SIM_SCGC4_I2C1;
    
    clear_request_queue(port);
}

// Takes owernship of request
void i2c_master_start_tx(wk_port_request* request)
{
    i2c_port port = request->header.port_id;
    if (port >= NUM_I2C_PORTS) {
        mm_free(request);
        return;
    }

    if (port_info[port].state != STATE_IDLE) {
        // if I2C port busy then queue request
        append_request(port, request);
    } else {
        // start request immediately
        master_start_tx_now(request);
    }
}


// Takes owernship of request
void master_start_tx_now(wk_port_request* request)
{
    i2c_port port = request->header.port_id;

    // reset flags
    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    i2c->FLT |= I2C_FLT_STOPF | I2C_FLT_STARTF;
    i2c->FLT &= ~I2C_FLT_SSIE;
    i2c->S = I2C_S_IICIF | I2C_S_ARBL;

    // initialize state
    port_info_t* pi = &port_info[port];
    pi->state = STATE_TX;
    pi->sub_state = SUB_STATE_ADDR;
    pi->request = request;
    pi->processed = 0;

    // acquire bus
    uint8_t status = acquire_bus(port);
    if (status != I2C_STATUS_OK) {
        write_complete(port, status, 0);
        return;
    }

    // setup DMA
    uint8_t dma = pi->dma;
    if (pi->dma != DMA_CHANNEL_ERROR && WK_PORT_REQUEST_DATA_LEN(pi->request) > 3) {
        // switch to DMA for bulk of data
        dma_source_byte_buffer(dma, request->data, WK_PORT_REQUEST_DATA_LEN(pi->request));
        dma_dest_byte(dma, &i2c->D);
        pi->sub_state = SUB_STATE_DMA;
        i2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX | I2C_C1_DMAEN; // enable DMA
        dma_enable(pi->dma);
        // DMA will start after first byte (slave address)
    
    } else {
        // enable interrupt
        i2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX;
    }

    // transmit address (writing mode)
    i2c->D = (uint8_t)(request->action_attribute2 << 1);
}


// Does NOT take ownership of request
void i2c_master_start_rx(wk_port_request* request)
{
    i2c_port port = request->header.port_id;    
    if (port >= NUM_I2C_PORTS)
        return;

    // if port is busy copy the request and queue it
    if (port_info[port].state != STATE_IDLE) {

        wk_port_request* copy = mm_alloc(request->header.message_size);
        if (copy == NULL) {
            DEBUG_OUT("I2C insufficient memory");
            return;
        }
        
        memcpy(copy, request, request->header.message_size);
        append_request(port, copy);

    } else {
        master_start_rx_now(request);
    }
}


// Does NOT take ownership of request
void master_start_rx_now(wk_port_request* request)
{
    i2c_port port = request->header.port_id;
    
    // allocate response with RX buffer
    port_info_t* pi = &port_info[port];
    pi->response = create_response(request->header.port_id, request->header.request_id, (uint16_t)request->value1);
    if (pi->response == NULL)
        return;

    // reset flags
    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    i2c->FLT |= I2C_FLT_STOPF | I2C_FLT_STARTF;
    i2c->FLT &= ~I2C_FLT_SSIE;
    i2c->S = I2C_S_IICIF | I2C_S_ARBL;

    // start receive
    master_start_rx_3(port, request->action_attribute2);
}


// Switch from transmitting to receiving
void switch_to_rx(i2c_port port)
{
    port_info_t* pi = &port_info[port];
    wk_port_request* request = pi->request;

    // retain relevant data
    uint16_t port_id = request->header.port_id;
    uint16_t request_id = request->header.request_id;
    uint16_t rx_size = (uint16_t)request->value1;
    uint16_t slave_addr = request->action_attribute2;

    // free request with transmit data
    mm_free(request);
    pi->state = STATE_IDLE;

    // allocate response with RX buffer
    pi->response = create_response(port_id, request_id, rx_size);
    if (pi->response == NULL)
        return;
    
    master_start_rx_3(port, slave_addr);
}


void master_start_rx_3(i2c_port port, uint16_t slave_addr)
{
    // initialize state
    port_info_t* pi = &port_info[port];
    pi->state = STATE_RX;
    pi->sub_state = SUB_STATE_ADDR;
    pi->processed = 0;

    // acquire bus
    uint8_t status = acquire_bus(port);
    if (status != I2C_STATUS_OK) {
        read_complete(port, status, 0);
        return;
    }

    // prepare DMA
    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    uint8_t dma = pi->dma;
    if (dma != DMA_CHANNEL_ERROR) {
        dma_source_byte(dma, &i2c->D);
        dma_dest_byte_buffer(dma, pi->response->data, WK_PORT_EVENT_DATA_LEN(pi->response) - 1);
    }

    // enable interrupt and send address (for reading)
    i2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX;
    i2c->D = (uint8_t)((slave_addr << 1) | 1);
}


// check for pending request in queue
void check_queue(i2c_port port)
{
    port_info_t* pi = &port_info[port];
    while (pi->state == STATE_IDLE) {
        wk_port_request* request = get_next_request(port);
        if (request == NULL)
            return;

        if (request->action == WK_PORT_ACTION_RX_DATA) {
            master_start_rx_now(request);
            mm_free(request);
        } else if (request->action == WK_PORT_ACTION_RESET) {
            reset_bus_now(request);
        } else {
            master_start_tx_now(request);
        }
    }
}

// Caller must take owernship of returned object
wk_port_event* create_response(uint16_t port_id, uint16_t request_id, uint16_t rx_size)
{
    // allocate response message and copy data
    uint16_t msg_size = WK_PORT_EVENT_ALLOC_SIZE(rx_size);
    wk_port_event* response = mm_alloc(msg_size);
    if (response == NULL) {
        DEBUG_OUT("I2C RX insufficient mem");
        wk_send_port_event_2(port_id, WK_EVENT_DATA_RECV, request_id, I2C_STATUS_OUT_OF_MEMORY, 0, 0, NULL, 0);
        cool_down((i2c_port)port_id);
        return NULL;
    }
        
    response->header.message_size = msg_size;
    response->header.message_type = WK_MSG_TYPE_PORT_EVENT;
    response->header.port_id = port_id;
    response->header.request_id = request_id;
    response->event = WK_EVENT_DATA_RECV;
    return response;
}


void i2c_isr_handler(uint8_t port)
{
    port_info_t* pi = &port_info[port];
    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    uint8_t status = i2c->S;
    uint8_t sub_state = pi->sub_state;
    uint8_t completion_status = 0xff;
    
    i2c->S = I2C_S_IICIF | I2C_S_ARBL; // clear interrupt flag
    
    // master to slave transmission
    if (pi->state == STATE_TX) {

        // arbitration lost
        if (status & I2C_S_ARBL) {
            completion_status = I2C_STATUS_ARB_LOST;

        // no ACK received
        } else if (status & I2C_S_RXAK) {
            completion_status = I2C_STATUS_DATA_NAK;
            if (sub_state == SUB_STATE_ADDR || (sub_state == SUB_STATE_DMA && pi->processed == 0))
                completion_status = I2C_STATUS_ADDR_NAK;

        // transmission is progressing
        } else {

            // address transmitted
            if (sub_state == SUB_STATE_ADDR) {
                pi->sub_state = SUB_STATE_DATA;
                i2c->D = pi->request->data[0];
                
            // transmission complete
            } else if (pi->processed == WK_PORT_REQUEST_DATA_LEN(pi->request) - 1) {
                pi->processed++;
                if (pi->request->action == WK_PORT_ACTION_TX_DATA) {
                    completion_status = I2C_STATUS_OK;
                } else {
                    // continue receiving data
                    i2c->S = I2C_S_IICIF; // clear interrupt flag
                    switch_to_rx(port);
                }

            // transmit next byte
            } else {
                pi->processed++;
                i2c->D = pi->request->data[pi->processed];
            }
        }

        if (completion_status != 0xff) {
            i2c->C1 = I2C_C1_IICEN; // reset to RX
            write_complete(port, completion_status, pi->processed);    
        }

    // master to slave receive
    } else if (pi->state == STATE_RX) {

        uint16_t data_len = WK_PORT_EVENT_DATA_LEN(pi->request);
        
        // address transmitted
        if (sub_state == SUB_STATE_ADDR) {

            // arbitration lost
            if (status & I2C_S_ARBL) {
                completion_status = I2C_STATUS_ARB_LOST;

            // no ACK received
            } else if (status & I2C_S_RXAK) {
                completion_status = sub_state == I2C_STATUS_ADDR_NAK;

            // slave address transmitted
            } else {

                if (data_len > 3 && pi->dma != DMA_CHANNEL_ERROR) {
                    // enable DMA
                    pi->sub_state = SUB_STATE_DMA;
                    i2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_DMAEN; // enable DMA
                    dma_enable(pi->dma);
                    // DMA will start after first byte (slave address)
                    
                } else {
                    pi->sub_state = SUB_STATE_DATA;
                    i2c->C1 = data_len > 1
                        ? I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST
                        : I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TXAK; // set NAK
                }
                uint8_t __attribute__((unused)) data = i2c->D; // dummy read (see chip manual, I2C interrupt routine)
            }
            
        // received byte
        } else {

            // second to last byte: set NAK
            if (pi->processed == data_len - 2) {
                i2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TXAK;
            }
                
            // receive completed
            if (pi->processed == data_len - 1) {
                i2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
                pi->request->data[pi->processed++] = i2c->D;
                // complete bit
                for (int i = 0; i < 20; i++) { // TODO
                    __asm__ volatile ("nop");
                }
                completion_status = I2C_STATUS_OK;
                
            // receive next byte
            } else {
                pi->request->data[pi->processed++] = i2c->D;
            }
        }

        if (completion_status != 0xff) {
            i2c->C1 = I2C_C1_IICEN; // reset to RX
            read_complete(port, completion_status, pi->processed);
        }
    
    } else {
        DEBUG_OUT("Spurious I2C interrupt");
    }
}


uint8_t acquire_bus(i2c_port port)
{
    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    if (i2c->C1 & I2C_C1_MST) {
        // I2C module is already master; send a repeated start condition
        i2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_RSTA | I2C_C1_TX;

    } else {
        if (i2c->S & I2C_S_BUSY) {
            // bus is busy; return error
            return I2C_STATUS_BUS_BUSY;
        }

        // become master in transmit mode and send start condition
        i2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;

        // verify success
        if (!(i2c->C1 & I2C_C1_MST)) {
            return I2C_STATUS_BUS_BUSY;
        }
    }

    return 0;
}


static KINETIS_I2C_t* I2C_CTRL[] = {
    &KINETIS_I2C0,
    &KINETIS_I2C1
};

KINETIS_I2C_t* get_i2c_ctrl(uint8_t port)
{
    return I2C_CTRL[port];
}


void set_frequency(KINETIS_I2C_t* i2c, uint32_t bus_rate, uint32_t frequency)
{
    uint16_t target_div = (bus_rate + frequency / 2) / frequency;
    int idx = frequency_lookup(freq_divs, NUM_F_DIVS, target_div);

    i2c->F = freq_divs[idx].f_div;
    i2c->FLT = bus_rate >= 48000000 ? 4 : bus_rate / 12000000;
}


void dma_isr_handler(uint8_t port)
{
    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    port_info_t* pi = &port_info[port];
    uint8_t dma = pi->dma;

    // master to slave transmission
    if (pi->state == STATE_TX) {
        
        // DMA has written last byte to data register; I2C starts to send last byte
        if (dma_is_complete(dma)) {            
            dma_clear_interrupt(dma);
            dma_clear_complete(dma);
            pi->sub_state = SUB_STATE_DATA;
            pi->processed = WK_PORT_REQUEST_DATA_LEN(pi->request) - 1;
            // disable DMA
            i2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX;
            i2c->S = I2C_S_IICIF; // clear flags
            
        } else {
            uint8_t status = (i2c->S & I2C_S_ARBL) ? I2C_STATUS_ARB_LOST : I2C_STATUS_UNKNOWN;
            i2c->S = I2C_S_ARBL | I2C_S_IICIF; // clear flags
            i2c->C1 = I2C_C1_IICEN; // disable, set to RX

            dma_clear_interrupt(dma);
            dma_clear_error(dma);

            uint32_t processed = WK_PORT_REQUEST_DATA_LEN(pi->request) - dma_bytes_remaining(dma);
            write_complete(port, status, processed);
        }

    } else {

        // DMA has read second to last byte to data register; I2C starts to read last byte
        if (dma_is_complete(dma)) {
            // disable DMA and signal NAK
            i2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TXAK;
            i2c->S = I2C_S_IICIF; // clear flags

            dma_clear_interrupt(dma);
            dma_clear_complete(dma);
            pi->sub_state = SUB_STATE_DATA;
            pi->processed = WK_PORT_EVENT_DATA_LEN(pi->response) - 1;

        } else {
            i2c->S = I2C_S_ARBL | I2C_S_IICIF; // clear flags
            i2c->C1 = I2C_C1_IICEN; // disable, set to RX

            dma_clear_interrupt(dma);
            dma_clear_error(dma);

            uint32_t processed = WK_PORT_EVENT_DATA_LEN(pi->request) - 1 - dma_bytes_remaining(dma);
            read_complete(port, I2C_STATUS_UNKNOWN, processed);
        }
    }
}


void dma_i2c0_isr()
{
    dma_isr_handler(0);
}


void dma_i2c1_isr()
{
    dma_isr_handler(1);
}


void i2c0_isr()
{
    i2c_isr_handler(0);
}


void i2c1_isr()
{
    i2c_isr_handler(1);
}


void write_complete(i2c_port port, uint8_t status, uint16_t len)
{
    // save relevant values
    port_info_t* pi = &port_info[port];
    uint16_t port_id = pi->request->header.port_id;
    uint16_t request_id = pi->request->header.request_id;

    // free request
    mm_free(pi->request);
    pi->request = NULL;
    pi->state = STATE_IDLE;

    // send completion message
    wk_send_port_event_2(port_id, WK_EVENT_TX_COMPLETE, request_id, status, len, 0, NULL, 0);

    cool_down(port);
}


void read_complete(i2c_port port, uint8_t status, uint16_t len)
{
    // fill in additional values
    port_info_t* pi = &port_info[port];
    pi->response->event_attribute1 = status;
    pi->response->event_attribute2 = len;
    pi->response->header.message_size = WK_PORT_EVENT_ALLOC_SIZE(pi->processed);

    // send response
    endp1_tx_msg(&pi->response->header);

    // clean up
    pi->response = NULL;
    pi->state = STATE_IDLE;

    cool_down(port);
}


// --- TX queue


wk_port_request* get_next_request(i2c_port port)
{    
    port_info_t* pi = &port_info[port];
    if (pi->circ_request_head == pi->circ_request_tail)
        return NULL;
    
    pi->circ_request_tail++;
    if (pi->circ_request_tail >= CIRC_QUEUE_SIZE)
        pi->circ_request_tail = 0;
    
    return pi->circ_request_buf[pi->circ_request_tail];
}


void append_request(i2c_port port, wk_port_request* request)
{
    port_info_t* pi = &port_info[port];
    uint8_t head = pi->circ_request_head + 1;
    if (head >= CIRC_QUEUE_SIZE)
        head = 0;
    if (head != pi->circ_request_tail) {
        pi->circ_request_buf[head] = request;
        pi->circ_request_head = head;
    } else {
        DEBUG_OUT("I2C request overflow");
        mm_free(request);
    }
}


void clear_request_queue(i2c_port port)
{
    while (1) {
        wk_port_request* request = get_next_request(port);
        if (request == NULL)
            break;
        mm_free(request);
    }
}


// --- reset bus

// Takes owernship of request
void i2c_reset_bus(wk_port_request* request)
{
    i2c_port port = request->header.port_id;
    if (port >= NUM_I2C_PORTS) {
        mm_free(request);
        return;
    }

    if (port_info[port].state != STATE_IDLE) {
        // if I2C port busy then queue request
        append_request(port, request);
    } else {
        // start request immediately
        reset_bus_now(request);
    }
}


void reset_bus_now(wk_port_request* request)
{
    // reset bus manually simulating the SCL clock
    // until the slave releases SDA

    i2c_port port = request->header.port_id;
    port_info_t* pi = &port_info[port];
    pi->state = STATE_RESET_BUS;
    pi->request = request;
    pi->processed = 0;

    // reconfigure the pins
    const port_map_t* map = port_map + pi->pins;
    uint32_t scl_mask = 1 << (uint32_t)map->scl_pin;
    GPIO_PORT[map->scl_pin]->PSOR = scl_mask; // set SCL to high
    GPIO_PORT[map->scl_port]->PDDR |= scl_mask; // SCL direction: output
    uint32_t sda_mask = 1 << (uint32_t)map->sda_pin;
    GPIO_PORT[map->sda_port]->PDDR &= ~sda_mask; // SDA direction: input

    PCR(map->scl_port, map->scl_pin) = PORT_PCR_MUX(1) | PORT_PCR_SRE;
    PCR(map->sda_port, map->sda_pin) = PORT_PCR_MUX(1);

    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    i2c->C1 = 0;
    i2c->FLT = I2C_FLT_STOPF | I2C_FLT_STARTF;
    i2c->S = I2C_S_IICIF | I2C_S_ARBL;

    delay_wait(pi->trx_delay >> 3, reset_bus_tick, port);
}


void reset_bus_tick(uint32_t param)
{
    i2c_port port = (i2c_port)param;
    port_info_t* pi = &port_info[port];
    pi->processed++;

    const port_map_t* map = port_map + pi->pins;
    uint32_t sda_mask = 1 << (uint32_t)map->sda_pin;
    uint32_t sda_value = GPIO_PORT[map->sda_port]->PDIR & sda_mask;

    if (sda_value == 0 && pi->processed < 18) {
        // toggle SCL
        uint32_t scl_mask = 1 << (uint32_t)map->scl_pin;
        if ((pi->processed & 1) == 0) 
            GPIO_PORT[map->scl_pin]->PSOR = scl_mask; // set SCL to high
        else
            GPIO_PORT[map->scl_pin]->PCOR = scl_mask; // set SCL to low

        delay_wait(pi->trx_delay >> 3, reset_bus_tick, port);

    } else {
        // finished; restore I2C configuration
        KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
        i2c->C1 = I2C_C1_IICEN;
        i2c->S = I2C_S_IICIF | I2C_S_ARBL;

        NVIC_CLEAR_PENDING(port == 0 ? IRQ_I2C0 : IRQ_I2C1);
    
        PCR(map->scl_port, map->scl_pin) = PORT_PCR_MUX(map->scl_alt) | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE;
        PCR(map->sda_port, map->sda_pin) = PORT_PCR_MUX(map->sda_alt) | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE;

        uint32_t scl_mask = 1 << (uint32_t)map->scl_pin;
        GPIO_PORT[map->scl_pin]->PCOR = scl_mask; // set SCL to low
        GPIO_PORT[map->scl_port]->PDDR &= ~scl_mask; // SCL direction: input

        write_complete(port, sda_value == 0 ? I2C_STATUS_TIMEOUT : I2C_STATUS_OK, 0);
    }
}


//
// This I2C code is so efficient, it can start a new I2C transaction within 30µs.
// At 100 kbit/s, that's about the length of 3 bits. For most I2C devices, that's
// too fast. So wait for a short moment.
void cool_down(i2c_port port)
{
    port_info_t* pi = &port_info[port];
    pi->state = STATE_COOL_DOWN;

    delay_wait(pi->trx_delay, cool_down_done, port);
}


void cool_down_done(uint32_t param)
{
    i2c_port port = (i2c_port)param;

    port_info_t* pi = &port_info[port];
    pi->state = STATE_IDLE;
    check_queue(port);
}
    