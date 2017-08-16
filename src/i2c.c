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
#include "wirekite.h"
#include "debug.h"


// Hopefully, these defines will be in kinetis.h one day...
#define I2C_F_DIV52  ((uint8_t)0x43)
#define I2C_F_DIV60  ((uint8_t)0x45)
#define I2C_F_DIV136 ((uint8_t)0x4F)
#define I2C_F_DIV176 ((uint8_t)0x55)
#define I2C_F_DIV352 ((uint8_t)0x95)
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
    uint32_t scl_pin : 4;
    uint32_t sda_pin : 4;
    uint32_t scl_alt : 3;
    uint32_t sda_alt : 3;
} port_map_t;

// Pin/port mapping for Teensy LC
static const port_map_t port_map[] = {
    { 0, PORT_B, PORT_B, 0, 1, 2, 2 },  // I2C0 pin 16/17 PTB0/PTB1
    { 0, PORT_B, PORT_B, 2, 3, 2, 2 },  // I2C0 pin 19/18 PTB2/PTB3
    { 1, PORT_C, PORT_C, 1, 2, 2, 2 }   // I2C1 pin 22/23 PTC1/PTC2
};


static volatile uint32_t* PCR_ADDR[] = {
    &PORTA_PCR0,
    &PORTB_PCR0,
    &PORTC_PCR0,
    &PORTD_PCR0,
    &PORTE_PCR0
};

#define PCR(port, pin) (*(PCR_ADDR[port] + pin))


typedef struct {
    uint8_t f_div;
    uint16_t divider;
} freq_div_t;


static const freq_div_t freq_divs[] = {
    { I2C_F_DIV20, 20 },
    { I2C_F_DIV22, 22 },
    { I2C_F_DIV24, 24 },
    { I2C_F_DIV26, 26 },
    { I2C_F_DIV28, 28 },
    { I2C_F_DIV30, 30 },
    { I2C_F_DIV32, 32 },
    { I2C_F_DIV34, 34 },
    { I2C_F_DIV36, 36 },
    { I2C_F_DIV40, 40 },
    { I2C_F_DIV44, 44 },
    { I2C_F_DIV48, 48 },
    { I2C_F_DIV52, 52 },
    { I2C_F_DIV56, 56 },
    { I2C_F_DIV60, 60 },
    { I2C_F_DIV64, 64 },
    { I2C_F_DIV68, 68 },
    { I2C_F_DIV72, 72 },
    { I2C_F_DIV80, 80 },
    { I2C_F_DIV88, 88 },
    { I2C_F_DIV96, 96 },
    { I2C_F_DIV104, 104 },
    { I2C_F_DIV112, 112 },
    { I2C_F_DIV128, 128 },
    { I2C_F_DIV136, 136 },
    { I2C_F_DIV144, 144 },
    { I2C_F_DIV160, 160 },
    { I2C_F_DIV176, 176 },
    { I2C_F_DIV192, 192 },
    { I2C_F_DIV224, 224 },
    { I2C_F_DIV240, 240 },
    { I2C_F_DIV256, 256 },
    { I2C_F_DIV288, 288 },
    { I2C_F_DIV320, 320 },
    { I2C_F_DIV352, 352 },
    { I2C_F_DIV384, 384 },
    { I2C_F_DIV480, 480 },
    { I2C_F_DIV448, 448 },
    { I2C_F_DIV512, 512 },
    { I2C_F_DIV576, 576 },
    { I2C_F_DIV640, 640 },
    { I2C_F_DIV768, 768 },
    { I2C_F_DIV896, 896 },
    { I2C_F_DIV960, 960 },
    { I2C_F_DIV1024, 1024 },
    { I2C_F_DIV1152, 1152 },
    { I2C_F_DIV1280, 1280 },
    { I2C_F_DIV1536, 1536 },
    { I2C_F_DIV1920, 1920 },
    { I2C_F_DIV1792, 1792 },
    { I2C_F_DIV2048, 2048 },
    { I2C_F_DIV2304, 2304 },
    { I2C_F_DIV2560, 2560 },
    { I2C_F_DIV3072, 3072 },
    { I2C_F_DIV3840, 3840 }
};

#define NUM_F_DIVS (sizeof(freq_divs)/sizeof(freq_divs[0]))


#define STATE_INACTIVE 0
#define STATE_WAITING  1
#define STATE_TX 2
#define STATE_RX 3
#define STATE_ERROR 4

#define SUB_STATE_ADDR 0
#define SUB_STATE_DATA 1


typedef struct {
    uint8_t state;
    uint8_t sub_state;
    uint16_t processed;
    uint8_t data[52];
    uint16_t data_len;
    uint8_t dma;
    uint8_t pins;
    uint16_t request_id;
} port_info_t;

#define NUM_I2C_PORTS 2

static port_info_t port_info[NUM_I2C_PORTS];


static KINETIS_I2C_t* get_i2c_ctrl(i2c_port port);
static void set_frequency(KINETIS_I2C_t* i2c, uint32_t bus_rate, uint32_t frequency);
static uint8_t acquire_bus(i2c_port port);
//static void dma_i2c0_isr();
//static void dma_i2c1_isr();
static void send_write_completion(i2c_port port, uint8_t status, uint16_t len);
static void send_read_completion(i2c_port port, uint8_t status, uint16_t len);


void i2c_init()
{
    // nothing to do
}

void i2c_reset()
{
    for (int i = 0; i < NUM_I2C_PORTS; i++)
        if (port_info[i].state != STATE_INACTIVE)
            i2c_port_release(i);

    SIM_SCGC4 &= ~(SIM_SCGC4_I2C0 | SIM_SCGC4_I2C1);
}

i2c_port i2c_master_init(uint8_t pins, uint16_t attributes, uint32_t frequency)
{
    if (pins > I2C_PINS_MAX)
        return I2C_PORT_ERROR;

    uint8_t port = port_map[pins].i2c_port;
    if (port_info[port].state != STATE_INACTIVE)
        return I2C_PORT_ERROR;

    port_info[port].state = STATE_WAITING;
    port_info[port].pins = pins;

    if (port == 0) {
        SIM_SCGC4 |= SIM_SCGC4_I2C0;
    } else if (port == 1) {
        SIM_SCGC4 |= SIM_SCGC4_I2C1;
    }

    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);

    // configure as master
    i2c->C2 = I2C_C2_HDRS; // no effect on LC
    i2c->A1 = 0;
    i2c->RA = 0;

    // configure SCL
    PCR(port_map[pins].scl_port, port_map[pins].scl_pin) = PORT_PCR_MUX(port_map[pins].scl_alt) | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE;

    // configure SDA
    PCR(port_map[pins].sda_port, port_map[pins].sda_pin) = PORT_PCR_MUX(port_map[pins].sda_alt) | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE;

    set_frequency(i2c, port == 0 ? F_BUS : F_CPU, frequency);
    
    i2c->C1 = I2C_C1_IICEN;
    i2c->S = I2C_S_IICIF | I2C_S_ARBL;

    NVIC_ENABLE_IRQ(port == 0 ? IRQ_I2C0 : IRQ_I2C1);

    /*
    // configure DMA
    uint8_t dma = dma_acquire_channel();
    port_info[port].dma = dma;
    dma_disable_on_completion(dma);
    dma_attach_interrupt(dma, port == 0 ? dma_i2c0_isr : dma_i2c1_isr);
    dma_interrupt_on_completion(dma);
    dma_trigger_at_hw_evt(dma, port == 0 ? DMAMUX_SOURCE_I2C0 : DMAMUX_SOURCE_I2C1);
    */

    return port;
}

void i2c_port_release(i2c_port port)
{
    if (port >= NUM_I2C_PORTS)
        return;

    if (port_info[port].state == STATE_INACTIVE)
        return;
    
    //dma_release_channel(port_info[port].dma);

    uint8_t pins = port_info[port].pins;
    PCR(port_map[pins].scl_port, port_map[pins].scl_pin) = 0;
    PCR(port_map[pins].sda_port, port_map[pins].sda_pin) = 0;

    port_info[port].state = STATE_INACTIVE;
}


void i2c_port_reset(i2c_port port)
{

}


void i2c_master_start_send(wk_port_request* request)
{
    i2c_port port = request->port_id;

    // reset flags
    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    i2c->FLT |= I2C_FLT_STOPF | I2C_FLT_STARTF;
    i2c->FLT &= ~I2C_FLT_SSIE;
    i2c->S = I2C_S_IICIF | I2C_S_ARBL;

    // acquire bus
    if (acquire_bus(port) != 0) {
        send_write_completion(port, I2C_STATUS_TIMEOUT, 0);
        return;
    }

    port_info_t* pi = &port_info[port];
    pi->state = STATE_TX;
    pi->sub_state = SUB_STATE_ADDR;
    pi->request_id = request->request_id;
    pi->data_len = request->header.message_size - sizeof(wk_port_request) + 4;
    memcpy(pi->data, request->data, pi->data_len);
    pi->processed = 0;

    /*
    // setup DMA
    uint8_t dma = pi->dma;
    dma_source_byte_buffer(dma, data + 1, len - 2);
    dma_dest_byte(dma, &i2c->D);
    */

    // enable interrupt and send address (for writing)
    i2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX;
    i2c->D = (uint8_t)(request->action_attribute2 << 1);
}


void i2c_master_start_recv(wk_port_request* request)
{
    i2c_port port = request->port_id;

    // reset flags
    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    i2c->FLT |= I2C_FLT_STOPF | I2C_FLT_STARTF;
    i2c->FLT &= ~I2C_FLT_SSIE;
    i2c->S = I2C_S_IICIF | I2C_S_ARBL;

    port_info_t* pi = &port_info[port];

    // acquire bus
    if (acquire_bus(port) != 0) {
        send_read_completion(port, I2C_STATUS_TIMEOUT, pi->processed);
        return;
    }

    pi->state = STATE_RX;
    pi->sub_state = SUB_STATE_ADDR;
    pi->request_id = request->request_id;
    uint16_t* data_len_ptr = (uint16_t*)&request->data[0];
    pi->data_len = *data_len_ptr;
    pi->processed = 0;

    // enable interrupt and send address (for reading)
    i2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX;
    i2c->D = (uint8_t)((request->action_attribute2 << 1) | 1);
}


void i2c_isr_handler(uint8_t port)
{
    port_info_t* pi = &port_info[port];
    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    uint8_t status = i2c->S;
    uint8_t sub_state = pi->sub_state;
    uint8_t completion_status = 0xff;
    
    // master to slave transmission
    if (pi->state == STATE_TX) {

        // arbitration lost
        if (status & I2C_S_ARBL) {
            pi->state = STATE_ERROR;
            i2c->S = I2C_S_ARBL; // clear flag
            i2c->C1 = I2C_C1_IICEN; // reset to RX
            completion_status = I2C_STATUS_ARB_LOST;

        // no ACK received
        } else if (status & I2C_S_RXAK) {
            pi->state = STATE_ERROR;
            i2c->C1 = I2C_C1_IICEN; // reset to RX
            completion_status = sub_state == SUB_STATE_ADDR ? I2C_STATUS_ADDR_NAK : I2C_STATUS_DATA_NAK;

        // transmission is progressing
        } else {

            // address transmitted
            if (sub_state == SUB_STATE_ADDR) {
                //pi->state = STATE_TX_DMA_BULK;
                //i2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX | I2C_C1_DMAEN; // enable DMA
                //dma_enable(pi->dma);
                pi->sub_state = SUB_STATE_DATA;
                i2c->D = pi->data[0];

            /*
            // DMA has finished sending second to last byte
            } else if (state == STATE_TX_DMA_SND_LAST) {
                while (!(i2c->S & I2C_S_TCF)); // wait for transfer complete
                dma_clear_complete(port_info[port].dma);
                port_info[port].state = STATE_TX_INTR_LAST;
                i2c->D = port_info[port].buffer[port_info[port].buf_len - 1];
            */

            // transmission complete
            } else if (pi->processed == pi->data_len - 1) {
                pi->processed++;
                pi->state = STATE_WAITING;
                i2c->C1 = I2C_C1_IICEN; // reset to RX
                completion_status = I2C_STATUS_OK;

            // transmit next byte
            } else {
                pi->processed++;
                i2c->D = pi->data[pi->processed];
            }
        
        }

        if (completion_status != 0xff)
            send_write_completion(port, completion_status, pi->processed);    

    // master to slave receive
    } else if (pi->state == STATE_RX) {

        // address transmitted
        if (sub_state == SUB_STATE_ADDR) {

            // arbitration lost
            if (status & I2C_S_ARBL) {
                pi->state = STATE_ERROR;
                i2c->S = I2C_S_ARBL; // clear flag
                i2c->C1 = I2C_C1_IICEN; // reset to RX
                completion_status = I2C_STATUS_ARB_LOST;

            // no ACK received
            } else if (status & I2C_S_RXAK) {
                pi->state = STATE_ERROR;
                i2c->C1 = I2C_C1_IICEN; // reset to RX
                completion_status = sub_state == I2C_STATUS_ADDR_NAK;

            // slave address transmitted
            } else {
                pi->sub_state = SUB_STATE_DATA;
                i2c->C1 = pi->data_len > 1 ? I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST
                                        : I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TXAK; // set NAK
                uint8_t __attribute__((unused)) data = i2c->D;
            }
            
        // received byte
        } else {

            // second to last byte: set NAK
            if (pi->processed == pi->data_len - 2) {
                i2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TXAK;
            }
                
            // receive completed
            if (pi->processed == pi->data_len - 1) {
                i2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
                pi->data[pi->processed++] = i2c->D;
                for (int i = 0; i < 20; i++) {
                    __asm__ volatile ("nop");
                }
                i2c->C1 = I2C_C1_IICEN;
                completion_status = I2C_STATUS_OK;
                
            // receive next byte
            } else {
                pi->data[pi->processed++] = i2c->D;
            }
        }

        if (completion_status != 0xff)
            send_read_completion(port, completion_status, pi->processed);    
    
    } else {
        DEBUG_OUT("Spurious I2C interrupt");
    }

    i2c->S = I2C_S_IICIF; // clear interrupt flag
}


uint8_t acquire_bus(i2c_port port)
{
    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    if (i2c->C1 & I2C_C1_MST) {
        i2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_RSTA | I2C_C1_TX;
    } else {
        while (1) { // TODO: put into background
            if (!(i2c->S & I2C_S_BUSY)) {
                i2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
                break;
            }
        }
        if (!(i2c->C1 & I2C_C1_MST)) {
            port_info[port].state = STATE_ERROR;
            return 1;
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

    // binary search
    int lower = 0;
    int upper = NUM_F_DIVS;
    while (lower < upper) {
        int mid = (upper + lower) / 2;
        if (freq_divs[mid].divider < target_div)
            lower = mid + 1;
        else
            upper = mid;
    }

    // compare two closest results
    int idx = lower;
    if (idx == NUM_F_DIVS)
        idx--;
    else if (idx >= 1 && target_div - freq_divs[idx-1].divider < freq_divs[idx].divider - target_div)
        idx--;

    i2c->F = freq_divs[idx].f_div;
    i2c->FLT = bus_rate >= 48000000 ? 4 : bus_rate / 12000000;
}


/*
void dma_isr_handler(uint8_t port)
{
    KINETIS_I2C_t* i2c = get_i2c_ctrl(port);
    uint8_t state = port_info[port].state;
    uint8_t dma = port_info[port].dma;
    
    // master to slave transmission
    if (STATE_IS_TX(state)) {

        // DMA starts to send sencond to last byte
        if (state == STATE_TX_DMA_BULK && dma_is_complete(dma)) {
            dma_clear_interrupt(dma);
            i2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX;
            port_info[port].state = STATE_TX_DMA_SND_LAST;

        } else if (dma_is_error(dma)) {
            dma_clear_interrupt(dma);
            dma_clear_error(dma);

            if (i2c->S & I2C_S_ARBL) {
                // TODO
            }
            send_write_completion(port, port_info[port].request_id, I2C_STATUS_ARB_LOST);
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
*/


void i2c0_isr()
{
    i2c_isr_handler(0);
}


void i2c1_isr()
{
    i2c_isr_handler(1);
}


void send_write_completion(i2c_port port, uint8_t status, uint16_t len)
{
    port_info_t* pi = &port_info[port];
    wk_send_port_event_2(PORT_GROUP_I2C | port, WK_EVENT_TX_COMPLETE, pi->request_id, NULL, 0, status, len);
}


void send_read_completion(i2c_port port, uint8_t status, uint16_t len)
{
    port_info_t* pi = &port_info[port];
    wk_send_port_event_2(PORT_GROUP_I2C | port, WK_EVENT_DATA_RECV, pi->request_id, pi->data, pi->processed, status, len);
}
