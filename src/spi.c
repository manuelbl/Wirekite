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
#include "ports.h"
#include "usb.h"
#include "util.h"
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

#if defined(__MKL26Z64__)

#define NUM_SPI_PORTS 2

static const port_map_t SCK_map[] = {
    { 13, PORT_C,  5, 2, 0 }, // PTC5
    { 14, PORT_D,  1, 2, 0 }, // PTD1
    { 20, PORT_D,  5, 2, 1 }  // PTD5
};

static const port_map_t MOSI_map[] = {
    { 11, PORT_C,  6, 2, 0 }, // PTC6
    {  7, PORT_D,  2, 2, 0 }, // PTD2
    {  0, PORT_B, 16, 2, 1 }, // PTB16
    { 21, PORT_D,  6, 2, 1 }  // PTD6
};    

static const port_map_t MISO_map[] = {
    { 12, PORT_C,  7, 2, 0 }, // PTC7
    {  8, PORT_D,  3, 2, 0 }, // PTD3
    {  1, PORT_B, 17, 2, 1 }, // PTB17
    {  5, PORT_D,  7, 2, 1 }  // PTD7
};

#elif defined(__MK20DX256__)

// In the MK20DX256, only SPI 0 module works.
// SPI 1 seems to exist but has no SCK pin!

#define NUM_SPI_PORTS 2

static const port_map_t SCK_map[] = {
    { 13, PORT_C,  5, 2, 0 }, // PTC5
    { 14, PORT_D,  1, 2, 0 }  // PTD1
};

static const port_map_t MOSI_map[] = {
//    { 26, PORT_E,  1, 2, 1 }, // PTE1
//    {  0, PORT_B, 16, 2, 1 }, // PTB16
    { 11, PORT_C,  6, 2, 0 }, // PTC6
    {  7, PORT_D,  2, 2, 0 }  // PTD2
};    

static const port_map_t MISO_map[] = {
//    { 26, PORT_E,  1, 7, 1 }, // PTE1
//    {  1, PORT_B, 17, 2, 1 }, // PTB17
    { 12, PORT_C,  7, 2, 0 }, // PTC7
    {  8, PORT_D,  3, 2, 0 }  // PTD3
};

#endif


#define NUM_SCK (sizeof(SCK_map)/sizeof(SCK_map[0]))
#define NUM_MOSI (sizeof(MOSI_map)/sizeof(MOSI_map[0]))
#define NUM_MISO (sizeof(MISO_map)/sizeof(MISO_map[0]))


#if defined(__MKL26Z64__)

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

#elif defined(__MK20DX256__)

#define F_DIV_DBR_MASK 0x40
#define F_DIV_DBR_OFFSET 6
#define F_DIV_PBR_MASK 0x30
#define F_DIV_PBR_OFFSET 4
#define F_DIV_BR_MASK 0x0f
#define F_DIV_BR_OFFSET 0

static const freq_div_t freq_divs[] = {
    { 0x40, 2 },
    { 0x50, 3 },
    { 0x00, 4 },
    { 0x60, 5 },
    { 0x10, 6 },
    { 0x70, 7 },
    { 0x01, 8 },
    { 0x52, 9 },
    { 0x20, 11 },
    { 0x11, 13 },
    { 0x30, 14 },
    { 0x62, 15 },
    { 0x03, 17 },
    { 0x12, 19 },
    { 0x21, 20 },
    { 0x72, 22 },
    { 0x13, 26 },
    { 0x31, 29 },
    { 0x22, 31 },
    { 0x04, 36 },
    { 0x23, 41 },
    { 0x32, 45 },
    { 0x14, 52 },
    { 0x33, 60 },
    { 0x05, 72 },
    { 0x24, 88 },
    { 0x15, 104 },
    { 0x34, 120 },
    { 0x06, 143 },
    { 0x25, 175 },
    { 0x16, 207 },
    { 0x35, 239 },
    { 0x07, 286 },
    { 0x26, 351 },
    { 0x17, 415 },
    { 0x36, 479 },
    { 0x08, 572 },
    { 0x27, 701 },
    { 0x18, 830 },
    { 0x37, 958 },
    { 0x09, 1145 },
    { 0x28, 1402 },
    { 0x19, 1659 },
    { 0x38, 1916 },
    { 0x0A, 2290 },
    { 0x29, 2804 },
    { 0x1A, 3318 },
    { 0x39, 3831 },
    { 0x0B, 4579 },
    { 0x2A, 5609 },
    { 0x1B, 6636 },
    { 0x3A, 7663 },
    { 0x0C, 9159 },
    { 0x2B, 11217 },
    { 0x1C, 13273 },
    { 0x3B, 15326 },
    { 0x0D, 18318 },
    { 0x2C, 22435 },
    { 0x1D, 26545 },
    { 0x3C, 30652 },
    { 0x0E, 0 }
};

static const freq_div_t delay_divs[] = {
    { 0x00, 4 },
    { 0x01, 8 },
    { 0x10, 12 },
    { 0x02, 16 },
    { 0x20, 20 },
    { 0x11, 24 },
    { 0x30, 28 },
    { 0x03, 32 },
    { 0x21, 40 },
    { 0x12, 48 },
    { 0x31, 56 },
    { 0x04, 64 },
    { 0x22, 80 },
    { 0x13, 96 },
    { 0x32, 112 },
    { 0x05, 128 },
    { 0x23, 160 },
    { 0x14, 192 },
    { 0x33, 224 },
    { 0x06, 256 },
    { 0x24, 320 },
    { 0x15, 384 },
    { 0x34, 448 },
    { 0x07, 512 },
    { 0x25, 640 },
    { 0x16, 768 },
    { 0x35, 896 },
    { 0x08, 1024 },
    { 0x26, 1280 },
    { 0x17, 1536 },
    { 0x36, 1792 },
    { 0x09, 2048 },
    { 0x27, 2560 },
    { 0x18, 3072 },
    { 0x37, 3584 },
    { 0x0A, 4096 },
    { 0x28, 5120 },
    { 0x19, 6144 },
    { 0x38, 7168 },
    { 0x0B, 8192 },
    { 0x29, 10240 },
    { 0x1A, 12288 },
    { 0x39, 14336 },
    { 0x0C, 18384 },
    { 0x2A, 20480 },
    { 0x1B, 24576 },
    { 0x3A, 28672 },
    { 0x0D, 32768 },
    { 0x2B, 40960 },
    { 0x1C, 55152 },
    { 0x3B, 57344 },
    { 0x0E, 0 }
};

#define NUM_DELAY_DIVS (sizeof(delay_divs)/sizeof(delay_divs[0]))

#endif

#define NUM_F_DIVS (sizeof(freq_divs)/sizeof(freq_divs[0]))


#define STATE_INACTIVE 0
#define STATE_IDLE  1
#define STATE_TX 2
#define STATE_RX 3
#define STATE_TX_N_RX 4
#define STATE_ERROR 5


#define CIRC_QUEUE_SIZE 20


typedef struct {
    union {
        wk_port_request* request;
        wk_port_event* response;
    };
    uint16_t tx_processed;
    uint16_t rx_processed;
    uint8_t circ_request_head;
    uint8_t circ_request_tail;
    uint8_t dma_tx;
    uint8_t dma_rx;
    uint8_t state;
    uint8_t sck;
    uint8_t mosi;
    uint8_t miso;
    wk_port_request* circ_request_buf[CIRC_QUEUE_SIZE];
} spi_port_info_t;

static spi_port_info_t port_info[NUM_SPI_PORTS];


#if defined(__MKL26Z64__)
#define SPI_t KINETISL_SPI_t
#elif defined(__MK20DX256__)
#define SPI_t KINETISK_SPI_t
#endif
static SPI_t* get_spi_ctrl(spi_port port);
static void set_frequency(SPI_t* spi, uint32_t bus_rate, uint32_t frequency);
static void dma_spi0_tx_isr();
static void dma_spi0_rx_isr();
static void dma_spi1_tx_isr();
static void dma_spi1_rx_isr();
static void master_start_rx_now(wk_port_request* request);
static void master_start_trx(wk_port_request* request, uint8_t new_state);
static void set_digital_output_2(wk_port_request* request);
static void trx_complete(spi_port port, uint8_t status, uint16_t len);
static wk_port_request* get_next_request(spi_port port);
static void append_request(spi_port port, wk_port_request* request);
static void check_queue(spi_port port);
static void clear_request_queue(spi_port port);
static void spi_isr_handler(uint8_t port);
static wk_port_event* create_response(uint16_t port_id, uint16_t request_id, uint16_t rx_size, uint16_t cs);
static void convert_to_response(wk_port_request* request);


void spi_init()
{
    // nothing to do
}

void spi_reset()
{
    for (int i = 0; i < NUM_SPI_PORTS; i++)
        if (port_info[i].state != STATE_INACTIVE)
            spi_port_release(i);

#if defined(__MKL26Z64__)
    SIM_SCGC4 &= ~(SIM_SCGC4_SPI0 | SIM_SCGC4_SPI1);
#elif defined(__MK20DX256__)
    SIM_SCGC6 &= ~(SIM_SCGC6_SPI0 | SIM_SCGC6_SPI1);
#endif
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
    pi->state = STATE_IDLE;
    pi->sck = sck_port;
    pi->mosi = mosi_port;
    pi->miso = miso_port;

    // enable clock
#if defined(__MKL26Z64__)
    SIM_SCGC4 |= spi_port == 0 ? SIM_SCGC4_SPI0 : SIM_SCGC4_SPI1;
#elif defined(__MK20DX256__)
    SIM_SCGC6 |= spi_port == 0 ? SIM_SCGC6_SPI0 : SIM_SCGC6_SPI1;
#endif

    // initialize SPI module
    SPI_t* spi = get_spi_ctrl(spi_port);

#if defined(__MKL26Z64__)

    uint8_t c1 = SPI_C1_MSTR | SPI_C1_SPE;
    if ((attributes & SPI_CONFIG_MODE_MASK) == SPI_CONFIG_MODE2 || (attributes & SPI_CONFIG_MODE_MASK) == SPI_CONFIG_MODE3)
        c1 |= SPI_C1_CPOL;
    if ((attributes & SPI_CONFIG_MODE_MASK) == SPI_CONFIG_MODE1 || (attributes & SPI_CONFIG_MODE_MASK) == SPI_CONFIG_MODE3)
        c1 |= SPI_C1_CPHA;
    if ((attributes & SPI_CONFIG_LSB_FIRST) != 0)
        c1 |= SPI_C1_LSBFE;
    spi->C1 = c1;

    uint8_t c2 = 0;
    if ((attributes & SPI_CONFIG_BITS_MASK) == SPI_CONFIG_16_BIT)
        c2 |= SPI_C2_SPIMODE;
    spi->C2 = c2;

    set_frequency(spi, spi_port == 0 ? F_BUS : F_CPU, frequency);
    
#elif defined(__MK20DX256__)

    spi->MCR = SPI_MCR_MSTR | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF | SPI_MCR_HALT | SPI_MCR_DCONF(0);
    uint32_t ctar = SPI_CTAR_FMSZ(7);
    if ((attributes & SPI_CONFIG_MODE_MASK) == SPI_CONFIG_MODE2 || (attributes & SPI_CONFIG_MODE_MASK) == SPI_CONFIG_MODE3)
        ctar |= SPI_CTAR_CPOL;
    if ((attributes & SPI_CONFIG_MODE_MASK) == SPI_CONFIG_MODE1 || (attributes & SPI_CONFIG_MODE_MASK) == SPI_CONFIG_MODE3)
        ctar |= SPI_CTAR_CPHA;
    if ((attributes & SPI_CONFIG_LSB_FIRST) != 0)
        ctar |= SPI_CTAR_LSBFE;
    spi->CTAR0 = ctar;

    set_frequency(spi, F_BUS, frequency);
    
#endif

    // configure SCK
    PCR(SCK_map[sck_port].port, SCK_map[sck_port].pin) = PORT_PCR_MUX(SCK_map[sck_port].alt);
    
    // configure MOSI
    PCR(MOSI_map[mosi_port].port, MOSI_map[mosi_port].pin) = PORT_PCR_MUX(MOSI_map[mosi_port].alt);
    
    // configure MISO
    if (miso_port != 0xff)
        PCR(MISO_map[miso_port].port, MISO_map[miso_port].pin) = PORT_PCR_MUX(MISO_map[miso_port].alt);
    
    // Enable interrupt
    NVIC_CLEAR_PENDING(spi_port == 0 ? IRQ_SPI0 : IRQ_SPI1);
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


// Takes ownership of request; releases it when the transmission is done
void spi_master_start_send(wk_port_request* request)
{
    spi_port port = (uint8_t) request->header.port_id;

    if (port >= NUM_SPI_PORTS || port_info[port].state == STATE_INACTIVE) {
        // not configured
        mm_free(request);
    } else if (port_info[port].state != STATE_IDLE) {
        // if SPI port busy then queue request
        append_request(port, request);
    } else {
        uint8_t new_state = STATE_TX;
        if (request->action == WK_PORT_ACTION_TX_N_RX_DATA) {
            convert_to_response(request);
            new_state = STATE_TX_N_RX;
        }
        master_start_trx(request, new_state);
    }
}


// Does NOT take ownership of request
void spi_master_start_recv(wk_port_request* request)
{
    spi_port port = (uint8_t) request->header.port_id;    
    if (port >= NUM_SPI_PORTS || port_info[port].state == STATE_INACTIVE)
        return; // not configured

    // if port is busy copy the request and queue it
    if (port_info[port].state != STATE_IDLE) {

        wk_port_request* copy = mm_alloc(request->header.message_size);
        if (copy == NULL) {
            DEBUG_OUT("SPI insufficient memory");
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
    spi_port port = (uint8_t)request->header.port_id;
    
    // allocate response with RX buffer
    spi_port_info_t* pi = &port_info[port];
    pi->response = create_response(request->header.port_id, request->header.request_id,
        (uint16_t)request->value1, request->action_attribute2);
    if (pi->response == NULL)
        return;

    // SPI is a full-duplex protocol - always. So even though we're just reading,
    // we also need to initialize the data as it is sent at the same time.
    if (request->action_attribute1 != 0)
        memset(pi->response->data, request->action_attribute1, request->value1);

    // start receive (wk_port_request and wk_port_event have the same layout; so we pass one for the other)
    master_start_trx(pi->request, STATE_RX);
}


void master_start_trx(wk_port_request* request, uint8_t new_state)
{
    // takes ownership of request; releases it when the transmission is done

    // chip select
    digital_pin cs = request->action_attribute2;
    if (cs != DIGI_PIN_ERROR)
        digital_pin_set_output(cs, 0);
    
    // initialize state
    spi_port port = (uint8_t) request->header.port_id;
    spi_port_info_t* pi = &port_info[port];
    pi->state = new_state;
    pi->request = request;
    pi->tx_processed = 0;
    pi->rx_processed = 0;
    
    // setup DMA
    SPI_t* spi = get_spi_ctrl(port);
    uint8_t dma_tx = pi->dma_tx;
    uint8_t dma_rx = pi->dma_rx;
    
    uint16_t data_len = WK_PORT_REQUEST_DATA_LEN(pi->request);
    int use_dma = dma_tx != DMA_CHANNEL_ERROR && data_len > 3;

#if defined(__MKL26Z64__)

    if (use_dma) {
        // switch to DMA for bulk of data
        dma_source_byte_buffer(dma_tx, request->data + 1, data_len - 1);
        dma_dest_byte(dma_tx, &spi->DL);
        dma_source_byte(dma_rx, &spi->DL);
        dma_dest_byte_buffer(dma_rx, request->data, data_len);
        // DMA will start after first byte    
    } else {
        spi->C1 |= SPI_C1_SPIE;
    }

    // transmit first byte
    while ((spi->S & SPI_S_SPTEF) == 0);
    spi->DL = request->data[0];

    if (use_dma) {
        spi->C2 |= SPI_C2_RXDMAE | SPI_C2_TXDMAE;
        dma_enable(dma_rx);
        dma_enable(dma_tx);
    }

#elif defined(__MK20DX256__)
    
    if (use_dma) {
        // prepare DMA
        dma_source_byte_buffer(dma_tx, request->data, data_len);
        dma_dest_byte(dma_tx, (volatile uint8_t*)&spi->PUSHR);
        dma_source_byte(dma_rx, (volatile uint8_t*)&spi->POPR);
        dma_dest_byte_buffer(dma_rx, request->data, data_len);

        // enable DMA
        spi->RSER = SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS | SPI_RSER_TFFF_RE | SPI_RSER_TFFF_DIRS;
        dma_enable(dma_rx);
        dma_enable(dma_tx);

        // enable SPI module
        spi->SR = SPI_SR_TCF | SPI_SR_EOQF | SPI_SR_TFUF | SPI_SR_TFFF | SPI_SR_RFOF | SPI_SR_RFDF;
        spi->MCR &= ~SPI_MCR_HALT; // start SPI module

    } else {

        spi->RSER = SPI_RSER_RFDF_RE;
        spi->SR = SPI_SR_TCF | SPI_SR_EOQF | SPI_SR_TFUF | SPI_SR_TFFF | SPI_SR_RFOF | SPI_SR_RFDF;

        // enable SPI module
        spi->MCR &= ~SPI_MCR_HALT; // start SPI module

        // push first bytes into FIFO
        while (pi->tx_processed < data_len && (spi->SR & SPI_SR_TFFF) != 0) {
            uint32_t pushr = SPI_PUSHR_CTAS(0) | request->data[pi->tx_processed++];
            spi->PUSHR = pushr;
            spi->SR = SPI_SR_TFFF;
        }
        if (pi->tx_processed < data_len)
            spi->RSER |= SPI_RSER_TFFF_RE;
    }

#endif
}


void spi_set_digital_output(wk_port_request* request)
{
    // take ownership of request; release it when the transmission is done
    spi_port port = (uint8_t) request->action_attribute2;

    if (port_info[port].state == STATE_INACTIVE) {
        // not configured
        mm_free(request);
    } else if (port_info[port].state != STATE_IDLE) {
        // if SPI port busy then queue request
        append_request(port, request);
    } else {
        set_digital_output_2(request);
    }
}


void set_digital_output_2(wk_port_request* request)
{
    digital_pin pin = request->header.port_id & PORT_GROUP_DETAIL_MASK;
    digital_pin_set_output(pin, (uint8_t)request->value1);

    // free request
    uint16_t request_id = request->header.request_id;
    uint16_t port_id = request->header.port_id;
    mm_free(request);

    // send completion message
    wk_send_port_event_2(port_id, WK_EVENT_SET_DONE, request_id, 0, 0, 0, NULL, 0);
}


void set_frequency(SPI_t* spi, uint32_t bus_rate, uint32_t frequency)
{
    uint16_t target_div = (bus_rate + frequency / 2) / frequency;
    int idx = frequency_lookup(freq_divs, NUM_F_DIVS, target_div);

#if defined(__MKL26Z64__)

    spi->BR = freq_divs[idx].f_div;

#elif defined(__MK20DX256__)

    uint32_t fdiv = freq_divs[idx].f_div;
    uint32_t ctar = spi->CTAR0;
    if ((fdiv & F_DIV_DBR_MASK) != 0)
        ctar |= SPI_CTAR_DBR;
    uint32_t pbr = (fdiv & F_DIV_PBR_MASK) >> F_DIV_PBR_OFFSET;
    uint32_t br = (fdiv & F_DIV_BR_MASK) >> F_DIV_BR_OFFSET;
    ctar |= SPI_CTAR_PBR(pbr) | SPI_CTAR_BR(br);

    // Select a delay after the last bit.
    // The goal is to get half a cycle, i.e. half of the divider.
    // If the exact value is not available, the next bigger
    // divider is selected so the delay is not shorter than half a cycle.
    // The missing doubler and differences in the scaler values make it
    // slightly challenging to find the best value.
    static const uint8_t pbrs[] = { 2, 3, 5, 7 };
    uint32_t divider = pbrs[pbr] * (br < 2 ? (2 << br) : (br == 2 ? 6 : (1 << br)));
    if ((fdiv & F_DIV_DBR_MASK) != 0)
        divider >>= 1;
    int idx2 = frequency_lookup(delay_divs, NUM_DELAY_DIVS, divider);
    uint32_t del_div = delay_divs[idx2].f_div;
    pbr = (del_div & F_DIV_PBR_MASK) >> F_DIV_PBR_OFFSET;
    br = (del_div & F_DIV_BR_MASK) >> F_DIV_BR_OFFSET;

    if (ctar & SPI_CTAR_CPHA)
        ctar |= SPI_CTAR_PASC(pbr) | SPI_CTAR_ASC(br);
    else
        ctar |= SPI_CTAR_PCSSCK(pbr) | SPI_CTAR_CSSCK(br);
    spi->CTAR0 = ctar;

#endif 
}


void spi_port_release(spi_port port)
{
    if (port >= NUM_SPI_PORTS)
        return;

    spi_port_info_t* pi = &port_info[port];
    if (pi->state == STATE_INACTIVE)
        return;

    pi->state = STATE_INACTIVE;

    SPI_t* spi = get_spi_ctrl(port);

#if defined(__MKL26Z64__)
    spi->C1 = 0;
    spi->C2 = 0;
#elif defined(__MK20DX256__)
    spi->MCR |= SPI_MCR_HALT;
    spi->MCR = SPI_MCR_MSTR | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF | SPI_MCR_HALT | SPI_MCR_HALT | SPI_MCR_DCONF(0);
    spi->SR = SPI_SR_TCF | SPI_SR_EOQF | SPI_SR_TFUF | SPI_SR_TFFF | SPI_SR_RFOF | SPI_SR_RFDF;
    spi->RSER = 0;
#endif    
    
    if (pi->dma_tx != DMA_CHANNEL_ERROR)
        dma_release_channel(pi->dma_tx);
    if (pi->dma_rx != DMA_CHANNEL_ERROR)
        dma_release_channel(pi->dma_rx);

    NVIC_DISABLE_IRQ(port == 0 ? IRQ_SPI0 : IRQ_SPI1);
        
    wk_port_request* request = pi->request;
    if (request != NULL) {
        digital_pin cs = request->action_attribute2;
        if (cs != DIGI_PIN_ERROR)
            digital_pin_set_output(cs, 1);
        mm_free(request);
        pi->request = NULL;
    }
    
    PCR(SCK_map[pi->sck].port, SCK_map[pi->sck].pin) = 0;
    PCR(MOSI_map[pi->mosi].port, MOSI_map[pi->mosi].pin) = 0;
    if (pi->miso != 0xff)
        PCR(MISO_map[pi->miso].port, MISO_map[pi->miso].pin) = 0;
    
    clear_request_queue(port);
}


void dma_tx_isr_handler(uint8_t port)
{
    SPI_t* spi = get_spi_ctrl(port);
    spi_port_info_t* pi = &port_info[port];
    uint8_t dma = pi->dma_tx;

    if (dma == DMA_CHANNEL_ERROR)
        return; // oops

    if (dma_is_error(dma)) {
        // If this error handling code is ever executed, it is most likely a software bug.
        // In SPI, there are no ACKs. And the master cannot recognize if the slave
        // has received or transmitted something or even exists. So no error can occur.
        DEBUG_OUT("SPI DMA TX error");

#if defined(__MKL26Z64__)
        uint32_t processed = WK_PORT_REQUEST_DATA_LEN(pi->request) - dma_bytes_remaining(dma) + 1;
#elif defined(__MK20DX256__)
        uint32_t processed = WK_PORT_REQUEST_DATA_LEN(pi->request) - dma_bytes_remaining(dma);
#endif
        dma_clear_error(dma);

        // disable RX DMAs
        uint8_t dma_rx = pi->dma_rx;
        if (dma_rx != DMA_CHANNEL_ERROR) {
            dma_disable(dma_rx);
            dma_clear_error(dma_rx);
            dma_clear_interrupt(dma_rx);
        }
        
        // disable DMA
#if defined(__MKL26Z64__)
        spi->C2 &= ~(SPI_C2_TXDMAE | SPI_C2_RXDMAE);
#elif defined(__MK20DX256__)
        spi->MCR |= SPI_MCR_HALT; // stop SPI module
        spi->RSER = 0;
        spi->SR = SPI_SR_TCF | SPI_SR_EOQF | SPI_SR_TFUF | SPI_SR_TFFF | SPI_SR_RFOF | SPI_SR_RFDF;
#endif

        trx_complete(port, SPI_STATUS_UNKNOWN, processed);

    // DMA has written last byte to data register; SPI starts to send last byte
    } else if (dma_is_complete(dma)) {
        // disable TX DMA
#if defined(__MKL26Z64__)
        spi->C2 &= ~SPI_C2_TXDMAE;
#elif defined(__MK20DX256__)
        spi->RSER &= ~(SPI_RSER_TFFF_RE | SPI_RSER_TFFF_DIRS);
#endif
        dma_clear_complete(dma);
        // no further action; the last byte is still being received

    } else {
        // This occurs on the Teensy 3.2 from time to time.
        // Very difficult to debug because it's intermittent
        // and tends to disappear if debugging code is added here.
        // It occurs after a TX DMA transfer has completed
        // but before the RX DMA for the same SPI transaction completes.
        DEBUG_OUT("Spurious SPI DMA TX interrupt");
        dma_clear_interrupt(dma);
        return; // oops
    }

    // disable TX DMA
    dma_disable(dma);
    dma_clear_interrupt(dma);
}


void dma_rx_isr_handler(uint8_t port)
{
    SPI_t* spi = get_spi_ctrl(port);
    spi_port_info_t* pi = &port_info[port];
    uint8_t dma = pi->dma_rx;

    if (dma == DMA_CHANNEL_ERROR)
        return; // oops

    uint8_t status;

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
#if defined(__MKL26Z64__)
    spi->C2 &= ~(SPI_C2_TXDMAE | SPI_C2_RXDMAE);
#elif defined(__MK20DX256__)
    spi->MCR |= SPI_MCR_HALT; // stop SPI module
    spi->RSER = 0;
    spi->SR = SPI_SR_TCF | SPI_SR_EOQF | SPI_SR_TFUF | SPI_SR_TFFF | SPI_SR_RFOF | SPI_SR_RFDF;
#endif
    dma_disable(dma);
    dma_clear_interrupt(dma);

    trx_complete(port, status, WK_PORT_REQUEST_DATA_LEN(pi->request));
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


void trx_complete(spi_port port, uint8_t status, uint16_t len)
{
    // chip select
    spi_port_info_t* pi = &port_info[port];
    wk_port_request* request = pi->request;
    digital_pin cs = request->action_attribute2;
    if (cs != DIGI_PIN_ERROR)
        digital_pin_set_output(cs, 1);
    
    if (pi->state == STATE_TX) {
        // save relevant values
        uint16_t port_id = request->header.port_id;
        uint16_t request_id = request->header.request_id;

        // free request
        mm_free(request);
        
        // send completion message
        wk_send_port_event_2(port_id, WK_EVENT_TX_COMPLETE, request_id, status, len, 0, NULL, 0);

    } else { // STATE_RX and STATE_TX_N_RX
        wk_port_event* response = pi->response;
        response->event_attribute1 = status;
        response->event_attribute2 = len;
        response->header.message_size = WK_PORT_EVENT_ALLOC_SIZE(len);

        // send response
        endp1_tx_msg(&response->header);
    }

    // cleanup
    pi->request = NULL;
    pi->state = STATE_IDLE;
    pi->tx_processed = 0;
    pi->rx_processed = 0;

    check_queue(port);
}


// check for pending request in queue
void check_queue(spi_port port)
{
    while (1) {
        wk_port_request* request = get_next_request(port);
        if (request == NULL)
            return;

        if (request->action == WK_PORT_ACTION_TX_DATA) {
            master_start_trx(request, STATE_TX);
            return;
        } else if (request->action == WK_PORT_ACTION_TX_N_RX_DATA) {
            convert_to_response(request);
            master_start_trx(request, STATE_TX_N_RX);
            return;
        } else if (request->action == WK_PORT_ACTION_RX_DATA) {
            master_start_rx_now(request);
            mm_free(request);
            return;
        } else if (request->action == WK_PORT_ACTION_SET_VALUE) {
            set_digital_output_2(request);
        } else {
            DEBUG_OUT("Invalid request in SPI queue");
        }
    }
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
    SPI_t* spi = get_spi_ctrl(port);

    wk_port_request* request = pi->request;

#if defined(__MKL26Z64__)

    // read received data byte;
    // must first read status register even though we know
    // data byte is ready as interrupt was triggered;
    // replaces: while ((spi->S & SPI_S_SPRF) == 0);
    uint8_t __attribute__((unused)) dummy = spi->S;
    int processed = pi->rx_processed;
    request->data[processed] = spi->DL;
    processed++;
    pi->rx_processed = (uint16_t)processed;
    
    int data_len = WK_PORT_REQUEST_DATA_LEN(pi->request);
    if (processed == data_len) {
        // request is complete
        spi->C1 &= ~SPI_C1_SPIE;
        trx_complete(port, SPI_STATUS_OK, processed);    
        
    } else {
        // transmit next data byte;
        // must first read status register even though we know register is
        // ready to receive data as writes are interleaved with reads;
        // replaces: while ((spi->S & SPI_S_SPTEF) == 0);
        uint8_t __attribute__((unused)) dummy = spi->S;
        spi->DL = request->data[processed];
    }

#elif defined(__MK20DX256__)

    int data_len = WK_PORT_REQUEST_DATA_LEN(pi->request);

    // read bytes in RX FIFO
    int processed = pi->rx_processed;
    while (processed < data_len && (spi->SR & SPI_SR_RFDF) != 0) {
        request->data[processed] = (uint8_t)spi->POPR;
        spi->SR = SPI_SR_RFDF;
        processed++;
    }
    pi->rx_processed = processed;

    // write bytes to TX FIFO
    processed = pi->tx_processed;
    while (processed < data_len && (spi->SR & SPI_SR_TFFF) != 0) {
        uint32_t pushr = SPI_PUSHR_CTAS(0) | request->data[processed];
        spi->PUSHR = pushr;
        spi->SR = SPI_SR_TFFF;
        processed++;
    }
    pi->tx_processed = processed;

    // done?
    if (pi->rx_processed == data_len && pi->tx_processed == data_len) {
        spi->MCR |= SPI_MCR_HALT; // stop SPI module
        spi->RSER = 0;
        spi->SR = SPI_SR_TCF | SPI_SR_EOQF | SPI_SR_TFUF | SPI_SR_TFFF | SPI_SR_RFOF | SPI_SR_RFDF;
        trx_complete(port, SPI_STATUS_OK, processed);
    }

#endif
}


// Caller must take owernship of returned object
wk_port_event* create_response(uint16_t port_id, uint16_t request_id, uint16_t rx_size, uint16_t cs)
{
    // allocate response message and copy data
    uint16_t msg_size = WK_PORT_EVENT_ALLOC_SIZE(rx_size);
    wk_port_event* response = mm_alloc(msg_size);
    if (response == NULL) {
        DEBUG_OUT("SPI RX insufficient mem");
        wk_send_port_event_2(port_id, WK_EVENT_DATA_RECV, request_id, SPI_STATUS_OUT_OF_MEMORY, 0, 0, NULL, 0);
        return NULL;
    }
        
    response->header.message_size = msg_size;
    response->header.message_type = WK_MSG_TYPE_PORT_EVENT;
    response->header.port_id = port_id;
    response->header.request_id = request_id;
    response->event = WK_EVENT_DATA_RECV;
    response->event_attribute2 = cs; // will later be overwritten by status
    return response;
}


// In-place conversion to response (they have the same layout and - in this
// special case - must have the same size)
void convert_to_response(wk_port_request* request)
{
    request->header.message_type = WK_MSG_TYPE_PORT_EVENT;
    request->action = WK_EVENT_DATA_RECV;
    request->action_attribute1 = 0; // for the moment
}


static SPI_t* SPI_CTRL[] = {
#if defined(__MKL26Z64__)
    &KINETISL_SPI0,
    &KINETISL_SPI1
#elif defined(__MK20DX256__)
    &KINETISK_SPI0
#endif
    
};

SPI_t* get_spi_ctrl(uint8_t port)
{
    return SPI_CTRL[port];
}
