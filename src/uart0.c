/**
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include "uart.h"
#include "kinetis.h"
#include "yield.h"


#define IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256

#define C2_ENABLE           UART_C2_TE | UART_C2_RE | UART_C2_RIE
#define C2_TX_ACTIVE		C2_ENABLE | UART_C2_TIE
#define C2_TX_COMPLETING	C2_ENABLE | UART_C2_TCIE
#define C2_TX_IIDLE 		C2_ENABLE

static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_buffer_head = 0;
static volatile uint16_t rx_buffer_tail = 0;
static volatile uint8_t is_transmitting = 0;
static volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
static volatile uint16_t tx_buffer_head = 0;
static volatile uint16_t tx_buffer_tail = 0;
static uart_event_func_t rx_evt_func;

static void uart_putchar(int32_t c);
static int32_t uart_getchar();
static void yield_func();


void uart0_init(int32_t baudrate)
{
    // enable the clock
    SIM_SCGC4 |= SIM_SCGC4_UART0;

    // configure pins
    PORTB_PCR16 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); // pin 0
    PORTB_PCR17 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); // pin 1

   // disable transmitter and receiver
    UART0_C2 = 0;

    // 8 bit, no parity
    UART0_C1 = 0;

    // set baud rate (sbr and osr) bits
    int32_t divisor = (F_PLL / 2 / 16 + (baudrate >> 1)) / baudrate;
	UART0_BDH = (divisor >> 8) & 0x1F;
	UART0_BDL = divisor & 0xFF;

    // enable transmitter and receiver
    UART0_C2 = C2_TX_IIDLE;

    // register yield func
    yield_add_func(yield_func);

    // enable interrupts
    NVIC_CLEAR_PENDING(IRQ_UART0_STATUS);
    NVIC_SET_PRIORITY(IRQ_UART0_STATUS, IRQ_PRIORITY);
    NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);
}


void uart0_set_recv_evt(uart_event_func_t func)
{
    rx_evt_func = func;
}


void uart0_end()
{
    if (!(SIM_SCGC4 & SIM_SCGC4_UART0))
        return;

    while (is_transmitting)
        yield();
    
    NVIC_CLEAR_PENDING(IRQ_UART0_STATUS);
    NVIC_DISABLE_IRQ(IRQ_UART0_STATUS);
    UART0_C2 = 0;

    // release pins
    PORTB_PCR16 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1); // pin 0
    PORTB_PCR17 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1); // pin 1

    // disable clock
    SIM_SCGC4 &= ~SIM_SCGC4_UART0;

    rx_buffer_head = 0;
    rx_buffer_tail = 0;
}


void uart0_write(const char* ptr, int32_t len)
{
    for (int32_t i = 0; i < len; i++)
        uart_putchar((uint8_t)ptr[i]);
}


void uart0_println(const char* ptr)
{
    for (int32_t i = 0; ptr[i]; i++)
        uart_putchar((uint8_t)ptr[i]);
    uart_putchar('\r');
    uart_putchar('\n');
}


void uart0_flush()
{
    while (is_transmitting)
        yield();
}


int32_t uart0_avail()
{
    uint16_t head = rx_buffer_head;
    uint16_t tail = rx_buffer_tail;
    if (head >= tail)
        return head - tail;
    
    return RX_BUFFER_SIZE + head - tail;
}


int32_t uart0_read(char* ptr, int32_t len)
{
    for (int32_t i = 0; i < len; i++) {
        int32_t c = uart_getchar();
        if (c < 0)
            return i;
        ptr[i] = c;
    }
    return len;
}


void uart_putchar(int32_t c)
{
    if (!(SIM_SCGC4 & SIM_SCGC4_UART0))
        return;

    uint16_t head = tx_buffer_head;
    head++;
    if (head >= TX_BUFFER_SIZE)
        head = 0;

    // while buffer is full...
    while (tx_buffer_tail == head) {
        int priority = nvic_execution_priority();
        if (priority <= IRQ_PRIORITY) {
            if (UART0_S1 & UART_S1_TDRE) {
                uint16_t tail = tx_buffer_tail;
                tail++;
                if (tail >= TX_BUFFER_SIZE)
                    tail = 0;
                uint16_t byte = tx_buffer[tail];
                UART0_D = byte;
                tx_buffer_tail = tail;
            }
        } else if (priority >= 256) {
            yield();
        }
    }

    tx_buffer[head] = c;
    is_transmitting = 1;
    tx_buffer_head = head;
    UART0_C2 = C2_TX_ACTIVE;
}


int32_t uart_getchar()
{
    uint16_t head = rx_buffer_head;
    uint16_t tail = rx_buffer_tail;
    if (head == tail)
        return -1;
    
    tail++;
    if (tail >= RX_BUFFER_SIZE)
        tail = 0;
    
    int32_t c = rx_buffer[tail];
    rx_buffer_tail = tail;
    return c;
}


void uart0_status_isr(void)
{
    // character received
    if (UART0_S1 & UART_S1_RDRF) {
        uint16_t byte = UART0_D;
        uint16_t head = rx_buffer_head + 1;
        if (head >= RX_BUFFER_SIZE)
            head = 0;
        if (head != rx_buffer_tail) {
            rx_buffer[head] = byte;
            rx_buffer_head = head;
        }
        // else drop the character
    }

    // transmit buffer ready to accept a character
    uint8_t c = UART0_C2;
    if ((c && UART_C2_TIE) && (UART0_S1 & UART_S1_TDRE)) {
        uint16_t head = tx_buffer_head;
        uint16_t tail = tx_buffer_tail;
        if (head == tail) {
            // no more characters ready to transmit
            UART0_C2 = C2_TX_COMPLETING;
        } else {
            tail++;
            if (tail >= TX_BUFFER_SIZE)
                tail = 0;
            UART0_D = tx_buffer[tail];
            tx_buffer_tail = tail;
        }
    }

    // transmission complete
    if ((c & UART_C2_TCIE) && (UART0_S1 & UART_S1_TC)) {
        is_transmitting = 0;
        UART0_C2 = C2_TX_IIDLE;
    }
}


void yield_func()
{
    if (uart0_avail() <= 0)
        return;
    if (rx_evt_func == NULL)
        return;
    rx_evt_func();
}