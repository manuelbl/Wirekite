/*
 * Wirekite - MCU code 
 * Copyright (c) 2017 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

#include <stddef.h>
#include "kinetis.h"

#include "usb.h"
#include "mem.h"
#include "debug.h"


extern uint8_t wk_reset_flag;

#define N_ENDPOINTS 3

#define ENDP0_PACKET_SIZE 64
#define ENDP1_PACKET_SIZE 64
#define ENDP2_PACKET_SIZE 64

#define PID_OUT   0x1
#define PID_ACK   0x2
#define PID_IN    0x9
#define PID_SOF   0x5
#define PID_SETUP 0xd

#define DEV_STATE_DEFAULT    0x00
#define DEV_STATE_ADDRESS    0x01
#define DEV_STATE_CONFIGURED 0x02

//determines an appropriate BDT index for the given conditions (see fig. 41-3)
#define RX 0
#define TX 1
#define EVEN 0
#define ODD  1
#define BDT(endpoint, tx, odd) (bdt_table[(endpoint << 2) | (tx << 1) | odd])
#define STAT_TO_BDT(stat) (bdt_table[stat >> 2])

#define DATA0 0
#define DATA1 1

/**
 * Buffer Descriptor Table entry
 * There are two entries per direction per endpoint:
 *   In  Even/Odd
 *   Out Even/Odd
 */
typedef struct {
    volatile uint32_t desc;
    volatile void* addr;
} bdt_t;

/**
 * Buffer descriptor table, aligned to a 512-byte boundary (see linker file)
 */
__attribute__ ((section(".usbdescriptortable"), used))
static bdt_t bdt_table[N_ENDPOINTS * 4]; // max endpoints is 15 + 1 control


#define BDT_BC_SHIFT   16
#define BDT_OWN   0x80
#define BDT_DATA1 0x40
#define BDT_KEEP  0x20
#define BDT_NINC  0x10
#define BDT_DTS   0x08
#define BDT_STALL 0x04

#define BD_OWNED_BY_USB(count, data) ((count << BDT_BC_SHIFT) | BDT_OWN | BDT_DTS | (data ? BDT_DATA1 : 0x00))
#define BDT_PID(desc) ((desc >> 2) & 0xF)


typedef struct {
    union {
        struct {
            uint8_t bmRequestType;
            uint8_t bRequest;
        };
        uint16_t wRequestAndType;
    };
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} setup_t;

/**
 * Descriptors
 */

typedef struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wString[];
} string_desc_t;

typedef struct {
    uint16_t wValue;
    uint16_t wIndex;
    const void* addr;
    uint8_t length;
} descriptor_entry_t;

/**
 * Device descriptor
 */
static uint8_t device_desc[] = {
    18,                 // bLength = 18 bytes
    1,                  // bDescriptorType = device
    0x00, 0x02,         // bcdUSB = 2.00
    0xff,               // bDeviceClass = vendor-specific
    0x00,               // bDeviceSubClass = 0
    0x00,               // bDeviceProtocol = no class-specific protocol
    ENDP0_PACKET_SIZE,  // bMaxPacketSize0 = 64 bytes
    0xc0, 0x16,         // idVendor = 16c0 // TODO
    0x01, 0x27,         // idProduct = 2701 // TODO
    0x10, 0x00,         // bcdDevice = version 1
    1,                  // iManufacturer = string descriptor 1
    2,                  // iProduct = string descriptor 2
    3,                  // iSerialNumber = string descriptor 3
    1                   // bNumConfigurations
};

/**
 * Configuration descriptor
 */
static uint8_t configuration_desc[] = {
    9,                  // bLength = 9 bytes
    2,                  // bDescriptorType = configuration
    9 + 9 + 7 + 7, 0x00,// wTotalLength
    1,                  // bNumInterfaces = 1
    1,                  // bConfigurationValue = 1 (value for this configuration)
    0,                  // iConfiguration = null
    0x80,               // bmAttributes = default (bus-powered, no remote wakeup)
    250,                // bMaxPower = 500 mA

    /* INTERFACE 0 BEGIN */
    9,                  // bLength = 9 bytes
    4,                  // bDescriptorType = interface
    0,                  // bInterfaceNumber
    0,                  // bAlternateSetting
    2,                  // bNumEndpoints
    0xff,               // bInterfaceClass = vendor-specific
    0x00,               // bInterfaceSubClass,
    0x00,               // bInterfaceProtocol = vendor-specific protocol
    0,                  // iInterface = null

        /* INTERFACE 0, ENDPOINT 1 BEGIN */
        7,              // bLength = 7 bytes
        5,              // bDescriptorType = interface
        0x81,           // bEndpointAddress = endpoint 1 / IN (TX)
        0x02,           // bmAttributes = bulk endpoint
        ENDP1_PACKET_SIZE, 0x00,
                        // wMaxPacketSize
        0,              // bInterval = ?
        /* INTERFACE 0, ENDPOINT 1 END */

        /* INTERFACE 0, ENDPOINT 2 BEGIN */
        7,              // bLength = 7 bytes
        5,              // bDescriptorType = interface
        0x02,           // bEndpointAddress = endpoint 2 / OUT (RX)
        0x02,           // bmAttributes = bulk endpoint
        ENDP2_PACKET_SIZE, 0x00,
                        // wMaxPacketSize
        0               // bInterval = ?
        /* INTERFACE 0, ENDPOINT 2 END */

    /* INTERFACE 0 END */
};

static string_desc_t language_desc = {
    .bLength = 2 + 1 * 2,
    .bDescriptorType = 3, // string
    .wString = { 0x0409 } // English (US)
};

static string_desc_t manufacturer_desc = {
    .bLength = 2 + 21 * 2,
    .bDescriptorType = 3, // string
    .wString = { 'M', 'a', 'n', 'u', 'e', 'l', ' ', 'B', 'l', 'e', 'i', 'c', 'h', 'e', 'n', 'b', 'a', 'c', 'h', 'e', 'r' }
};

static string_desc_t product_desc = {
    .bLength = 2 + 8 * 2,
    .bDescriptorType = 3, // string
    .wString = { 'W', 'i', 'r', 'e', 'k', 'i', 't', 'e' }
};

#define WCID_VENDOR_CODE 0x22

// Microsoft WCID information, also see https://github.com/pbatard/libwdi/wiki/WCID-Devices
static string_desc_t msft_sig_desc = {
    .bLength = 2 + 8 * 2,
    .bDescriptorType = 3, // string
    .wString = { 'M', 'S', 'F', 'T', '1', '0', '0', WCID_VENDOR_CODE }
};

// Microsoft WCID feature descriptor
static uint8_t wcid_feature_desc[] = {
    0x28, 0x00, 0x00, 0x00, // length = 40 bytes
    0x00, 0x01, // version 1.0 (in BCD)
    0x04, 0x00, // compatibility descriptor index 0x0004
    0x01, // number of sections
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, // reserved (7 bytes)
    0x00, // interface number 0
    0x01, // reserved
    0x57, 0x49, 0x4E, 0x55,
    0x53, 0x42, 0x00, 0x00, // Compatible ID "WINUSB\0\0"
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, // Subcompatible ID (unused)
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00 // reserved 6 bytes
};

static uint8_t wcid_extended_properties_feature_desc[] = {
    0x8E, 0x00, 0x00, 0x00, // length = 142 bytes
    0x00, 0x01, // version 1.0 (in BCD)
    0x05, 0x00, // extended property descriptor index 0x0005
    0x01, 0x00, // number of sections
    0x84, 0x00, 0x00, 0x00, // length of properties section = 132 bytes
    0x01, 0x00, 0x00, 0x00, // property data type = REG_SZ
    0x28, 0x00, // property name length = 40 bytes
    0x44, 0x00, 0x65, 0x00,
    0x76, 0x00, 0x69, 0x00,
    0x63, 0x00, 0x65, 0x00,
    0x49, 0x00, 0x6e, 0x00,
    0x74, 0x00, 0x65, 0x00,
    0x72, 0x00, 0x66, 0x00,
    0x61, 0x00, 0x63, 0x00,
    0x65, 0x00, 0x47, 0x00, 
    0x55, 0x00, 0x49, 0x00,
    0x44, 0x00, 0x00, 0x00, // "DeviceInterfaceGUID", UTF-16, NULL-terminated
    0x4e, 0x00, 0x00, 0x00, // property data length = 78 bytes
    0x7b, 0x00, 0x64, 0x00,
    0x30, 0x00, 0x31, 0x00,
    0x30, 0x00, 0x62, 0x00,
    0x61, 0x00, 0x39, 0x00,
    0x30, 0x00, 0x2d, 0x00,
    0x30, 0x00, 0x32, 0x00,
    0x35, 0x00, 0x61, 0x00,
    0x2d, 0x00, 0x34, 0x00,
    0x63, 0x00, 0x39, 0x00,
    0x34, 0x00, 0x2d, 0x00,
    0x62, 0x00, 0x32, 0x00,
    0x64, 0x00, 0x62, 0x00,
    0x2d, 0x00, 0x61, 0x00,
    0x31, 0x00, 0x33, 0x00,
    0x66, 0x00, 0x32, 0x00,
    0x30, 0x00, 0x35, 0x00,
    0x64, 0x00, 0x34, 0x00,
    0x33, 0x00, 0x37, 0x00,
    0x65, 0x00, 0x7d, 0x00,
    0x00, 0x00 // "{d010ba90-025a-4c94-b2db-a13f205d437e}",  UTF-16, NULL-terminated
};

static const descriptor_entry_t descriptors[] = {
    { 0x0100, 0x0000, device_desc, sizeof(device_desc) },
    { 0x0200, 0x0000, &configuration_desc, sizeof(configuration_desc) },
    { 0x0300, 0x0000, &language_desc, 4 },
    { 0x0301, 0x0409, &manufacturer_desc, 2 + 21 * 2 },
    { 0x0302, 0x0409, &product_desc, 2 + 8 * 2 },
    { 0x03ee, 0x0000, &msft_sig_desc, 2 + 8 * 2 },
    { 0x0000, 0x0000, NULL, 0 }
};

static uint8_t serial_num_buf[28];

static void usb_buffer_init(uint8_t configuration);


/**
 * Device data
 */

static uint8_t dev_configuration = 0;
static uint8_t dev_state = DEV_STATE_DEFAULT;
static volatile uint8_t reply_buffer[8] __attribute__ ((aligned (4)));


/**
 * Endpoint 0
 */


static volatile uint8_t endp0_rx[2][ENDP0_PACKET_SIZE] __attribute__ ((aligned (4)));
static uint8_t endp0_odd = 0;
static uint8_t endp0_data = 0;
static volatile const uint8_t* endp0_tx_dataptr = NULL; // pointer to current transmit chunk
static uint16_t endp0_tx_datalen = 0; // length of data remaining to send

static void endp0_transmit(volatile const void* data, uint16_t length);
static void endp0_handle_setup(setup_t* packet);


/**
 * Endpoint 0 handler
 */
void endp0_handler(uint8_t stat)
{
    static setup_t last_setup;

    volatile const uint8_t* data = NULL;
    uint32_t size = 0;

    // determine which bdt we are looking at here
    bdt_t* bdt = &STAT_TO_BDT(stat);

    switch (BDT_PID(bdt->desc))
    {
    case PID_SETUP:
        // extract the setup token
		last_setup = *((setup_t*)(bdt->addr));

		// return buffer to USB FS
        bdt->desc = BD_OWNED_BY_USB(ENDP0_PACKET_SIZE, 1);

        // clear any pending IN stuff
        BDT(0, TX, EVEN).desc = 0;
		BDT(0, TX, ODD).desc = 0;
        endp0_data = 1;

        // cast the data into our setup type and run the setup
        endp0_handle_setup(&last_setup);

        // unfreeze this endpoint
        USB0_CTL = USB_CTL_USBENSOFEN;
        break;

    case PID_IN: // TX completed
        // continue sending any pending transmit data
        data = endp0_tx_dataptr;
		if (data) {
			size = endp0_tx_datalen;
			if (size > ENDP0_PACKET_SIZE)
                size = ENDP0_PACKET_SIZE;
			endp0_transmit(data, size);
			data += size;
			endp0_tx_datalen -= size;
			endp0_tx_dataptr = (endp0_tx_datalen > 0 || size == ENDP0_PACKET_SIZE) ? data : NULL;
		}

        // delayed setting of address can happen here
        if (last_setup.wRequestAndType == 0x0500) {
            USB0_ADDR = last_setup.wValue;
            if (last_setup.wValue == 0)
                dev_state = DEV_STATE_DEFAULT;
        }

        break;
    
    case PID_OUT: // RX received
        // give buffer back to USB
        bdt->desc = BD_OWNED_BY_USB(ENDP0_PACKET_SIZE, 1);

        break;

    default:
        // nothing to do here
        break;
    }
}


/**
 * Endpoint 0 transmission
 */
static void endp0_transmit(volatile const void* data, uint16_t length)
{
    BDT(0, TX, endp0_odd).addr = (void *)data;
    BDT(0, TX, endp0_odd).desc = BD_OWNED_BY_USB(length, endp0_data);
    // toggle the odd and data bits
    endp0_odd ^= 1;
    endp0_data ^= 1;
}


/**
 * Endpoint 0 setup
 */
static void endp0_handle_setup(setup_t* packet)
{
    volatile const uint8_t* data = NULL;
    uint16_t data_length = 0;
    uint32_t size = 0;
    uint8_t index;

#ifdef _DEBUG
    char debug[] = "setup xxxx";
    bytes_to_hex(debug + 6, (uint8_t*) &packet->wRequestAndType, 2);
    DEBUG_OUT(debug);
#endif

#ifdef _DEBUG
    char debug2[] = "set cfg xx";
    char debug4[] = "get dsc xxxx xxxx";
#endif

    switch(packet->wRequestAndType) {

        case 0x0880: // GET_CONFIGURATION
            reply_buffer[0] = dev_configuration;
            data  = reply_buffer;
            data_length = 1;
            break;

        case 0x0680: // GET_DESCRIPTOR (device)
        case 0x0681: // GET_DESCRIPTOR (endpoint)
#ifdef _DEBUG
            bytes_to_hex(debug4 + 8, (uint8_t*) &packet->wValue, 2);
            bytes_to_hex(debug4 + 13, (uint8_t*) &packet->wIndex, 2);
            DEBUG_OUT(debug4);
#endif

            // check for serial number
            if (packet->wValue == 0x0303 && packet->wIndex == 0x0409) {
                string_desc_t* serial_num_desc = (string_desc_t*)serial_num_buf;
                data = serial_num_buf;
                data_length = serial_num_desc->bLength;
                goto send;
            }
            for (const descriptor_entry_t* entry = descriptors; 1; entry++) {
                if (entry->addr == NULL)
                    break;

                if (packet->wValue == entry->wValue && packet->wIndex == entry->wIndex) {
                    data = entry->addr;
                    data_length = entry->length;
                    goto send;
                }
            }
            goto stall;

        case 0x0080: // GET_STATUS (device)
        case 0x0081: // GET_STATUS (interface)
            reply_buffer[0] = 0;
            reply_buffer[1] = 0;
            data = reply_buffer;
            data_length = 2;
            break;

        case 0x0082: // GET_STATUS (endpoint)
            index = packet->wIndex & 0x7F;
            if ((dev_state != DEV_STATE_CONFIGURED && index > 0) || index > N_ENDPOINTS)
                goto stall;

            reply_buffer[0] = 0;
            reply_buffer[1] = 0;
            if (*(uint8_t *)(&USB0_ENDPT0 + index * 4) & USB_ENDPT_EPSTALL)
                reply_buffer[0] = 1;
            data = reply_buffer;
            data_length = 2;
            break;

        case 0x0500: // SET_ADDRESS
            // wait for IN packet
            break;

        case 0x0900: // SET_CONFIGURATION
            if (packet->wValue != 0 && packet->wValue != 1)
                goto stall;
            dev_configuration = packet->wValue;
#ifdef _DEBUG
            bytes_to_hex(debug2 + 8, &dev_configuration, 1);
            DEBUG_OUT(debug2);
#endif
            dev_state = dev_configuration != 0 ? DEV_STATE_CONFIGURED : DEV_STATE_ADDRESS;
            usb_buffer_init(dev_configuration);
            endp0_data = 1;
            endp0_tx_dataptr = NULL;
            endp0_tx_datalen = 0;
            break;

        case 0x00c0 | (WCID_VENDOR_CODE << 8):
        case 0x00c1 | (WCID_VENDOR_CODE << 8):
            if (packet->wIndex == 0x0004) {
                data = wcid_feature_desc;
                data_length = wcid_feature_desc[0];
                goto send;
            } else if (packet->wIndex == 0x0005) {
                data = wcid_extended_properties_feature_desc;
                data_length = wcid_extended_properties_feature_desc[0];
                goto send;
            }
            break;

        default:
            goto stall;
    }

send:
    //if we get here, we need to send some data

    // truncate the data length to whatever the setup packet is expecting
    if (data_length > packet->wLength)
        data_length = packet->wLength;

    // transmit 1st chunk
    size = data_length;
    if (size > ENDP0_PACKET_SIZE)
        size = ENDP0_PACKET_SIZE;
    endp0_transmit(data, size);
    data += size; // move the pointer down
    data_length -= size; // move the size down
    if (data_length == 0 && size < ENDP0_PACKET_SIZE)
        return; // all done!

    // transmit 2nd chunk
    size = data_length;
    if (size > ENDP0_PACKET_SIZE)
        size = ENDP0_PACKET_SIZE;
    endp0_transmit(data, size);
    data += size; // move the pointer down
    data_length -= size; // move the size down
    if (data_length == 0 && size < ENDP0_PACKET_SIZE)
        return; // all done!

    // if any data remains to be transmitted, we need to store it
    endp0_tx_dataptr = data;
    endp0_tx_datalen = data_length;
    return;

    // if we make it here, we are not able to send data and have stalled
stall:
    DEBUG_OUT("STALL");
    USB0_ENDPT0 = USB_ENDPT_EPSTALL | USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;
}


/**
 * Endpoint 1 handler
 */

#define TX_NUM_BUFFERS 20

static wk_msg_header* circ_tx_buf[TX_NUM_BUFFERS];
static uint8_t circ_tx_head = 0;
static uint8_t circ_tx_tail = 0;
static uint8_t endp1_odd = 0;
static uint8_t endp1_data = 0;


wk_msg_header* endp1_get_next_buffer();
void endp1_append_buffer(wk_msg_header* buf);


void endp1_handler(uint8_t stat)
{
    // determine which bdt we are looking at here
    bdt_t* bdt = &STAT_TO_BDT(stat);

    switch (BDT_PID(bdt->desc)) {
    case PID_SETUP:
        /*
		// we are now done with the buffer
        bdt->desc = BD_OWNED_BY_USB(ENDP1_PACKET_SIZE, 1);

        // clear any pending IN stuff
        BDT(1, TX, EVEN).desc = 0;
		BDT(1, TX, ODD).desc = 0;
        endp1_data = 1;
        */
        break;

    case PID_IN: // TX
        mm_free((void*) bdt->addr);
        bdt->addr = 0;

        __disable_irq();
        while ((BDT(1, TX, endp1_odd).desc & BDT_OWN) == 0) {
            wk_msg_header* msg = (wk_msg_header*) endp1_get_next_buffer();
            if (msg == NULL)
                break;
            BDT(1, TX, endp1_odd).addr = msg;
            BDT(1, TX, endp1_odd).desc = BD_OWNED_BY_USB(msg->message_size, endp1_data);
            endp1_data ^= 1;
            endp1_odd ^= 1;
        }
        __enable_irq();
        break;

    case PID_OUT: // RX
        // this endpoint only transmits
        break;

    case PID_SOF:
        break;
    }
}


void endp1_tx_msg(wk_msg_header* msg) {

    if (dev_state != DEV_STATE_CONFIGURED)
        return; // not in configured state

    __disable_irq();

    if (BDT(1, TX, endp1_odd).desc & BDT_OWN) {
        // queue message
        endp1_append_buffer(msg);
    } else {
        BDT(1, TX, endp1_odd).addr = msg;
        BDT(1, TX, endp1_odd).desc = BD_OWNED_BY_USB(msg->message_size, endp1_data);
        endp1_data ^= 1;
        endp1_odd ^= 1;
    }

    __enable_irq();
}


wk_msg_header* endp1_get_next_buffer() {

    if (circ_tx_head == circ_tx_tail)
        return NULL;
    
    circ_tx_tail++;
    if (circ_tx_tail >= TX_NUM_BUFFERS)
        circ_tx_tail = 0;
    
    return circ_tx_buf[circ_tx_tail];
}


void endp1_append_buffer(wk_msg_header* buf) {
    uint8_t head = circ_tx_head + 1;
    if (head >= TX_NUM_BUFFERS)
        head = 0;
    if (head != circ_tx_tail) {
        circ_tx_buf[head] = buf;
        circ_tx_head = head;
    } else {
        // drop the buffer
        mm_free(buf);
    }
}


/**
 * Endpoint 2 handler
 */

static volatile uint8_t endp2_buffer_even[ENDP2_PACKET_SIZE] __attribute__ ((aligned (4)));
static volatile uint8_t endp2_buffer_odd[ENDP2_PACKET_SIZE] __attribute__ ((aligned (4)));
static uint8_t endp2_odd = 0;
static uint8_t endp2_data = 0;


void endp2_handler(uint8_t stat)
{
    // determine which bdt we are looking at here
    bdt_t* bdt = &STAT_TO_BDT(stat);

    switch (BDT_PID(bdt->desc)) {
    case PID_SETUP:
        /*
		// we are now done with the buffer
        bdt->desc = BD_OWNED_BY_USB(ENDP2_PACKET_SIZE, 1);

        // clear any pending IN stuff
        BDT(2, RX, EVEN).desc = 0;
		BDT(2, RX, ODD).desc = 0;
        endp2_data = 1;

        // unfreeze this endpoint
        USB2_CTL = USB_CTL_USBENSOFEN;
        */
        break;

    case PID_IN: // TX
        // this endpoint only receives data
        break;

    case PID_OUT: // RX
        // nothing to do here
        break;

    case PID_SOF:
        break;
    }
}


volatile uint8_t* endp2_get_rx_buffer()
{
    if (dev_state != DEV_STATE_CONFIGURED)
        return NULL; // not in configured state

    if (BDT(2, RX, endp2_odd).desc & BDT_OWN)
        return NULL; // no received data currently available

    return BDT(2, RX, endp2_odd).addr;
}


int16_t endp2_get_rx_size()
{
    if (dev_state != DEV_STATE_CONFIGURED)
        return -1; // not in configured state

    if (BDT(2, RX, endp2_odd).desc & BDT_OWN)
        return -1; // no received data currently available

    return (BDT(2, RX, endp2_odd).desc >> BDT_BC_SHIFT) & 0x03ff;

}


void endp2_consume_rx_buffer()
{
    BDT(2, RX, endp2_odd).desc = BD_OWNED_BY_USB(ENDP2_PACKET_SIZE, endp2_data);
    endp2_data ^= 1;
    endp2_odd ^= 1;
}


/**
 * Device setup and interrupt
 */

void usb_init(const char* serial_number)
{
    usb_buffer_init(0);

    string_desc_t* desc = (string_desc_t*)&serial_num_buf;
    desc->bDescriptorType = 3;
    uint8_t len = 0;
    while (serial_number[len]) {
        desc->wString[len] = serial_number[len];
        len++;
    }
    desc->bLength = 2 + len * 2;

	// assume 48 MHz clock already running
	// SIM - enable clock
	SIM_SCGC4 |= SIM_SCGC4_USBOTG;
#ifdef HAS_KINETIS_MPU
	MPU_RGDAAC0 |= 0x03000000;
#endif
#if F_CPU == 180000000 || F_CPU == 216000000
	// if using IRC48M, turn on the USB clock recovery hardware
	USB0_CLK_RECOVER_IRC_EN = USB_CLK_RECOVER_IRC_EN_IRC_EN | USB_CLK_RECOVER_IRC_EN_REG_EN;
	USB0_CLK_RECOVER_CTRL = USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN |
		USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN;
#endif

	// reset USB module
	USB0_USBTRC0 = USB_USBTRC_USBRESET;
	while ((USB0_USBTRC0 & USB_USBTRC_USBRESET) != 0) ; // wait for reset to end

    // set BDT base registers
    USB0_BDTPAGE1 = ((uint32_t)bdt_table) >> 8;  //bits 15-9
    USB0_BDTPAGE2 = ((uint32_t)bdt_table) >> 16; //bits 23-16
    USB0_BDTPAGE3 = ((uint32_t)bdt_table) >> 24; //bits 31-24

    // clear all ISR flags and enable weak pull downs
    USB0_ISTAT = 0xFF;
    USB0_ERRSTAT = 0xFF;
    USB0_OTGISTAT = 0xFF;
    USB0_USBTRC0 |= 0x40; //a hint was given that this is an undocumented interrupt bit

    // enable USB reset interrupt
    USB0_CTL = USB_CTL_USBENSOFEN;
    USB0_USBCTRL = 0;

    USB0_INTEN = USB_INTEN_USBRSTEN;

	// enable interrupt in NVIC...
	NVIC_SET_PRIORITY(IRQ_USBOTG, 128);
	NVIC_ENABLE_IRQ(IRQ_USBOTG);

	// enable d+ pullup
	USB0_CONTROL = USB_CONTROL_DPPULLUPNONOTG;
}


void usb_buffer_init(uint8_t configuration)
{
    endp0_odd = EVEN;
    endp0_data = DATA0;
    BDT(0, RX, EVEN).addr = endp0_rx[EVEN];
    BDT(0, RX, EVEN).desc = BD_OWNED_BY_USB(ENDP0_PACKET_SIZE, DATA0);
    BDT(0, RX, ODD).addr = endp0_rx[ODD];
    BDT(0, RX, ODD).desc = BD_OWNED_BY_USB(ENDP0_PACKET_SIZE, DATA1);
    BDT(0, TX, EVEN).addr = 0;
    BDT(0, TX, EVEN).desc = 0;
    BDT(0, TX, ODD).addr = 0;
    BDT(0, TX, ODD).desc = 0;

    //initialize endpoint0 to 0x0d (41.5.23)
    //transmit, recieve, and handshake
    USB0_ENDPT0 = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;

    endp1_odd = EVEN;
    endp1_data = DATA0;
    BDT(1, RX, EVEN).addr = 0;
    BDT(1, RX, EVEN).desc = 0;
    BDT(1, RX, ODD).addr = 0;
    BDT(1, RX, ODD).desc = 0;
    BDT(1, TX, EVEN).addr = 0;
    BDT(1, TX, EVEN).desc = 0;
    BDT(1, TX, ODD).addr = 0;
    BDT(1, TX, ODD).desc = 0;

    while (1) {
        void* buf = endp1_get_next_buffer();
        if (buf == NULL)
            break;
        mm_free(buf);
    }

    USB0_ENDPT1 = configuration == 1 ? USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK : 0;

    endp2_odd = EVEN;
    endp2_data = DATA0;
    BDT(2, RX, EVEN).addr = endp2_buffer_even;
    BDT(2, RX, EVEN).desc = BD_OWNED_BY_USB(ENDP2_PACKET_SIZE, DATA0);
    BDT(2, RX, ODD).addr = endp2_buffer_odd;
    BDT(2, RX, ODD).desc = BD_OWNED_BY_USB(ENDP2_PACKET_SIZE, DATA1);
    BDT(2, TX, EVEN).addr = 0;
    BDT(2, TX, EVEN).desc = 0;
    BDT(2, TX, ODD).addr = 0;
    BDT(2, TX, ODD).desc = 0;

    USB0_ENDPT2 = configuration == 1 ? USB_ENDPT_EPRXEN | USB_ENDPT_EPHSHK : 0;

    wk_reset_flag = 1;
}


void usb_isr(void)
{
    uint8_t status;

restart:
    status = USB0_ISTAT;

    if (status & USB_ISTAT_SOFTOK) {
		USB0_ISTAT = USB_ISTAT_SOFTOK;
    }

    if (status & USB_ISTAT_TOKDNE) {
        //handle completion of current token being processed
        uint8_t stat = USB0_STAT;
        uint8_t endpoint = stat >> 4;

#ifdef _DEBUG
        char debug[] = "TOKDNE xx";
        bytes_to_hex(debug + 7, &endpoint, 1);
        DEBUG_OUT(debug);
#endif

        if (endpoint == 0) {
            endp0_handler(stat);
        } else if (endpoint == 1) {
            endp1_handler(stat);
        } else if (endpoint == 2) {
            endp2_handler(stat);
        }

        USB0_ISTAT = USB_ISTAT_TOKDNE;
        goto restart;
    }

    if (status & USB_ISTAT_USBRST) {
        //handle USB reset
        DEBUG_OUT("USBRST");

        //initialize endpoint 0 ping-pong buffers
        USB0_CTL |= USB_CTL_ODDRST;

        usb_buffer_init(0);

        //clear all interrupts...this is a reset
        USB0_ERRSTAT = 0xff;
        USB0_ISTAT = 0xff;

        //after reset, we are address 0, per USB spec
        USB0_ADDR = 0;

        //all necessary interrupts are now active
        USB0_ERREN = 0xFF;
        USB0_INTEN = USB_INTEN_USBRSTEN | USB_INTEN_ERROREN |
            USB_INTEN_SOFTOKEN | USB_INTEN_TOKDNEEN |
            USB_INTEN_SLEEPEN | USB_INTEN_STALLEN;

        return;
    }

    if (status & USB_ISTAT_STALL) {
        DEBUG_OUT("STALL");
        //handle usb stall
        USB0_ENDPT0 = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;
        USB0_ISTAT = USB_ISTAT_STALL;
    }

    if (status & USB_ISTAT_ERROR) {
        DEBUG_OUT("ERROR");
        //handle error
		uint8_t err = USB0_ERRSTAT;
        USB0_ERRSTAT = err;
        USB0_ISTAT = USB_ISTAT_ERROR;
    }

    if (status & USB_ISTAT_SLEEP) {
        //handle USB sleep
        USB0_ISTAT = USB_ISTAT_SLEEP;
    }
}