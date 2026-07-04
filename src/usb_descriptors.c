/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include <tusb.h>
#include <bsp/board_api.h>

// set some example Vendor and Product ID
// the board will use to identify at the host
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define CDC_EXAMPLE_VID     0xCafe
// use _PID_MAP to generate unique PID for each interface
#define CDC_EXAMPLE_PID     (0x4000 | _PID_MAP(CDC, 0))
#define CDC_EXAMPLE_BCD     0x0200

tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = CDC_EXAMPLE_BCD,

    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = CDC_EXAMPLE_VID,
    .idProduct = CDC_EXAMPLE_PID,
    .bcdDevice = 0x0100,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,

    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void);

enum {
    ITF_NUM_CDC_0 = 0,
    ITF_NUM_CDC_0_DATA,

    ITF_NUM_CDC_1,
    ITF_NUM_CDC_1_DATA,

    ITF_NUM_CDC_2,
    ITF_NUM_CDC_2_DATA,

    ITF_NUM_CDC_3,
    ITF_NUM_CDC_3_DATA,

    ITF_NUM_CDC_4,
    ITF_NUM_CDC_4_DATA,

    ITF_NUM_CDC_5,
    ITF_NUM_CDC_5_DATA,

    ITF_NUM_CDC_6,
    ITF_NUM_CDC_6_DATA,

    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + CFG_TUD_CDC * TUD_CDC_DESC_LEN)

// Endpoint numbers.
// Cada CDC usa:
// - un endpoint IN de notificación,
// - un endpoint OUT de datos,
// - un endpoint IN de datos.
#define EPNUM_CDC_0_NOTIF   0x81
#define EPNUM_CDC_0_OUT     0x01
#define EPNUM_CDC_0_IN      0x88

#define EPNUM_CDC_1_NOTIF   0x82
#define EPNUM_CDC_1_OUT     0x02
#define EPNUM_CDC_1_IN      0x89

#define EPNUM_CDC_2_NOTIF   0x83
#define EPNUM_CDC_2_OUT     0x03
#define EPNUM_CDC_2_IN      0x8A

#define EPNUM_CDC_3_NOTIF   0x84
#define EPNUM_CDC_3_OUT     0x04
#define EPNUM_CDC_3_IN      0x8B

#define EPNUM_CDC_4_NOTIF   0x85
#define EPNUM_CDC_4_OUT     0x05
#define EPNUM_CDC_4_IN      0x8C

#define EPNUM_CDC_5_NOTIF   0x86
#define EPNUM_CDC_5_OUT     0x06
#define EPNUM_CDC_5_IN      0x8D

#define EPNUM_CDC_6_NOTIF   0x87
#define EPNUM_CDC_6_OUT     0x07
#define EPNUM_CDC_6_IN      0x8E

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC_0,
    STRID_CDC_1,
    STRID_CDC_2,
    STRID_CDC_3,
    STRID_CDC_4,
    STRID_CDC_5,
    STRID_CDC_6,
};

// configure descriptor for 7 CDC interfaces
uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),

    // CDC 0: debug
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, STRID_CDC_0, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),

    // CDC 1: configuración full duplex
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, STRID_CDC_1, EPNUM_CDC_1_NOTIF, 8, EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 64),

    // CDC 2: datos ADS canales 1 y 2
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_2, STRID_CDC_2, EPNUM_CDC_2_NOTIF, 8, EPNUM_CDC_2_OUT, EPNUM_CDC_2_IN, 64),

    // CDC 3: datos ADS canales 3 y 4
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_3, STRID_CDC_3, EPNUM_CDC_3_NOTIF, 8, EPNUM_CDC_3_OUT, EPNUM_CDC_3_IN, 64),

    // CDC 4: datos ADS canales 5 y 6
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_4, STRID_CDC_4, EPNUM_CDC_4_NOTIF, 8, EPNUM_CDC_4_OUT, EPNUM_CDC_4_IN, 64),

    // CDC 5: datos ADS canales 7 y 8
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_5, STRID_CDC_5, EPNUM_CDC_5_NOTIF, 8, EPNUM_CDC_5_OUT, EPNUM_CDC_5_IN, 64),

    // CDC 6: RLD
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_6, STRID_CDC_6, EPNUM_CDC_6_NOTIF, 8, EPNUM_CDC_6_OUT, EPNUM_CDC_6_IN, 64),
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index);

tusb_desc_device_qualifier_t const desc_device_qualifier = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = CDC_EXAMPLE_BCD,

    .bDeviceClass = TUSB_CLASS_CDC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 0x01,
    .bReserved = 0x00
};

uint8_t const* tud_descriptor_device_qualifier_cb(void);

char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },
    "Raspberry Pi",
    "EMG ADS1298",
    NULL,
    "CDC0 Debug",
    "CDC1 Config",
    "CDC2 ADS CH1-CH2",
    "CDC3 ADS CH3-CH4",
    "CDC4 ADS CH5-CH6",
    "CDC5 ADS CH7-CH8",
    "CDC6 RLD"
};

static uint16_t _desc_str[32 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid);

// --------------------------------------------------------------------+
// IMPLEMENTATION
// --------------------------------------------------------------------+

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

uint8_t const* tud_descriptor_device_qualifier_cb(void)
{
    return (uint8_t const *)&desc_device_qualifier;
}

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void) langid;
    size_t char_count;

    if (index == STRID_LANGID) {
        memcpy(&_desc_str[1], string_desc_arr[STRID_LANGID], 2);
        char_count = 1;
    } else if (index == STRID_SERIAL) {
        char_count = board_usb_get_serial(_desc_str + 1, 32);
    } else {
        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
            return NULL;
        }

        const char *str = string_desc_arr[index];
        char_count = strlen(str);

        size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1;
        if (char_count > max_count) {
            char_count = max_count;
        }

        for (size_t i = 0; i < char_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (char_count * 2 + 2));

    return _desc_str;
}
