#ifndef USB_MANAGER_H
#define USB_MANAGER_H

#include <stdint.h>
#include "ads1298.h"

// ==================================================
// CANALES USB CDC DEL PROYECTO
// ==================================================
// CDC 0: debug
// CDC 1: configuración full duplex
// CDC 2: datos ADS canales 1 y 2
// CDC 3: datos ADS canales 3 y 4
// CDC 4: datos ADS canales 5 y 6
// CDC 5: datos ADS canales 7 y 8
// CDC 6: RLD

enum EmgCdcInterface : uint8_t {
    CDC_DEBUG     = 0,
    CDC_CONFIG    = 1,
    CDC_DATA_CH12 = 2,
    CDC_DATA_CH34 = 3,
    CDC_DATA_CH56 = 4,
    CDC_DATA_CH78 = 5,
    CDC_RLD       = 6,
    CDC_COUNT     = 7
};

void emg_usb_init(ads1298 *ads_device);
void emg_usb_task(void);
void emg_usb_write(uint8_t itf, const char *msg);
void emg_usb_write_line(uint8_t itf, const char *msg);
void emg_usb_write_debug(const char *msg);

#endif
