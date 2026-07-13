#include "acquisition.h"

#include "pico/stdlib.h"

#include "usb_manager.h"

static ads1298 *g_ads = nullptr;

void acquisition_init(ads1298 *ads_device)
{
    g_ads = ads_device;
}

void acquisition_core1_entry()
{
    while (true) {
        if (g_ads == nullptr || !g_ads->drdyIsLow()) {
            tight_loop_contents();
            continue;
        }

        uint8_t status[3];
        int32_t channels[8];

        g_ads->lock();
        bool ok = g_ads->readDataFrame(status, channels);
        g_ads->unlock();

        if (!ok) {
            continue;
        }

        // Cada CDC de datos (CDC2-5) transporta 2 canales físicos, en el
        // mismo orden fijo definido en el mapeo de canales (ver CLAUDE.md):
        // par 0 = canales 1-2, par 1 = canales 3-4, par 2 = canales 5-6,
        // par 3 = canales 7-8.
        emg_usb_push_channel_pair(0, channels[0], channels[1]);
        emg_usb_push_channel_pair(1, channels[2], channels[3]);
        emg_usb_push_channel_pair(2, channels[4], channels[5]);
        emg_usb_push_channel_pair(3, channels[6], channels[7]);

        uint8_t loff_statp = ads1298::extractLoffStatP(status);
        uint8_t loff_statn = ads1298::extractLoffStatN(status);
        emg_usb_push_rld_status(loff_statp, loff_statn);
    }
}
