#include "usb_manager.h"

#include <stdio.h>
#include <string.h>

#include <bsp/board_api.h>
#include <tusb.h>

static ads1298 *g_ads = nullptr;

static const char *interface_name(uint8_t itf)
{
    switch (itf) {
        case CDC_DEBUG:     return "DEBUG INTERFACE (CDC0)";
        case CDC_CONFIG:    return "CONFIGURATION INTERFACE (CDC1)";
        case CDC_DATA_CH12: return "DATA STREAM CH1-CH2 (CDC2)";
        case CDC_DATA_CH34: return "DATA STREAM CH3-CH4 (CDC3)";
        case CDC_DATA_CH56: return "DATA STREAM CH5-CH6 (CDC4)";
        case CDC_DATA_CH78: return "DATA STREAM CH7-CH8 (CDC5)";
        case CDC_RLD:       return "RLD INTERFACE (CDC6)";
        default:            return "UNKNOWN CDC INTERFACE";
    }
}

static void send_interface_banner(uint8_t itf)
{
    if (!tud_cdc_n_connected(itf)) {
        return;
    }

    emg_usb_write_line(itf, "=========================================");
    emg_usb_write_line(itf, "EMG MULTICHANNEL - ADS1298");
    emg_usb_write_line(itf, interface_name(itf));
    emg_usb_write_line(itf, "=========================================");

    if (itf == CDC_CONFIG) {
        emg_usb_write_line(itf, "Comandos disponibles por ahora:");
        emg_usb_write_line(itf, "  ID      -> lee el registro ID del ADS1298");
        emg_usb_write_line(itf, "  READ_ID -> igual que ID");
    }
}

static bool command_is(const char *rx, const char *cmd)
{
    size_t n = strlen(cmd);
    return strncmp(rx, cmd, n) == 0 &&
           (rx[n] == '\0' || rx[n] == '\r' || rx[n] == '\n');
}

void emg_usb_init(ads1298 *ads_device)
{
    g_ads = ads_device;

    board_init();
    tusb_init();

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }
}

void emg_usb_task(void)
{
    tud_task();

    // Futuro: acá se puede agregar el envío continuo de datos a:
    // CDC_DATA_CH12, CDC_DATA_CH34, CDC_DATA_CH56, CDC_DATA_CH78 y CDC_RLD.
}

void emg_usb_write(uint8_t itf, const char *msg)
{
    if (itf >= CDC_COUNT || msg == nullptr) {
        return;
    }

    if (tud_cdc_n_connected(itf)) {
        tud_cdc_n_write(itf, msg, strlen(msg));
        tud_cdc_n_write_flush(itf);
    }
}

void emg_usb_write_line(uint8_t itf, const char *msg)
{
    if (itf >= CDC_COUNT || msg == nullptr) {
        return;
    }

    if (tud_cdc_n_connected(itf)) {
        tud_cdc_n_write(itf, msg, strlen(msg));
        tud_cdc_n_write(itf, "\r\n", 2);
        tud_cdc_n_write_flush(itf);
    }
}

void emg_usb_write_debug(const char *msg)
{
    emg_usb_write_line(CDC_DEBUG, msg);
}

// TinyUSB llama automáticamente a este callback cuando se abre/cierra un puerto CDC.
// Cuando DTR pasa a true, el monitor serial del PC acaba de abrir ese puerto.
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)rts;

    if (dtr) {
        send_interface_banner(itf);
    }
}

// TinyUSB llama automáticamente a este callback cuando llega información por algún CDC.
void tud_cdc_rx_cb(uint8_t itf)
{
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE + 1];

    uint32_t count = tud_cdc_n_read(itf, buf, CFG_TUD_CDC_RX_BUFSIZE);
    buf[count] = 0;

    if (itf == CDC_CONFIG) {
        if (command_is((char *)buf, "ID") || command_is((char *)buf, "READ_ID")) {
            if (g_ads == nullptr) {
                emg_usb_write_line(CDC_CONFIG, "ERROR: ADS1298 no inicializado");
                return;
            }

            uint8_t id = g_ads->readId();

            char msg[32];
            snprintf(msg, sizeof(msg), "ID=0x%02X", id);
            emg_usb_write_line(CDC_CONFIG, msg);

            char dbg[64];
            snprintf(dbg, sizeof(dbg), "[CFG] ID solicitado por CDC1: 0x%02X", id);
            emg_usb_write_line(CDC_DEBUG, dbg);
        } else {
            emg_usb_write_line(CDC_CONFIG, "OK");
        }
    } else if (itf == CDC_RLD) {
        emg_usb_write_line(CDC_RLD, "RLD_OK");
    } else {
        // Vacía RX de cualquier otro CDC para no bloquearlo.
        emg_usb_write_line(itf, "OK");
    }
}
