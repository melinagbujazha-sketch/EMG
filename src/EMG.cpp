#include <stdio.h>

#include "pico/stdlib.h"
#include "ads1298.h"
#include "usb_manager.h"

// ==================================================
// ADS1298
// ==================================================

// Instancia global del ADS1298.
// Pines por defecto definidos en ads1298.h:
// DOUT GP16, CS GP17, SCLK GP18, DIN GP19,
// DRDY GP20, RESET GP21, START GP22, PWDN GP26.
ads1298 ads;

// ==================================================
// Programa principal
// ==================================================

int main(void)
{
    emg_usb_init(&ads);

    // Espera corta para que la PC llegue a enumerar los CDC USB.
    sleep_ms(3000);

    ads.init();
    ads.reset();

    uint8_t id = ads.readId();

    emg_usb_write_line(CDC_DEBUG, "Iniciando firmware EMG...");

    char msg[64];
    snprintf(msg, sizeof(msg), "ID leido del ADS1298: 0x%02X", id);
    emg_usb_write_line(CDC_DEBUG, msg);

    if (id == ads1298::EXPECTED_ID_ADS1298) {
        emg_usb_write_line(CDC_DEBUG, "OK: ADS1298 detectado correctamente.");
    } else {
        emg_usb_write_line(CDC_DEBUG, "ERROR: no se obtuvo el ID esperado. Valor esperado: 0x92");
    }

    while (true) {
        // TinyUSB debe ejecutarse constantemente.
        emg_usb_task();
    }

    return 0;
}
