#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "acquisition.h"
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

// CONFIG1 de arranque: HR=1 (alta resolución), DR=010 (8 kSPS en modo HR).
// Este es el data rate máximo soportado por el firmware (ver validación de
// throughput USB en CLAUDE.md); el comando MODE por CDC1 solo alterna
// HR/LP sin tocar este límite de DR.
static constexpr uint8_t DEFAULT_CONFIG1 = ads1298::CONFIG1_HR_BIT | 0x02;

// CONFIG3 de arranque: bit reservado (0x40, debe estar siempre en 1 según
// el datasheet) + PD_REFBUF=1 para habilitar el buffer de referencia
// interno de 2.4V. Sin este bit el reset de fábrica deja la referencia
// apagada (VREFP sin excitar), lo que invalida cualquier medición que
// dependa de ella (incluida la test signal interna).
static constexpr uint8_t DEFAULT_CONFIG3 = 0x40 | ads1298::CONFIG3_PD_REFBUF_BIT;

// CONFIG4 de arranque: PD_LOFF_COMP=1 para habilitar los comparadores de
// lead-off. Sin este bit el reset de fábrica los deja apagados y
// LOFF_STATP/LOFF_STATN quedan siempre en 0 (todos "conectados") sin
// importar LOFF_SENSx/LOFF_MODE/LOFF_CURRENT/LOFF_THRESHOLD.
static constexpr uint8_t DEFAULT_CONFIG4 = ads1298::CONFIG4_PD_LOFF_COMP_BIT;

static void configure_ads_defaults()
{
    ads.lock();
    ads.sendCommand(ads1298::CMD_SDATAC);
    ads.writeRegister(ads1298::REG_CONFIG1, DEFAULT_CONFIG1);
    ads.writeRegister(ads1298::REG_CONFIG3, DEFAULT_CONFIG3);
    ads.writeRegister(ads1298::REG_CONFIG4, DEFAULT_CONFIG4);
    ads.unlock();
}

// ==================================================
// Programa principal
// ==================================================

int main(void)
{
    emg_usb_init(&ads);

    // Espera corta para que la PC llegue a enumerar los CDC USB.
    sleep_ms(3000);

    uint32_t spi_hz = ads.init();
    ads.reset();

    uint8_t id = ads.readId();

    emg_usb_write_line(CDC_DEBUG, "Iniciando firmware EMG...");

    char msg[64];
    // SPI a 4 MHz nominal (ver ads1298.h: init(), bug real corregido de un
    // SPI a 1 MHz que no alcanzaba a sostener 8 kSPS). Loguea el valor real
    // que RP2040 logró (el divisor de reloj puede no dar exactamente lo
    // pedido) para confirmar en hardware que el fix quedó bien aplicado.
    snprintf(msg, sizeof(msg), "SPI ADS1298 a %lu Hz", (unsigned long)spi_hz);
    emg_usb_write_line(CDC_DEBUG, msg);

    snprintf(msg, sizeof(msg), "ID leido del ADS1298: 0x%02X", id);
    emg_usb_write_line(CDC_DEBUG, msg);

    if (id == ads1298::EXPECTED_ID_ADS1298) {
        emg_usb_write_line(CDC_DEBUG, "OK: ADS1298 detectado correctamente.");
    } else {
        emg_usb_write_line(CDC_DEBUG, "ERROR: no se obtuvo el ID esperado. Valor esperado: 0x92");
    }

    configure_ads_defaults();

    // El buffer de referencia interno recién habilitado (PD_REFBUF en
    // CONFIG3) necesita un tiempo de establecimiento antes de que VREFP sea
    // confiable; sin esta espera las primeras conversiones (y cualquier
    // medición que dependa de la referencia, como la test signal interna)
    // pueden salir con offset/amplitud incorrectos. Ver datasheet, sección
    // de secuencia de encendido, para ajustar este valor si hiciera falta.
    sleep_ms(150);

    ads.startConversions();

    // El core 1 hace polling de DRDY y lee los datos del ADS1298 por SPI;
    // el core 0 (este) se dedica exclusivamente a TinyUSB: drenar las colas
    // de datos hacia los CDC de salida y atender los comandos de
    // configuración que llegan por CDC1.
    acquisition_init(&ads);
    multicore_launch_core1(acquisition_core1_entry);

    while (true) {
        emg_usb_task();
    }

    return 0;
}
