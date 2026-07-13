#ifndef ACQUISITION_H
#define ACQUISITION_H

#include "ads1298.h"

// Debe llamarse antes de lanzar el core 1 (multicore_launch_core1).
void acquisition_init(ads1298 *ads_device);

// Punto de entrada del core 1: hace polling de DRDY, lee el frame de
// conversión del ADS1298 por SPI (protegido por el mutex compartido con el
// core 0) y empuja las muestras a las colas de usb_manager para que el
// core 0 las transmita por USB. Nunca retorna y nunca llama funciones tud_*.
void acquisition_core1_entry();

#endif
