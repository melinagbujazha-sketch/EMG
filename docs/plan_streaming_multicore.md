# Plan: streaming de datos ADS1298 por USB CDC (multicore) + API de configuración

> Nota: este plan ya fue implementado (ver `CLAUDE.md`, sección "Arquitectura de adquisición y
> streaming (multicore)", para el estado final tal como quedó en el código). Se conserva acá como
> referencia histórica de las decisiones de diseño y su justificación.

## Contexto

El firmware actual (`src/EMG.cpp`, `src/ads1298.cpp`, `src/usb_manager.cpp`) solo inicializa el
ADS1298, lee el registro ID por CDC0/CDC1 y no hace streaming de datos. El objetivo de este plan es
implementar la adquisición continua de los 8 canales EMG + estado RLD/lead-off, transmitida por los
7 CDC USB ya definidos, más una API de configuración completa por CDC1 (control de PGA, encendido de
canales, modo del filtro digital, RLD, test signals, mux de entrada y lead-off detection).

Decisiones ya validadas con el usuario en esta conversación (no se repiten discusiones):
- **Cap de data rate: 8 kSPS** (`CONFIG1.DR = 010b` en modo HR). Se validó que a esta tasa el
  throughput total (~224 KB/s en los 4 CDC de datos) queda muy por debajo del techo práctico de
  USB Full Speed (~1 MB/s agregado), con margen amplio para el ancho de banda real de una señal EMG.
- **Sin bit discriminador de canal**: como el ADS1298 muestrea los 8 canales simultáneamente, cada
  evento de DRDY ya trae ambos canales de un mismo CDC juntos en una trama de tamaño fijo.
- **Arquitectura de 2 cores**: un core dedicado a polling de DRDY + SPI (adquisición), otro core
  dedicado a TinyUSB (envío de datos + API de configuración).

## Arquitectura

### Reparto de cores

- **Core 1 (adquisición)**: loop dedicado que hace polling de DRDY (GP20). Cuando DRDY está en bajo,
  toma el mutex del bus ADS1298/SPI, envía `RDATA`, lee el status word + 8 canales de 24 bits,
  libera el mutex, arma las tramas fijas por par de canales y las empuja a colas lock-free
  (`queue_t` de `pico/util/queue.h`), una por cada CDC de datos (CDC2-5) más una para RLD/lead-off
  (CDC6). Este core **nunca** llama funciones `tud_*`.

- **Core 0 (USB / control)**: loop principal que llama `tud_task()` continuamente (único core que
  toca TinyUSB), drena las colas llenadas por el core 1 y hace `tud_cdc_n_write` hacia cada CDC de
  datos, y procesa los comandos de configuración que llegan por CDC1 (`tud_cdc_rx_cb`). Cualquier
  comando de configuración que necesite tocar el ADS1298 (RREG/WREG) toma el mismo mutex compartido
  con el core 1, hace `SDATAC` → operación → vuelve a dejar el ADS1298 en el estado de adquisición
  esperado, y libera el mutex.

  **Nota de implementación (deviación beneficiosa del plan original):** como la adquisición usa
  exclusivamente `RDATA` (nunca `RDATAC`), y el ADS1298 se deja en `SDATAC` una única vez al inicio,
  los comandos de configuración no necesitan mandar `SDATAC` ni restablecer ningún modo de streaming
  después de un `RREG`/`WREG` — alcanza con tomar el mutex. Ver `CLAUDE.md` para el detalle final.

### Sincronización del bus ADS1298/SPI

- Nuevo `mutex_t ads_mutex` (pico/mutex.h) que protege toda transacción SPI hacia el ADS1298
  (`sendCommand`, `readRegister`, `writeRegister`, y la nueva lectura de frame de datos). Se usa
  tanto desde el loop de adquisición (core 1) como desde el manejo de comandos de CDC1 (core 0).

### Formato de trama por CDC de datos

Por cada evento de DRDY, para cada CDC de datos (2 canales físicos cada uno):

```
[0]     marcador de sync (0xAA)
[1..3]  canal A, 24 bits, complemento a 2, MSB primero
[4..6]  canal B, 24 bits, complemento a 2, MSB primero
```

7 bytes por trama. A 8 kSPS: 56 KB/s por CDC de datos × 4 = 224 KB/s total, dentro del presupuesto
validado. CDC6 (RLD) manda una trama con `LOFF_STATP` (8 bits) / `LOFF_STATN` (8 bits), a la misma
cadencia (viene gratis en el status word de cada `RDATA`). `RLD_STAT` no se transmite en el stream
continuo porque vive en `CONFIG3`, no en el status word por muestra; se consulta bajo demanda con
`RREG 03`.

### API de configuración por CDC1 — comandos de texto

Se mantiene `RREG <addr_hex>` / `WREG <addr_hex> <valor_hex>` (acceso crudo, útil como escape hatch
para lo no cubierto por comandos de alto nivel) y `ID`/`READ_ID` tal como están. Además, se agregan
comandos de alto nivel que hacen read-modify-write sobre los bits correspondientes (necesario porque
varias de estas configuraciones comparten registro con otros campos, p. ej. `CHnSET` guarda a la vez
power-down, ganancia y mux del mismo canal):

| Comando | Registro(s)/bits afectados | Efecto |
|---|---|---|
| `GAIN <ch 1-8> <1\|2\|3\|4\|6\|8\|12>` | `CHnSET.GAIN[2:0]` | Ganancia del PGA del canal |
| `CHPOWER <ch 1-8> <ON\|OFF>` | `CHnSET.PDn` (+ `MUX`→`001` al apagar) | Enciende/apaga la lectura de un canal |
| `MODE <HR\|LP>` | `CONFIG1.HR` | Modo de resolución/filtro digital (ver nota de interpretación abajo) |
| `RLD <ON\|OFF>` | `CONFIG3.PD_RLD` | Enciende/apaga el amplificador RLD |
| `RLD_CH <ch 1-8> <P\|N> <0\|1>` | `RLD_SENSP`/`RLD_SENSN` | Incluye/excluye el electrodo del promedio de RLD |
| `TESTSIG <ON\|OFF>` | `CONFIG2.INT_TEST` | Habilita la señal de test interna |
| `TESTSIG_AMP <1\|2>` | `CONFIG2.TEST_AMP` | Amplitud de la señal de test |
| `TESTSIG_FREQ <SLOW\|FAST\|DC>` | `CONFIG2.TEST_FREQ[1:0]` | Frecuencia de la señal de test |
| `MUX <ch 1-8> <NORMAL\|SHORT\|RLD_MEAS\|SUPPLY\|TEMP\|TEST\|RLD_DRP\|RLD_DRN>` | `CHnSET.MUX[2:0]` | Fuente de entrada del canal (incluye rutear la test signal) |
| `LOFF_CH <ch 1-8> <P\|N> <0\|1>` | `LOFF_SENSP`/`LOFF_SENSN` | Habilita lead-off detection en ese lado del canal |
| `LOFF_MODE <DC\|AC>` | `LOFF.FLEAD_OFF[1:0]` | Tipo de excitación de lead-off |
| `LOFF_TYPE <RESISTOR\|CURRENT>` | `LOFF.VLEAD_OFF_EN` | Pullup/pulldown vs. fuente de corriente |
| `LOFF_CURRENT <6\|12\|18\|24>` | `LOFF.ILEAD_OFF[1:0]` | Magnitud de corriente de lead-off (nA) |
| `LOFF_THRESHOLD <valor>` | `LOFF.COMP_TH[2:0]` | Umbral del comparador de lead-off (código crudo 0-7) |
| `START` / `STOP` | pin/comando START-STOP del ADS1298 | Arranca/detiene la adquisición en caliente |

**Nota de interpretación:** el ADS1298 no tiene un registro de "filtro" independiente — el único
control relacionado con el filtrado interno (más allá de la tasa de datos) es el bit `CONFIG1.HR`
(High-Resolution vs Low-Power), que cambia el ancho de banda efectivo del filtro digital sinc3. Se
interpretó "prender/apagar filtros internos" como este toggle `MODE HR|LP`.

Todo comando que modifique registros toma `ads_mutex` → lee el registro (si hace falta, para
read-modify-write) → `WREG` → libera `ads_mutex` (sin SDATAC/restart, ver nota de implementación).

## Archivos tocados

- `inc/ads1298.h` / `src/ads1298.cpp`: mutex (`lock`/`unlock`), `readDataFrame()` (RDATA completo),
  `readModifyWriteRegister()`, `extractLoffStatP/N()`, `gainToCode()`, y las constantes de bits de
  registros usadas por los comandos de alto nivel.
- `inc/usb_manager.h` / `src/usb_manager.cpp`: colas de adquisición (`queue_t`), funciones
  `emg_usb_push_channel_pair`/`emg_usb_push_rld_status`, drenado de colas en `emg_usb_task()`, y el
  dispatcher completo de comandos de CDC1 (`handle_config_command()`).
- `inc/acquisition.h` / `src/acquisition.cpp` (nuevo): entry point de core 1.
- `src/EMG.cpp`: configuración de arranque del ADS1298 (data rate cap), lanzamiento de core 1, loop
  principal de core 0.
- `CMakeLists.txt`: `pico_multicore` agregado a `target_link_libraries`, `src/acquisition.cpp`
  agregado a `add_executable`.
- `CLAUDE.md`: documentación de la arquitectura final, la tabla de comandos y el formato de trama.

## Verificación

Validación en 3 etapas, en este orden:

1. **Test signals internas**: `TESTSIG ON` + `MUX <ch> TEST`, confirmar la forma de onda esperada
   (cuadrada, con la amplitud/frecuencia configurada) en el CDC de datos correspondiente.
2. **Señal continua externa por canal**: aplicar una señal DC/continua conocida directamente a cada
   par `INxP`/`INxN` y verificar que el valor digital recibido corresponde al esperado según la
   ganancia configurada (`GAIN`) y la referencia de voltaje del ADS1298.
3. **Generador de señales**: validar con una señal AC realista para confirmar la cadena completa
   SPI→USB en condiciones dinámicas, incluyendo verificar que no haya pérdida de muestras ni
   desincronización de la trama (marcador `0xAA`) a 8 kSPS sostenido.

Nota: el GUI (`emg_multicdc_gui.py`) todavía lee líneas de texto separadas por `\n` y no interpreta
los frames binarios de los CDC de datos — para las 3 etapas puede alcanzar con un script Python
auxiliar simple que lea bytes crudos y parsee el frame fijo, sin necesidad de modificar el GUI
todavía (ver "Próximos pasos" en `CLAUDE.md`).
