# EMG — Raspberry Pi Pico + ADS1298

Firmware para un sistema de adquisición de señales EMG (electromiografía) multicanal, basado en
una Raspberry Pi Pico (RP2040) conectada a un ADC frontal analógico ADS1298 (Texas Instruments)
por SPI, y expuesto a una PC mediante múltiples interfaces USB CDC (TinyUSB). Incluye además una
GUI en Python (`emg_multicdc_gui.py`) para monitorear/enviar datos por cada CDC.

> **Instrucción de mantenimiento:** este archivo debe mantenerse actualizado. Cada vez que se haga
> un cambio relevante en el proyecto (firmware, `emg_multicdc_gui.py`, estructura de archivos,
> protocolo USB/SPI, pines, build, etc.), actualiza la sección correspondiente de este `CLAUDE.md`
> en el mismo turno en que se hace el cambio, sin que el usuario tenga que pedirlo explícitamente.

## Estado actual del proyecto

Implementado:
- Inicialización y reset del ADS1298 por SPI (`ads1298` clase en `src/ads1298.cpp` / `inc/ads1298.h`).
- Lectura del registro ID (`readId`/`checkId`) para verificar comunicación con el chip.
- 7 interfaces USB CDC vía TinyUSB, con descriptores y manejo básico de comandos (`usb_manager`,
  `usb_descriptors.c`).
- Comando `ID` / `READ_ID` por el canal de configuración (CDC1) que lee y devuelve el registro ID.
- Mensajes de debug por CDC0.
- Banner de bienvenida por cada CDC al conectar (DTR→1) con el nombre de la interfaz entre
  paréntesis (p. ej. `"DATA STREAM CH1-CH2 (CDC2)"`), usado también para identificación automática
  de puertos desde la GUI de Python (ver más abajo).
- GUI de monitoreo en Python (`emg_multicdc_gui.py`, PyQt6 + pyserial) con un panel por CDC,
  auto-detección de qué puerto COM corresponde a cada CDC, y envío/recepción de texto por canal.

Pendiente (no implementado todavía, ver comentarios "Futuro" en el código):
- Streaming continuo de datos de conversión del ADS1298 (RDATAC / DRDY) hacia los CDC 2-5 (canales
  EMG) y CDC6 (RLD).
- Protocolo de configuración full-duplex real por CDC1 (por ahora solo entiende `ID`/`READ_ID` y
  responde `OK` a cualquier otra cosa).
- Escritura de registros de configuración del ADS1298 (ganancia, mux de canal, data rate, lead-off,
  RLD, etc.) desde comandos por CDC1.
- Uso de la línea DRDY (GP20) para sincronizar la lectura de datos (implementado `drdyIsLow()` pero
  aún no usado en el loop principal).

## Hardware

### Raspberry Pi Pico (RP2040)

Pines usados por el ADS1298 (definidos por defecto en el constructor de `ads1298`,
`inc/ads1298.h`, todos configurables como parámetros del constructor):

| Señal ADS1298 | Pin GPIO Pico | Función |
|---|---|---|
| DOUT (MISO) | GP16 | SPI0 MISO |
| CS          | GP17 | SPI0 chip select |
| SCLK        | GP18 | SPI0 SCK |
| DIN (MOSI)  | GP19 | SPI0 MOSI |
| DRDY        | GP20 | Entrada, data ready (activo en bajo) |
| RESET       | GP21 | Salida, reset activo en bajo |
| START       | GP22 | Salida, inicia conversiones |
| PWDN        | GP26 | Salida, power-down activo en bajo (se deja en alto = encendido) |

SPI usado: `spi0`, inicializado a 1 MHz por defecto (`ads1298::init(uint32_t spi_baudrate_hz = 1000*1000)`).

**Modo SPI del ADS1298: modo 1 → CPOL=0, CPHA=1** (confirmado en datasheet, sección 7.6/7.7:
"NOTE: SPI settings are CPOL = 0 and CPHA = 1"). Formato 8 bits, MSB first. Esto ya está bien
implementado en `ads1298::init()`.

### ADS1298 (Texas Instruments)

- Datasheet completo en `docs/ads1298.pdf` (SBAS459K, familia ADS1294/1296/1298 y variantes R).
- 8 canales simultáneos, ADC delta-sigma de 24 bits, PGA programable (ganancias 1,2,3,4,6,8,12).
- Data rate: 250 SPS a 32 kSPS (depende de modo HR/LP y registro CONFIG1.DR).
- Reloj interno de 2.048 MHz (oscilador on-chip) o reloj externo vía pin CLK.
- ID esperado del registro ID (0x00): **0x92** para el ADS1298 (ya usado como
  `EXPECTED_ID_ADS1298` en el código). Formato del ID: bits[7:5] = familia de dispositivo,
  bits[2:0] = número de canales (010 = 8 canales → ADS1298/ADS1298R).

#### Comandos SPI (opcodes) — ya definidos en `ads1298.h`

| Comando | Byte | Notas |
|---|---|---|
| WAKEUP  | 0x02 | Sale de standby |
| STANDBY | 0x04 | Entra a standby (bajo consumo, referencia sigue activa) |
| RESET   | 0x06 | Reset por software; requiere esperar 18 tCLK |
| START   | 0x08 | Inicia conversiones (alternativa al pin START) |
| STOP    | 0x0A | Detiene conversiones |
| RDATAC  | 0x10 | Modo lectura continua (modo por defecto al encender) |
| SDATAC  | 0x11 | Detiene RDATAC; necesario antes de leer/escribir registros |
| RDATA   | 0x12 | Lee un dato de conversión bajo demanda |
| RREG    | 0x20 \| addr | Leer registro(s); 2do byte = cantidad-1 |
| WREG    | 0x40 \| addr | Escribir registro(s); 2do byte = cantidad-1 |

Importante: el dispositivo arranca en modo **RDATAC** por defecto tras power-up/reset. Para poder
usar RREG/WREG hay que mandar **SDATAC** primero (el código ya hace esto en `readId()` llamando a
`sendCommand(CMD_SDATAC)` antes de leer el registro ID).

#### Registros principales (ya definidos en `ads1298.h`)

ID(0x00), CONFIG1-4, LOFF, CH1SET..CH8SET (0x05-0x0C), RLD_SENSP/N, LOFF_SENSP/N, LOFF_FLIP,
LOFF_STATP/N (solo lectura), GPIO, PACE, RESP, WCT1, WCT2. Ver tabla completa de bits en la sección
9.6 del datasheet.

#### Formato de datos de salida (para cuando se implemente streaming)

- Precedido por un **status word** de 24 bits: `1 1 0 0 LOFF_STATP[7:0] LOFF_STATN[7:0] GPIO[7:4]`.
- Cada canal: 24 bits en complemento a 2, MSB primero (17 o 19 bits útiles si DR=000/001).
- Total con 8 canales a resolución de 24 bits: 24 (status) + 8×24 = 216 bits por frame.
- A 32 kSPS / 64 kSPS los canales son de 16 bits (24 status + 8×16 = 152 bits).
- DRDY (activo en bajo) indica dato listo; se limpia en el primer flanco de bajada de SCLK tras
  DRDY, independientemente de si se lee el dato o no.
- Lectura recomendada con **RDATA** (mejor para sistemas tipo ECG/EMG donde se cambian registros
  seguido) o **RDATAC** (mejor para loggers donde los registros no cambian).

#### Timing relevante

- `tCLK` (periodo reloj maestro): 414-514 ns (reloj externo recomendado ~2.048 MHz).
- Tras RESET: esperar 18 tCLK antes de operar el chip.
- Settling time inicial de conversión depende de DR (Tabla 12 del datasheet): desde 296 tCLK (DR
  más alto, HR) hasta 36872 tCLK (DR más bajo, LP).
- Después de un cambio de entrada (step), el filtro digital tarda 3-4 × tDR en asentarse.

## Software / Firmware

### Estructura de archivos

- `src/EMG.cpp` — `main()`. Inicializa USB, inicializa/reset ADS1298, lee ID, imprime por debug,
  loop de `emg_usb_task()`.
- `inc/ads1298.h` / `src/ads1298.cpp` — driver del ADS1298 (SPI, comandos, registros).
- `inc/usb_manager.h` / `src/usb_manager.cpp` — capa sobre TinyUSB para gestionar los 7 CDC.
- `src/usb_descriptors.c` — descriptores USB (device, config, strings) para 7 interfaces CDC.
- `inc/tusb_config.h` — configuración de TinyUSB (`CFG_TUD_CDC = 7`, buffers RX/TX de 64 bytes).
- `CMakeLists.txt` — build con Pico SDK 2.2.0, librerías `pico_stdlib`, `hardware_spi`,
  `tinyusb_device`, `tinyusb_board`. `pico_enable_stdio_usb/uart` están **desactivados** porque el
  proyecto usa su propio CDC de TinyUSB en vez del stdio estándar del SDK.

### Mapeo de canales USB CDC (definido en `inc/usb_manager.h`)

| CDC # | Uso |
|---|---|
| 0 | Debug (mensajes de texto, solo lectura desde la PC) |
| 1 | Configuración full-duplex (comandos PC→Pico y respuestas) |
| 2 | Datos ADS canales 1 y 2 |
| 3 | Datos ADS canales 3 y 4 |
| 4 | Datos ADS canales 5 y 6 |
| 5 | Datos ADS canales 7 y 8 |
| 6 | RLD (right leg drive) |

Es decir, 2 canales físicos del ADS1298 por cada interfaz CDC de datos (2,3,4,5), tal como pidió el
usuario. Actualmente CDC2-5 y CDC6 solo responden "OK" a cualquier byte recibido (placeholder,
`tud_cdc_rx_cb` en `usb_manager.cpp`); todavía no envían datos de conversión reales.

Cada CDC tiene su propio endpoint de notificación, IN y OUT (ver `usb_descriptors.c`), VID/PID
`0xCafe`/`0x4000|...`, fabricante "Raspberry Pi", producto "EMG ADS1298".

### Build

- Requiere Pico SDK 2.2.0 + toolchain ARM GCC 14_2_Rel1 (gestionado por la extensión de VS Code
  para Pico, ver `.vscode/settings.json` y `CMakeLists.txt`).
- `PICO_BOARD = pico` (RP2040, no Pico 2/RP2350).
- Generador: Ninja.

## Herramienta PC: `emg_multicdc_gui.py`

GUI de escritorio (PyQt6 + pyserial) para monitorear y probar los 7 CDC del firmware desde la PC.
Requisitos: `pip install pyqt6 pyserial`. Uso: `python emg_multicdc_gui.py`.

### Qué hace

- Un panel (`CdcPanel`) por cada uno de los 7 CDC (`CDC_NAMES`), cada uno con: estado de conexión,
  área de texto de recepción, campo de envío de texto y botones enviar/limpiar.
- Un combo de asignación de puerto COM por CDC + spinbox de baudrate + botones "Buscar puertos",
  "Auto-detectar CDC", "Conectar seleccionados" y "Desconectar todo".
- Cada conexión serie corre en su propio `QThread`/`SerialWorker` (lectura por líneas `\n`,
  no bloqueante para la UI).

### Auto-detección de puertos COM (botón "Auto-detectar CDC")

Problema que resuelve: Windows no asigna los COM en el mismo orden que los índices de interfaz CDC
del firmware, y en este entorno **pyserial no expone de forma fiable** ni el número de interfaz USB
(`p.interface` devuelve `None`) ni el `hwid` crudo con `MI_XX` — solo VID, PID, número de serie y un
`location` no confiable (probado empíricamente: valores duplicados/incompletos como `x.4` repetido
en dos puertos distintos). Por lo tanto **no se puede usar el hwid/interfaz para saber qué puerto es
qué CDC**.

Solución implementada: identificación en tiempo de ejecución leyendo el banner que el firmware ya
manda por cada CDC en `tud_cdc_line_state_cb` cuando DTR pasa a 1 (`send_interface_banner()` en
`src/usb_manager.cpp`), que incluye el texto `"(CDC0)"` .. `"(CDC6)"`.

Funciones clave en `emg_multicdc_gui.py`:
- `parse_hwid(hwid)` — extrae VID, PID y número de serie del hwid que arma pyserial. **Importante**:
  el formato real es `"USB VID:PID=CAFE:4007 SER=... LOCATION=..."` (regex `VID:PID=XXXX:XXXX`), NO
  el formato crudo de Windows `VID_XXXX&PID_XXXX` (aunque ese también se intenta como fallback). Este
  fue un bug real ya corregido: la regex original solo buscaba el formato crudo y por eso no
  encontraba ningún dispositivo `0xCAFE` pese a que los puertos sí aparecían.
- `group_ports_by_serial()` — agrupa los puertos COM cuyo VID sea `0xCAFE` (VID del firmware, ver
  `EMG_VENDOR_ID` y `src/usb_descriptors.c: CDC_EXAMPLE_VID`) por número de serie de la placa física
  (para soportar varias Pico conectadas a la vez).
- `identify_cdc_index(device)` — abre el puerto brevemente, baja DTR, limpia el buffer, sube DTR
  (fuerza la transición 0→1 que dispara el banner) y busca `"(CDCn)"` en lo leído con la regex
  `_BANNER_CDC_RE`. Devuelve el índice 0-6 o `None` si no se pudo identificar (puerto ocupado, sin
  respuesta, etc.).
- `find_emg_boards(progress_cb=None)` — combina las dos anteriores y devuelve
  `{numero_de_serie: {cdc_index: puerto_com}}`.
- `MainWindow.auto_detect_ports()` — usa lo anterior, deja elegir la placa si hay más de una
  conectada, excluye puertos ya abiertos por conexiones activas del propio programa, y rellena los
  combos de asignación automáticamente. Mientras escanea (abre cada puerto ~0.35s) deshabilita el
  botón y muestra progreso.

Verificado manualmente en este entorno (Windows) con una Pico conectada: el orden real de COM no
coincidía con el de los CDC (ejemplo real: COM4→CDC6, COM5→CDC1, COM6→CDC2, COM7→CDC5, COM8→CDC4,
COM9→CDC3, COM10→CDC0), confirmando que la detección por banner es necesaria y funciona.

### Limitaciones conocidas

- La auto-detección abre/cierra cada puerto candidato secuencialmente (síncrono en el hilo de UI,
  con `QApplication.processEvents()` para no congelar del todo la ventana); tarda unos segundos con
  7 CDC.
- No puede auto-detectar puertos que ya estén conectados por otra aplicación (se reporta como no
  identificado); los que ya están conectados por el propio programa se excluyen del escaneo a
  propósito.
- Si en el futuro se cambia `CFG_TUD_CDC` o el VID/PID del firmware (`src/usb_descriptors.c`), hay
  que actualizar `EMG_VENDOR_ID` en `emg_multicdc_gui.py` a juego.

## Próximos pasos sugeridos (a partir de lo que falta)

1. Implementar el loop de streaming: usar DRDY (o RDATAC) para leer los 8 canales + status word por
   SPI y despachar cada par de canales al CDC correspondiente (2-5), y el estado RLD/lead-off al
   CDC6.
2. Definir un protocolo binario o de texto para CDC1 que permita configurar registros del ADS1298
   (ganancia por canal, data rate, referencia, RLD, lead-off) en tiempo real.
3. Considerar mover a modo RDATAC continuo con IRQ en DRDY en lugar de polling, para no bloquear
   `emg_usb_task()`.
4. Definir el formato de los datos que se envían por los CDC de datos (¿24 bits crudos? ¿escalados
   a mV? ¿con timestamp?) — pendiente de decisión de diseño.
5. En `emg_multicdc_gui.py`: una vez que el firmware transmita datos binarios reales de EMG por los
   CDC 2-5, la lectura por líneas de texto (`\n`) de `SerialWorker` dejará de servir; habrá que
   definir un framing binario (o mantener texto solo para debug/config y binario para datos) y
   graficar las señales (por ejemplo con pyqtgraph) en vez de solo mostrar texto.
6. Mover `identify_cdc_index`/auto-detección a un hilo (`QThread`) en vez de bloquear la UI con
   `processEvents()`, sobre todo si se agregan más CDC o el escaneo se vuelve más lento.
