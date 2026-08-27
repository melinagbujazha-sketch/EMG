# EMG — Raspberry Pi Pico + ADS1298

Firmware para un sistema de adquisición de señales EMG (electromiografía) multicanal, basado en
una Raspberry Pi Pico (RP2040) conectada a un ADC frontal analógico ADS1298 (Texas Instruments)
por SPI, y expuesto a una PC mediante múltiples interfaces USB CDC (TinyUSB). Incluye además una
GUI en Python (`emg_multicdc_gui.py`) pensada para uso clínico: conexión de un solo botón (sin
exponer CDC ni puertos COM), sidebar de canales/filtros/RLD-lead-off y grilla de plots en tiempo
real, con el detalle de debug (mapeo manual de puertos, texto crudo, configuración granular) en
una pestaña "Avanzado" aparte.

> Guía paso a paso para dejar el proyecto andando desde cero (SDK de Pico, compilar/flashear el
> firmware, dependencias de Python y cómo correr la GUI) en [`README.md`](README.md). Este
> `CLAUDE.md` es la referencia técnica; el `README.md` es la guía de instalación/uso.

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
- Mensajes de debug por CDC0.
- Banner de bienvenida por cada CDC al conectar (DTR→1) con el nombre de la interfaz entre
  paréntesis (p. ej. `"DATA STREAM CH1-CH2 (CDC2)"`), usado también para identificación automática
  de puertos desde la GUI de Python (ver más abajo).
- GUI clínica en Python (`emg_multicdc_gui.py`, PyQt6 + pyserial + pyqtgraph + numpy + scipy), con
  4 pestañas: **"Monitor"** (sidebar de canales/ganancia/filtros/RLD-lead-off al 20% + grilla de
  plots en tiempo real al 80%, conexión de un solo botón), **"Señales de prueba"** (test signal
  interna simplificada), **"Análisis"** (carga offline de una grabación CSV previa, plot con
  selección de rango arrastrable y recuadro de amplitud pico a pico, sin necesitar el equipo
  conectado) y **"Avanzado"** (todo el detalle de debug: mapeo manual de puertos por CDC, texto
  crudo de CDC0/CDC1, estado completo de lead-off/RLD y tabla granular de configuración). Filtros
  notch/pasaaltos/pasabajos implementados en software (scipy.signal), no en firmware. Ver sección
  "Herramienta PC" más abajo para el detalle completo.
- **Streaming continuo multicore** de los 8 canales EMG + estado de lead-off por USB, y una **API de
  configuración completa** por CDC1 (ganancia, encendido de canales, modo HR/LP, RLD, test signals,
  mux de entrada, lead-off detection). Ver sección "Arquitectura de adquisición y streaming" más
  abajo para el detalle completo.

Pendiente:
- Sin pruebas en hardware todavía de la nueva arquitectura de streaming ni del GUI actualizado
  (recién implementados; falta la validación en 3 etapas descrita en "Próximos pasos").

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
- **Filtro EMI de entrada: siempre activo, no es configurable.** Datasheet, sección 9.3.1.1 "EMI
  Filter" (p. 26): "An RC filter at the input acts as an EMI filter on all channels. The –3-dB
  filter bandwidth is approximately 3 MHz." Es un filtro RC pasivo cableado en el silicio, uno por
  canal, sin ningún bit de registro que lo habilite/deshabilite ni que aparezca en `ads1298.h` — no
  hay nada que el firmware necesite hacer para "activarlo", ya está funcionando por diseño. Su
  ancho de banda (~3 MHz) está muy por encima de cualquier corte de los filtros de software de la
  GUI (notch/pasaaltos/pasabajos hasta 500 Hz, ver "Filtros de señal" más abajo) o del propio
  data rate del ADC (hasta 8 kSPS): no tiene ningún efecto perceptible sobre la señal EMG, sirve
  para atenuar interferencia de radiofrecuencia muy por fuera de la banda de interés.

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

#### Formato de datos de salida del ADS1298 (por SPI, vía RDATA)

- Precedido por un **status word** de 24 bits: `1 1 0 0 LOFF_STATP[7:0] LOFF_STATN[7:0] GPIO[7:4]`.
- Cada canal: 24 bits en complemento a 2, MSB primero (17 o 19 bits útiles si DR=000/001).
- Total con 8 canales a resolución de 24 bits: 24 (status) + 8×24 = 216 bits por frame (28 bytes:
  1 de opcode RDATA + 27 de respuesta). A data rates de 8 kSPS (el cap usado en este firmware) el
  frame siempre es de 24 bits/canal.
- DRDY (activo en bajo) indica dato listo; se limpia en el primer flanco de bajada de SCLK tras
  DRDY, independientemente de si se lee el dato o no.
- El firmware usa **RDATA** (no RDATAC) por cada flanco de DRDY: `ads1298::readDataFrame()` en
  `src/ads1298.cpp` hace la transacción SPI completa (opcode + 27 bytes de respuesta) en una sola
  llamada y devuelve el status word (3 bytes) + los 8 canales ya con signo extendido a `int32_t`.
  `ads1298::extractLoffStatP/N()` separan `LOFF_STATP`/`LOFF_STATN` del status word (ver Figura 61
  del datasheet, "Status Word Content", para el detalle bit a bit).

#### Timing relevante

- `tCLK` (periodo reloj maestro): 414-514 ns (reloj externo recomendado ~2.048 MHz).
- Tras RESET: esperar 18 tCLK antes de operar el chip.
- Settling time inicial de conversión depende de DR (Tabla 12 del datasheet): desde 296 tCLK (DR
  más alto, HR) hasta 36872 tCLK (DR más bajo, LP).
- Después de un cambio de entrada (step), el filtro digital tarda 3-4 × tDR en asentarse.

## Arquitectura de adquisición y streaming (multicore)

Implementada para cumplir con el presupuesto de throughput USB validado (ver sección siguiente) y
para no bloquear la adquisición con el procesamiento de comandos de configuración. El plan de
diseño original (con el contexto completo de las decisiones tomadas) quedó guardado en
`docs/plan_streaming_multicore.md`.

### Reparto de cores

- **Core 1** (`acquisition_core1_entry()` en `src/acquisition.cpp`, lanzado desde `EMG.cpp` con
  `multicore_launch_core1`): hace polling de `drdyIsLow()` en loop cerrado. Cuando DRDY está en
  bajo, toma `ads.lock()`, llama `ads.readDataFrame(status, channels)` (un solo `RDATA` por SPI),
  libera el lock, y empuja los 4 pares de canales + el estado de lead-off a las colas de
  `usb_manager` (`emg_usb_push_channel_pair`/`emg_usb_push_rld_status`). **Nunca llama funciones
  `tud_*`.**
- **Core 0** (`main()` en `EMG.cpp`, loop de `emg_usb_task()`): único core que toca TinyUSB. Llama
  `tud_task()`, drena las 5 colas (4 de datos + 1 de RLD, `queue_t` de `pico/util/queue.h`,
  definidas en `usb_manager.cpp`) y manda los frames binarios por `tud_cdc_n_write`. También atiende
  los comandos de configuración de CDC1 (`tud_cdc_rx_cb` → `handle_config_command()`), que usan
  `ads.lock()`/`ads.unlock()` alrededor de cualquier `RREG`/`WREG` para no pisar una transacción SPI
  en curso del core 1.
- **Mutex compartido**: `ads1298::_bus_mutex` (pico/mutex.h), inicializado en `ads1298::init()`,
  expuesto como `ads.lock()`/`ads.unlock()`. Protege *toda* transacción SPI hacia el chip. Como el
  firmware nunca vuelve a modo `RDATAC` (usa `RDATA` exclusivamente, ver más abajo), los comandos
  `RREG`/`WREG` no necesitan mandar `SDATAC` antes de cada operación ni restablecer ningún modo de
  streaming después — alcanza con tomar el mutex.
- Si en el futuro se cambia el mapeo de canales por CDC o el número de interfaces de datos, hay que
  actualizar `DATA_CDC_COUNT` en `usb_manager.cpp` y los índices fijos usados en
  `acquisition_core1_entry()`.

### Presupuesto de throughput USB (validado)

- **Cap de data rate: 8 kSPS** (`CONFIG1.DR = 010b` en modo HR, `DEFAULT_CONFIG1` en `EMG.cpp`).
  A esta tasa el throughput total (~224 KB/s en los 4 CDC de datos, trama de 7 bytes c/u) queda muy
  por debajo del techo práctico de USB Full Speed (~1 MB/s agregado; RP2040 no tiene High Speed),
  con margen amplio para el ancho de banda real de una señal EMG (~20 Hz-1 kHz). El comando `MODE`
  (ver tabla de comandos) solo alterna HR/LP sin tocar este límite de `DR`.
- **`CONFIG3.PD_REFBUF` (buffer de referencia interno) debe habilitarse explícitamente.**
  `configure_ads_defaults()` en `EMG.cpp` escribe `DEFAULT_CONFIG3 = 0x40 |
  CONFIG3_PD_REFBUF_BIT` (bit reservado siempre en 1 + PD_REFBUF=1), y espera
  150 ms (`sleep_ms(150)` en `main()`) antes de `startConversions()` para que
  la referencia se establezca. **Bug real corregido:** el firmware nunca
  tocaba `CONFIG3`, así que el chip arrancaba con el buffer de referencia
  apagado (reset de fábrica). Con VREFP sin excitar, cualquier medición que
  dependa de la referencia interna de 2.4V queda con amplitud/offset
  indefinidos — esto se detectó analizando estadísticamente una captura de
  la test signal interna (`MUX 1 TEST` + `TESTSIG ON`) que mostraba una
  distribución ~gaussiana con media ~45 mV y desvío ~5 mV (en vez de los
  ~0 mV ± 1-2 mV esperados), aunque con una periodicidad real de ~4 Hz que
  coincidía con `TESTSIG_FREQ FAST` (confirmando que el MUX/test-signal sí
  se estaba aplicando, pero con la referencia inválida). Como esto afecta
  también a las mediciones reales (no solo a la test signal), es un bug que
  hay que validar en hardware antes de confiar en cualquier lectura de EMG.
- Como el ADS1298 muestrea los 8 canales simultáneamente, cada evento de DRDY ya trae ambos canales
  de un mismo CDC juntos en una sola trama — no hace falta un bit discriminador de canal.
- **`CONFIG4.PD_LOFF_COMP` (comparadores de lead-off) debe habilitarse explícitamente.**
  `configure_ads_defaults()` escribe `DEFAULT_CONFIG4 = CONFIG4_PD_LOFF_COMP_BIT` (0x04).
  **Bug real corregido:** el firmware nunca tocaba `CONFIG4`, así que los comparadores de lead-off
  quedaban permanentemente apagados (reset de fábrica) — `LOFF_STATP`/`LOFF_STATN` leían siempre 0
  (todos "conectados") sin importar `LOFF_CH`/`LOFF_MODE`/`LOFF_TYPE`/`LOFF_CURRENT`/
  `LOFF_THRESHOLD`. Detectado porque la detección de lead-off no reaccionaba nunca en la GUI aunque
  hubiera canales sin electrodo conectado.

### Formato de trama binaria por USB

Cada CDC de datos (CDC2-5, 2 canales físicos cada uno) manda, por cada evento de DRDY:

```
[0]     marcador de sync (0xAA)
[1..3]  canal A, 24 bits, complemento a 2, MSB primero
[4..6]  canal B, 24 bits, complemento a 2, MSB primero
```

7 bytes por trama (`write_channel_be24()` en `usb_manager.cpp`). CDC6 (RLD/lead-off) manda:

```
[0] marcador de sync (0xAA)
[1] LOFF_STATP (8 bits, uno por canal)
[2] LOFF_STATN (8 bits, uno por canal)
```

3 bytes por trama, a la misma cadencia que los datos (viene gratis en el status word de cada
`RDATA`, no requiere una lectura de registro aparte). `RLD_STAT` (si conectado o no el electrodo
RLD) **no** se transmite en este stream continuo porque vive en `CONFIG3` (no en el status word por
muestra) y leerlo cada muestra agregaría tráfico SPI innecesario; se puede consultar bajo demanda
con `RREG 03` desde CDC1.

**Escrituras USB atómicas:** `emg_usb_write()`, `emg_usb_write_line()`, `drain_data_queue()` y
`drain_rld_queue()` verifican `tud_cdc_n_write_available(itf)` **antes** de escribir (y antes de
sacar el elemento de la cola, en el caso de las tramas de datos): si no entra el mensaje/trama
completo en el FIFO de TX de TinyUSB, se descarta entero en vez de escribirlo a medias. Esto
corrige un bug real observado en hardware: `tud_cdc_n_write()` trunca en silencio si no hay
espacio, y una trama de datos a medias corrompía el framing de todas las tramas siguientes hasta el
próximo resincronismo (se veía como picos de ruido absurdos en la señal), y un mensaje de texto a
medias producía líneas de debug concatenadas/truncadas en CDC0 (visto en la práctica con ráfagas de
comandos, p. ej. los 16 `RREG` de `read_current_configuration()` al conectar CDC1 desde la GUI).

### API de configuración por CDC1 — comandos de texto

Implementados en `handle_config_command()` (`src/usb_manager.cpp`). Todos responden `OK` o
`ERROR: <motivo>` (excepto `RREG`/`ID` que devuelven el valor leído). Los comandos que modifican un
subconjunto de bits de un registro compartido (p. ej. `CHnSET`) hacen read-modify-write vía
`ads1298::readModifyWriteRegister()`, preservando el resto de los campos.

Cada comando recibido por CDC1 se loguea primero por CDC_DEBUG (CDC0) como
`"[CFG] comando recibido: <línea cruda>"`, **antes** de tokenizarlo (porque `strtok()` muta el
buffer insertando terminadores nulos en los separadores) — sirve para confirmar en tiempo real qué
API se está ejecutando realmente al ir probando contra hardware.

| Comando | Registro(s)/bits afectados | Efecto |
|---|---|---|
| `ID` / `READ_ID` | `REG_ID` | Lee el registro ID del ADS1298 |
| `RREG <addr_hex>` | cualquiera | Lee un registro crudo (escape hatch) |
| `WREG <addr_hex> <valor_hex>` | cualquiera | Escribe un registro crudo (escape hatch) |
| `GAIN <ch 1-8> <1\|2\|3\|4\|6\|8\|12>` | `CHnSET.GAIN[2:0]` | Ganancia del PGA del canal |
| `CHPOWER <ch 1-8> <ON\|OFF>` | `CHnSET.PDn` (+ `MUX`→`001` al apagar) | Enciende/apaga la lectura de un canal |
| `MODE <HR\|LP>` | `CONFIG1.HR` | High-Resolution vs Low-Power (ancho de banda del filtro digital) |
| `RLD <ON\|OFF>` | `CONFIG3.PD_RLD` | Enciende/apaga el amplificador RLD (pese al nombre `PD_`, 1=habilitado) |
| `RLD_CH <ch 1-8> <P\|N> <0\|1>` | `RLD_SENSP`/`RLD_SENSN` | Incluye/excluye el electrodo del promedio de RLD |
| `TESTSIG <ON\|OFF>` | `CONFIG2.INT_TEST` | Habilita la señal de test interna |
| `TESTSIG_AMP <1\|2>` | `CONFIG2.TEST_AMP` | Amplitud de la señal de test |
| `TESTSIG_FREQ <SLOW\|FAST\|DC>` | `CONFIG2.TEST_FREQ[1:0]` | Frecuencia de la señal de test |
| `MUX <ch 1-8> <NORMAL\|SHORT\|RLD_MEAS\|SUPPLY\|TEMP\|TEST\|RLD_DRP\|RLD_DRN>` | `CHnSET.MUX[2:0]` | Fuente de entrada del canal (incluye rutear la test signal) |
| `LOFF_CH <ch 1-8> <P\|N> <0\|1>` | `LOFF_SENSP`/`LOFF_SENSN` | Habilita lead-off detection en ese lado del canal |
| `LOFF_MODE <DC\|AC>` | `LOFF.FLEAD_OFF[1:0]` | Tipo de excitación de lead-off |
| `LOFF_TYPE <RESISTOR\|CURRENT>` | `LOFF.VLEAD_OFF_EN` | Pullup/pulldown vs. fuente de corriente |
| `LOFF_CURRENT <6\|12\|18\|24>` | `LOFF.ILEAD_OFF[1:0]` | Magnitud de corriente de lead-off (nA) |
| `LOFF_THRESHOLD <0-7>` | `LOFF.COMP_TH[2:0]` | Código crudo del umbral del comparador (ver Tabla LOFF del datasheet para el % correspondiente) |
| `START` / `STOP` | pin START del ADS1298 | Arranca/detiene la adquisición en caliente |

Cualquier otro texto recibido por CDC1 responde `OK` (comportamiento histórico, sin cambios).

## Software / Firmware

### Estructura de archivos

- `src/EMG.cpp` — `main()`. Inicializa USB, inicializa/reset ADS1298, configura el data rate por
  defecto (8 kSPS), lanza el core 1 (`acquisition_core1_entry`) y deja el loop principal en
  `emg_usb_task()`.
- `inc/ads1298.h` / `src/ads1298.cpp` — driver del ADS1298 (SPI, comandos, registros, mutex,
  `readDataFrame`, helpers de read-modify-write y de extracción del status word).
- `inc/acquisition.h` / `src/acquisition.cpp` — entry point del core 1 (loop de polling de DRDY +
  lectura SPI + llenado de colas).
- `inc/usb_manager.h` / `src/usb_manager.cpp` — capa sobre TinyUSB para gestionar los 7 CDC: banner
  de bienvenida, colas de adquisición, drenado de colas hacia USB, y la API de configuración de
  CDC1 (tabla de comandos arriba).
- `src/usb_descriptors.c` — descriptores USB (device, config, strings) para 7 interfaces CDC.
- `inc/tusb_config.h` — configuración de TinyUSB (`CFG_TUD_CDC = 7`, buffers RX/TX de 64 bytes).
- `CMakeLists.txt` — build con Pico SDK 2.2.0, librerías `pico_stdlib`, `hardware_spi`,
  `pico_multicore`, `tinyusb_device`, `tinyusb_board`. `pico_enable_stdio_usb/uart` están
  **desactivados** porque el proyecto usa su propio CDC de TinyUSB en vez del stdio estándar del SDK.

### Mapeo de canales USB CDC (definido en `inc/usb_manager.h`)

| CDC # | Uso |
|---|---|
| 0 | Debug (mensajes de texto, solo lectura desde la PC) |
| 1 | Configuración full-duplex (comandos PC→Pico y respuestas, ver tabla de comandos arriba) |
| 2 | Datos ADS canales 1 y 2 (streaming binario, 7 bytes/trama) |
| 3 | Datos ADS canales 3 y 4 (streaming binario, 7 bytes/trama) |
| 4 | Datos ADS canales 5 y 6 (streaming binario, 7 bytes/trama) |
| 5 | Datos ADS canales 7 y 8 (streaming binario, 7 bytes/trama) |
| 6 | RLD / lead-off status (streaming binario, 3 bytes/trama) |

Cada CDC tiene su propio endpoint de notificación, IN y OUT (ver `usb_descriptors.c`), VID/PID
`0xCafe`/`0x4000|...`, fabricante "Raspberry Pi", producto "EMG ADS1298".

### Build

- Requiere Pico SDK 2.2.0 + toolchain ARM GCC 14_2_Rel1 (gestionado por la extensión de VS Code
  para Pico, ver `.vscode/settings.json` y `CMakeLists.txt`).
- `PICO_BOARD = pico` (RP2040, no Pico 2/RP2350).
- Generador: Ninja.

## Herramienta PC: `emg_multicdc_gui.py`

GUI de escritorio (PyQt6 + pyserial + pyqtgraph + numpy + scipy) pensada para uso clínico:
conexión de un solo botón, sidebar de canales/filtros/RLD-lead-off y grilla de plots en tiempo
real, dejando todo el detalle de CDC/puertos/debug en una pestaña "Avanzado" separada.
Requisitos: `pip install pyqt6 pyserial pyqtgraph numpy scipy`. Uso: `python emg_multicdc_gui.py`.
El plan de diseño original (streaming binario + panel de configuración) quedó en
`docs/plan_gui_streaming_config.md`; el rediseño clínico (sidebar, pestañas, filtros software) se
decidió en sesión, sin un plan_*.md dedicado (ver este mismo `CLAUDE.md` para el detalle vigente).

### Qué hace

- Ventana con 4 pestañas (`QTabWidget`): **"Monitor"**, **"Señales de prueba"**, **"Análisis"** y
  **"Avanzado"**. Barra superior común a las 4: botones **"Conectar equipo"**/**"Desconectar equipo"** + label de
  estado agregado (verde = las 7 interfaces conectadas, ámbar = parcial, rojo = ninguna),
  **"Iniciar grabación"**/**"Detener grabación"** + label de estado (ver "Grabación a CSV" más
  abajo), y **"Pantalla completa (F11)"** (`MainWindow.toggle_fullscreen()`). La ventana arranca
  maximizada.
- **"Monitor"** (pestaña principal, uso clínico): `QSplitter` horizontal con la sidebar a la
  izquierda (~20% del ancho, `setStretchFactor(0, 1)` vs `(1, 4)`) y el área de plots a la derecha
  (~80%):
  - Sidebar (`MainWindow._build_sidebar()`), sin ningún texto de nota que ocupe espacio de más —
    todo son controles:
    - **"Canales"**: checkbox de visibilidad + combo de ganancia (`GAIN_VALUES`) por cada canal
      físico (1-8). Los 8 `SingleChannelPlot` **siempre** existen y reciben datos (`append_batch`)
      aunque estén ocultos; destildar solo los saca del área de plots y de `refresh_plots()`. El
      combo de ganancia manda `GAIN <ch> <valor>` (`_on_sidebar_gain_changed`) y se mantiene
      sincronizado con el combo equivalente de `ConfigPanel` en "Avanzado" en ambos sentidos
      (incluida una relectura real de configuración al conectar, ver más abajo).
    - **"Escala de los plots"**: combo Automática/Manual + spinbox+unidad para el eje X (s/ms) y
      para el eje Y (mV/µV/V). Ver "Escala de los plots" más abajo. **Importante para verificar
      filtros a ojo:** en modo Automática (el default) el autorange de pyqtgraph reescala el eje Y
      para llenar el panel con la amplitud que haya en cada momento — si un filtro reduce la
      amplitud real (ruido de línea, deriva DC), el plot hace zoom y la forma se ve visualmente
      igual aunque los mV cambiaron. Para confirmar que un filtro realmente está actuando, pasar a
      modo Manual con un rango Y fijo antes de activarlo/desactivarlo.
    - **"Filtros (canales visibles)"**: 3 combos — Notch (Apagado/50 Hz/60 Hz), Pasaaltos
      (Apagado/1 Hz/2 Hz/5 Hz/10 Hz/20 Hz/50 Hz), Pasabajos (Apagado/100 Hz/125 Hz/150 Hz/500 Hz).
      Ver "Filtros de señal" más abajo.
    - **"RLD / Lead-off"**: dos checkboxes ON/OFF simples, sin nada más. RLD manda primero
      `RLD_CH <ch> P/N 1` (ON) o `0` (OFF) para los 8 canales y recién después `RLD ON`/`RLD OFF`
      (`_on_rld_toggle`) — **bug real corregido:** antes solo mandaba `RLD ON/OFF`
      (`CONFIG3.PD_RLD`, que únicamente enciende el amplificador), sin tocar nunca
      `RLD_SENSP`/`RLD_SENSN` (reset = 0x00, ningún canal incluido); el amplificador quedaba
      encendido pero sin ninguna entrada real, así que no hacía rechazo de modo común y no había
      cambio perceptible al conectar el electrodo de RLD. Lead-off manda `LOFF_CH <ch> P/N 1` (ON) o
      `0` (OFF) para los 8 canales de una vez, usando los parámetros finos (`LOFF_MODE`/
      `LOFF_TYPE`/`LOFF_CURRENT`/`LOFF_THRESHOLD`) que estén configurados en "Avanzado" (esa
      referencia solo vive en este `CLAUDE.md`, no como texto en la sidebar); ver también el bug de
      firmware corregido en `CONFIG4.PD_LOFF_COMP` (sección "Presupuesto de throughput USB") sin el
      cual esto nunca funcionaba, sea cual sea la configuración de la GUI — **ese fix requiere
      reflashear la placa; hasta no hacerlo, el lead-off sigue sin mostrar desconexiones aunque la
      GUI ya esté corregida.**
      El aviso de desconexión se muestra ahora en `SingleChannelPlot.set_leadoff()`, como un cartel
      "Electrodo desconectado" bajo el título de cada canal en el área de plots — ver más abajo
      ("Cartel de lead-off en cada plot") — no en la sidebar.
  - Área de plots (`MainWindow._rebuild_channel_grid()`): sin scrollbar — siempre se reparte para
    que los canales tildados entren en el espacio disponible, priorizando ver más escala horizontal
    (tiempo) que vertical:
    - **1-3 canales visibles**: una sola columna (`QVBoxLayout` con `addWidget(plot, stretch=1)`
      por canal), ocupando el 100% del ancho si es 1 solo canal.
    - **4-8 canales visibles**: dos columnas lado a lado dentro de `channel_grid`
      (`QHBoxLayout`) — la izquierda se lleva `ceil(n/2)` canales y la derecha `floor(n/2)` (p. ej.
      5 canales -> 3 y 2; 7 canales -> 4 y 3; 8 canales -> 4 y 4), cada columna con su propio
      `QVBoxLayout` interno repartiendo la altura por igual entre sus canales.
    - Los contenedores "columna" (`QWidget` con `QVBoxLayout`) son transitorios y se recrean
      enteros en cada llamada; los `SingleChannelPlot` en cambio son permanentes — se re-parentan a
      la columna nueva (`addWidget` los reparenta automáticamente) antes de que Qt destruya
      efectivamente la columna vieja (`deleteLater()` corre recién en el próximo ciclo de eventos).
    - `SingleChannelPlot`: eje Y en mV y eje X en **segundos**, ambos con autoescala por defecto (o
      rango fijo en modo manual, ver más abajo); tipografía/interlineado reducidos (título del
      `QGroupBox`, label de estado, ejes y ticks de pyqtgraph todos a ~9-10px, márgenes/spacing del
      layout interno recortados) para que hasta 4 canales apilados en una misma columna entren sin
      necesitar scroll. `MainWindow.pair_channels` rutea los batches de `poll_process_queues`.
    - **Cartel de lead-off en cada plot** (`SingleChannelPlot.leadoff_label`, bajo el título "Canal
      N"): vacío y sin fondo la mayor parte del tiempo; solo cuando `MainWindow._update_leadoff_status`
      detecta ese canal desconectado (y "Lead-off activo" está tildado) se pinta en rojo con el texto
      "Electrodo desconectado" (`SingleChannelPlot.set_leadoff()`). **Historial:** antes ahí se
      mostraba el estado de conexión del CDC de datos ("Conectado a COMx"/"Desconectado",
      `set_connected()`); se sacó porque confundía con lead-off y era redundante con el label
      agregado de conexión en la barra superior. También existió brevemente un cartel de lead-off
      aparte en la sidebar "Canales" (con fondo rojo fijo, visible aunque no hubiera desconexión
      real) — se sacó por lo mismo y se consolidó acá, en el lugar donde el usuario ya esperaba ver
      el estado de cada canal.
- **"Señales de prueba"** (`TestSignalPanel`, clase nueva e independiente de `ConfigPanel`): combos
  `TESTSIG`/`TESTSIG_AMP`/`TESTSIG_FREQ` + botones `START`/`STOP`, y un checkbox por canal que manda
  `MUX <ch> TEST` (tildado) o `MUX <ch> NORMAL` (destildado) — versión simplificada del combo
  completo de 8 modos de MUX que sigue existiendo en "Avanzado". La onda se ve en los plots de
  "Monitor" (siguen recibiendo datos sin importar la pestaña activa). Nota: como es un panel
  independiente del `ConfigPanel` de "Avanzado", cambiar TESTSIG/MUX desde un lado no actualiza
  visualmente los controles equivalentes del otro (ambos solo mandan comandos "fire and forget",
  igual que el resto de la API de CDC1).
- **"Análisis"** (`RecordingAnalysisPanel`, pestaña nueva e independiente de la adquisición en
  vivo): carga un CSV grabado por "Iniciar grabación" (mismo formato tidy `tiempo_s,canal,mV`, ver
  "Grabación a CSV" más abajo) para revisarlo offline, sin necesitar el equipo conectado. Layout
  `QHBoxLayout` con el plot a la izquierda (~80%, `stretch=4`) y un recuadro a la derecha (~20%,
  `stretch=1`, mismo criterio de reparto que el `QSplitter` 20/80 de "Monitor"):
  - Botón **"Cargar grabación..."** (`QFileDialog.getOpenFileName`) + label con el nombre del
    archivo (path completo como tooltip, igual criterio que el label de grabación de la barra
    superior). `RecordingAnalysisPanel._read_tidy_csv()` parsea el CSV con el módulo `csv` (sin
    pandas, no es una dependencia del proyecto) agrupando filas por canal y ordenando por
    `tiempo_s` (defensivo: las filas de un mismo canal ya vienen ordenadas por tiempo en el archivo,
    ver `MainWindow._record_batch`, pero un archivo editado a mano podría no estarlo). Para archivos
    muy largos (varios minutos a 8 kSPS) el parseo fila a fila en Python puede tardar unos segundos;
    se muestra cursor de espera (`QApplication.setOverrideCursor(WaitCursor)`, mismo patrón que
    `_auto_assign_ports`) mientras carga.
  - Combo **"Canal"**: se llena con los canales presentes en el archivo cargado (no asume los 8 —
    depende de qué canales estuvieran tildados al grabar). Al cambiar de canal se replotea la serie
    completa y la región de selección se resetea a todo el rango de tiempo de ese canal.
  - El plot (`pg.PlotWidget`) tiene una región arrastrable (`pg.LinearRegionItem`, orientación
    vertical por defecto) superpuesta: se selecciona el rango en el eje X arrastrando sus dos bordes
    con el mouse directamente sobre la señal, sin tipear números. Al cargar un archivo o cambiar de
    canal arranca cubriendo todo el rango. Botón **"Ver todo el rango"** la resetea en cualquier
    momento.
  - Recuadro **"Amplitud del rango seleccionado"**: `_on_region_changed()` (conectado a
    `region.sigRegionChanged`) recalcula en vivo, mientras se arrastra la región, sobre las muestras
    del canal actual dentro de `[xmin, xmax]` (búsqueda binaria con `np.searchsorted` sobre el array
    de tiempo ya ordenado), dos valores en labels grandes junto con el rango seleccionado en
    segundos:
    - **Pico a pico**: `max(mV) - min(mV)`.
    - **RMS**: `sqrt(mean(mV^2))` — fórmula estándar para una señal discreta `x[n]` con `N`
      muestras dentro de la ventana (`RMS = sqrt((1/N) * sum(x[n]^2))`), la misma que la mayoría de
      los papers de EMG usan para cuantificar la actividad muscular en una ventana de tiempo.
      Validado con un caso conocido (onda cuadrada de amplitud A -> RMS = A exacto).
- **"Avanzado"** (todo lo que antes era la UI principal, sin recortar nada): botones "Buscar
  puertos"/"Auto-detectar CDC"/"Conectar seleccionados"/"Desconectar todo" + spinbox de baudrate +
  tabla de asignación manual de puerto COM por CDC; `CdcPanel` de texto crudo para CDC0/CDC1;
  `RldStatusPanel` (grilla completa `LOFF_STATP`/`LOFF_STATN` por electrodo); y **`ConfigPanel`**
  igual que antes (tabla granular por canal — `GAIN`/`CHPOWER`/`MUX`/`LOFF_CH`/`RLD_CH` — más
  controles globales `MODE`/`RLD`/`TESTSIG*`/`LOFF_MODE`/`LOFF_TYPE`/`LOFF_CURRENT`/
  `LOFF_THRESHOLD`/`START`/`STOP` y el botón "Releer configuración actual del ADS1298").

### Filtros de señal (software, scipy — no firmware)

Decisión: los filtros se implementan en Python (`scipy.signal`), no en el ADS1298/RP2040. Motivo:
iteración inmediata sin recompilar/flashear, conserva el código ADC crudo en el buffer de cada
`SingleChannelPlot`, y no requiere tocar el chip (la ganancia, RLD y lead-off sí son comandos reales
de CDC1 que siguen aplicándose en hardware).

- `build_filter_sos(notch_hz, hpf_hz, lpf_hz, fs)`: arma un único array SOS combinando (según qué
  esté activo) un notch (`scipy.signal.iirnotch`, Q=30, convertido a SOS con `tf2sos`) y/o
  Butterworth de orden 4 pasaaltos/pasabajos (`scipy.signal.butter(..., output="sos")`), apilados
  con `np.vstack`. Devuelve `None` si ningún filtro está activo. `fs` = `SingleChannelPlot.
  ASSUMED_SAMPLE_RATE` (8000, igual al cap de firmware).
- `SingleChannelPlot.set_filter(sos)` guarda el SOS vigente; `refresh()` lo aplica con
  `scipy.signal.sosfiltfilt` (**fase cero**, sin retardo) sobre la ventana visible completa en cada
  redibujado (~30 Hz), después de convertir a mV y antes de graficar. Se eligió fase cero en vez de
  streaming con estado (`zi`) para no tener que mantener continuidad de filtro entre batches de
  datos — el costo es recalcular sobre toda la ventana en cada frame, aceptable a esta escala.
  Si la ventana todavía es muy corta para el padlen que necesita `sosfiltfilt` (arrancando el
  streaming), se ignora el error y se muestra sin filtrar hasta acumular más muestras.
- **Alcance confirmado con el usuario**: los filtros se aplican solo a los canales *actualmente
  visibles* (tildados en la sidebar), no a los 8 siempre — más liviano en cómputo.
  `MainWindow._on_filter_changed()` recalcula el SOS y lo propaga con `set_filter()` a
  `self.visible_channels`; `_rebuild_channel_grid()` también llama `set_filter()` sobre los
  canales que pasan a visibles, para que un canal recién tildado ya aparezca filtrado.
- **Cortes disponibles** (`MainWindow._NOTCH_HZ`/`_HPF_HZ`/`_LPF_HZ`): Notch Apagado/50/60 Hz;
  Pasaaltos Apagado/1/2/5/10/20/50 Hz; Pasabajos Apagado/100/125/150/500 Hz.

### Escala de los plots (sidebar "Escala de los plots")

Por defecto ambos ejes de cada `SingleChannelPlot` autoescalan (`pyqtgraph.enableAutoRange`), como
antes. El combo "Modo" agrega una alternativa **Manual**:

- Eje X: spinbox (`x_scale_spin`, 0.01-60) + combo de unidad (`s`/`ms`). Se traduce a segundos
  (`_X_UNIT_TO_SECONDS`) y se muestra como los últimos `x_scale` segundos dentro de la ventana
  bufferizada de `SingleChannelPlot.WINDOW_SECONDS` (4 s) — pedir más de 4 s no agrega historia
  real, solo dejaría espacio en blanco a la izquierda.
- Eje Y: spinbox (`y_scale_spin`, 0.001-10000) + combo de unidad (`mV`/`µV`/`V`, `_Y_UNIT_TO_MV`).
  Se aplica como rango simétrico `[-y_scale, y_scale]` (la señal EMG se asume centrada en 0).
- `MainWindow._on_scale_changed()` calcula ambos valores en las unidades internas (segundos/mV) y
  llama `SingleChannelPlot.set_scale_mode(auto, x_seconds, y_mv)` sobre **los 8 canales** (no solo
  los visibles), para que un canal recién tildado ya respete el modo/rango vigente.
- En modo automático, `set_scale_mode` solo reactiva `enableAutoRange` en ambos ejes; en modo
  manual lo desactiva y llama `setXRange`/`setYRange` una vez (no en cada `refresh()`).

### Grabación a CSV (barra superior, "Iniciar grabación"/"Detener grabación")

Botones **"Iniciar grabación"**/**"Detener grabación"** + label de estado, ubicados en la barra
superior (a la par de "Conectar equipo"/"Desconectar equipo", no en la sidebar — así siempre están
visibles sin competir por espacio con el resto de los controles). El path completo del archivo
queda como *tooltip* del label (el texto en pantalla es corto, solo "Grabando canales ..."/
"Grabación guardada"). Al iniciar:

- Pide carpeta de destino con `QFileDialog.getExistingDirectory` y arma un nombre de archivo
  automático `emg_recording_<YYYYMMDD_HHMMSS>.csv` (`MainWindow._start_recording`).
- Graba **los canales tildados en ese momento** (`self.visible_channels`, copiados a
  `self.recording_channels`) — si después se tildan/destildan canales durante la grabación, la
  grabación sigue solo con el conjunto original.
- Formato CSV **"tidy"/largo** (decidido con el usuario): columnas `tiempo_s, canal, mV`, una fila
  por muestra y canal. Se eligió sobre un formato "ancho" (una columna por canal) porque cada par
  de canales llega por un proceso/CDC independiente (ver "Arquitectura de conexión" más abajo) y no
  siempre en el mismo instante exacto — alinear a la fuerza en una sola fila por timestamp podría
  dejar celdas vacías o desfasajes de milisegundos entre columnas.
- El valor `mV` es **la señal filtrada** (decidido con el usuario, igual a lo que se ve en
  pantalla), no la señal cruda. Como `SingleChannelPlot` filtra con `sosfiltfilt` (fase cero) sobre
  toda la ventana visible — que no tiene sentido aplicar muestra a muestra sobre un archivo
  continuo — `MainWindow._record_batch()` implementa un filtrado **streaming** equivalente con
  `scipy.signal.sosfilt` manteniendo el estado (`zi`, inicializado con `sosfilt_zi(sos) *
  primera_muestra` para no arrancar con un transitorio hacia 0) persistente entre batches
  (`self.recording_filter_state[canal] = (sos, zi)`). El resultado es el mismo filtro (mismos
  cortes) pero no bit-a-bit idéntico al de pantalla (fase cero vs. causal) — trade-off aceptado
  explícitamente por el usuario. Si no hay ningún filtro activo (`current_filter_sos is None`), se
  graba el valor sin filtrar (igual a lo que se ve en pantalla en ese caso).
- El tiempo (`tiempo_s`) se calcula por canal como un contador de muestras acumulado dividido por
  `SingleChannelPlot.ASSUMED_SAMPLE_RATE` (8000), no un timestamp de reloj de pared — asume
  streaming continuo sin pérdida de muestras, misma asunción que ya hace el resto de la GUI.
- La escritura ocurre en `MainWindow.poll_process_queues()` (tag `"frame"`), justo después de
  `append_batch()`, para no depender de la tasa de refresco de pantalla.
- `_stop_recording()` cierra el archivo; `closeEvent()` también detiene una grabación en curso si
  se cierra la ventana sin apretar "Detener grabación".

### Lectura automática de la configuración real del ADS1298 al conectar

Al conectar CDC1 (ya sea vía "Conectar equipo" en "Monitor", "Conectar seleccionados" en
"Avanzado", o justo después de que el hilo de `SerialWorker` confirme la conexión),
`MainWindow.read_current_configuration()` manda automáticamente un `RREG` por cada registro
relevante (`CONFIG_READBACK_REGISTERS`: CONFIG1-3, LOFF, CH1SET..CH8SET, RLD_SENSP/N, LOFF_SENSP/N)
para que el `ConfigPanel` de "Avanzado" refleje cómo estaba configurado el chip realmente, en vez de
asumir los valores por defecto de fábrica. También hay un botón **"Releer configuración actual del
ADS1298"** en esa pestaña para repetir la lectura a mano (por si se cambió algo por `WREG` crudo o
se perdió alguna respuesta).

- Las respuestas ("`REG XX=YY`", formato de `handle_config_command()` en `usb_manager.cpp`) se
  interceptan en `MainWindow.on_data_received()` con la regex `_REG_RESPONSE_RE` y se despachan a
  `ConfigPanel.apply_register(addr, value)`, que decodifica los bits (mismos mapeos que
  `inc/ads1298.h`: `GAIN_CODE_TO_VALUE`, `MUX_MODES`, `TEST_FREQ_CODE_TO_TEXT`,
  `LOFF_CURRENT_CODE_TO_TEXT`, `LOFF_MODE_CODE_TO_TEXT`) y actualiza el combo/checkbox
  correspondiente con `blockSignals(True)` para **no** volver a mandar el comando al ADS1298 (evita
  un loop de reconfiguración al sincronizar el estado). Para `CH1SET..CH8SET` además emite
  `gain_changed(ch, gain)` (aunque venga de una relectura, no de que el usuario haya tocado el
  combo), para que `MainWindow.on_gain_changed()` también sincronice el combo de ganancia
  equivalente en la sidebar de "Monitor".
- Si el usuario cambia algo por `RREG`/`WREG` crudo en vez de por los controles de `ConfigPanel`,
  el GUI también se entera y actualiza los controles (porque escucha cualquier respuesta `REG XX=YY`
  que pase por CDC1, no solo las disparadas por `read_current_configuration()`).

### Conversión código ADC → mV

`code_to_millivolts(codes, gain)` en `emg_multicdc_gui.py` implementa
`Vin(mV) = codigo * VREF(2.4 V) * 1000 / (Gain * (2^23 - 1))`, según la Tabla 13 del datasheet
("Ideal Output Code versus Input Signal") y `ads1298::gainToCode()` del firmware. Notas importantes:

- **VREF se asume fijo en 2.4 V** (referencia interna por defecto, `CONFIG3.VREF_4V=0`) — la API de
  CDC1 no expone ningún comando para cambiar la referencia, así que no hay forma de que el GUI se
  entere si se cambiara por `RREG`/`WREG` crudo.
- Cada `SingleChannelPlot` (un canal físico por widget, ver "Qué hace" más arriba) guarda su propia
  `gain` (default 6, igual que el reset de `CHnSET` y el valor inicial de los combos `GAIN` en la
  sidebar de "Monitor" y en `ConfigPanel` de "Avanzado") y la actualiza vía `set_gain()` cuando
  cambia el combo de ganancia desde cualquiera de esos dos lugares (`_on_sidebar_gain_changed` /
  `on_gain_changed`, sincronizados entre sí). Si el usuario cambia la ganancia por `WREG` directo,
  el GUI no se entera y la conversión a mV quedará mal hasta que se use alguno de los combos (o se
  lea de nuevo la configuración, ver botón "Releer configuración actual").
- La conversión se aplica en `refresh()` (al momento de dibujar), no al ingerir los datos: si la
  ganancia cambió hace poco, las muestras más viejas que quedan en el buffer circular se dibujan con
  la ganancia *actual*, no con la que tenían en el momento de la captura — aproximación aceptable
  para visualización, no para medición de precisión (ver "Próximos pasos").

**Nota histórica — amplitud real de la test signal interna:** la test signal interna del ADS1298
(`TESTSIG ON`) es de amplitud **muy chica**: `CONFIG2.TEST_AMP` da ±1 mV (código 0) o ±2 mV (código
1) — fórmula `±(VREFP - VREFN)/2400 V` del datasheet —, **antes** de dividir por la ganancia en
`code_to_millivolts()` (la ganancia no cambia la amplitud física de la señal de test, solo cuántos
códigos ADC la representan). Es decir, en el plot **siempre** se ve una onda cuadrada de ~1-2 mV de
amplitud, sea cual sea la ganancia configurada — esto es correcto, no un bug. Confirmado
independientemente en el foro de TI (varios usuarios reportan la misma amplitud minúscula y el mismo
síntoma de "spikes que resultan ser datos corruptos de lecturas mal sincronizadas", no la señal
real): [ADS1298: Internal Test Signal](https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/817546/ads1298-internal-test-signal).
El bug real que sí corregimos fue el falso resincronismo de `extract_frames()` (ver "Formato de
trama binaria decodificado" más abajo), que producía esos picos de cientos de mV y, al ser mucho
más grandes que la señal real, dominaban el autoescalado del eje Y y tapaban la onda cuadrada
verdadera.

### Arquitectura de conexión: hilo (texto) vs proceso (binario)

CDC0/CDC1 (texto, interactivo, baja tasa) siguen usando `QThread` + `SerialWorker` como antes — ahí
alcanza con multitasking cooperativo.

CDC2-6 (streaming binario continuo, hasta 8 kSPS × 4 canales de datos) corren cada uno en su
**propio proceso del sistema operativo** (`multiprocessing.Process`), para lograr paralelismo real
entre canales sin competir por el GIL de Python ni con el hilo de la UI:

- `serial_process_worker(name, port, baudrate, frame_mode, out_queue, stop_event)` — función de
  **módulo** (no un método; así `multiprocessing` la puede picklear con el método `spawn` que usa
  Windows por defecto). Abre el puerto, busca el byte de sync `0xAA` con `extract_frames()`, decodifica
  tramas fijas de 7 bytes (`frame_mode="data"`, canales A/B de 24 bits con signo vía
  `decode_data_frame`/`decode_be24_signed`) o de 3 bytes (`frame_mode="rld"`, `decode_rld_frame`), y
  manda los batches ya decodificados por `out_queue` (`multiprocessing.Queue`) cada ~30 ms como
  `("frame", array_a, array_b)` / `("rld", array_a, array_b)`, y los cambios de estado como
  `("status", connected, mensaje)`. Corta el loop y cierra el puerto cuando `stop_event.is_set()`.
- `MainWindow` tiene un único `QTimer` (`poll_process_queues`, ~30 Hz) que drena sin bloquear
  (`queue.get_nowait()`) todas las colas activas y despacha los batches, vía `pair_channels`, al
  `SingleChannelPlot` de cada canal físico (o a `RldStatusPanel` para CDC6). Un segundo `QTimer`
  separado (`refresh_plots`, ~30 FPS) redibuja solo los plots de los canales tildados en la caja
  "Canales" de la sidebar, desacoplando la tasa de llegada de datos de la tasa de refresco de
  pantalla.
- `ThreadConnection` (CDC0/CDC1) y `ProcessConnection` (CDC2-6) son wrappers con la misma interfaz
  mínima (`.port`, `.write_line()`, `.stop()`) guardados en `MainWindow.connections`, para que
  `connect_selected`/`disconnect_all`/`disconnect_one`/`auto_detect_ports` no dupliquen lógica según
  el tipo de conexión.
- Verificación en el Administrador de tareas de Windows: con los 4 CDC de datos conectados deberían
  verse varios procesos `python.exe` adicionales (uno por proceso de `multiprocessing.Process`).

### Formato de trama binaria decodificado (debe coincidir con el firmware)

Ver también CLAUDE.md → "Formato de trama binaria por USB" (lado firmware). CDC2-5: `FRAME_SYNC_BYTE
(0xAA)` + 3 bytes canal A + 3 bytes canal B, MSB primero, complemento a 2. CDC6: `0xAA` +
`LOFF_STATP` + `LOFF_STATN`.

`extract_frames()` valida, antes de aceptar una trama candidata, que el byte donde debería estar el
sync de la *trama siguiente* (`buffer[frame_size]`) también sea `0xAA`; si no lo es, descarta solo
ese byte y reintenta con el próximo candidato (en vez de aceptar la trama a ciegas). Esto corrige un
bug real observado en hardware con la test signal interna: como su amplitud real es de ~1-2 mV (ver
más abajo), la mayoría de los bytes de canal son casi cero, y un byte de dato real que valga `0xAA`
por casualidad se confundía con el sync, produciendo muestras absurdas (picos de cientos de mV) que
además dominaban el autoescalado del eje Y y tapaban la señal real (ver "Nota histórica" abajo).

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
- `MainWindow._auto_assign_ports()` — extrae el núcleo del escaneo (elegir placa si hay más de una,
  excluir puertos ya abiertos por conexiones activas del propio programa, rellenar
  `self.port_combos`) en un método reutilizable que devuelve un resumen (`serial_number`,
  `assigned`, `missing`, `skipped`) o `None` si no hay placa o se canceló la selección. Mientras
  escanea (abre cada puerto ~0.35s) deshabilita el botón "Auto-detectar CDC" y muestra progreso.
  Lo usan **dos** entry points sin duplicar lógica:
  - `MainWindow.auto_detect_ports()` (botón "Auto-detectar CDC" en "Avanzado"): solo rellena los
    combos y muestra un `QMessageBox` con el resumen detallado, sin conectar nada (comportamiento
    histórico, sin cambios).
  - `MainWindow.connect_equipment()` (botón **"Conectar equipo"**, en la barra superior, visible en
    las 3 pestañas): además llama a `connect_selected()` para arrancar las conexiones, y solo
    avisa con un `QMessageBox` si faltó identificar alguna interfaz — pensado para que el usuario
    clínico no tenga que saber qué es un CDC ni tocar la pestaña "Avanzado" en el uso normal.

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

1. **Validar en hardware la nueva arquitectura de streaming**, en el orden acordado (ahora se puede
   hacer directamente con `emg_multicdc_gui.py`, que ya decodifica y grafica los frames binarios):
   1. Test signals internas: desde la pestaña "Señales de prueba" (`TestSignalPanel`), activar
      `TESTSIG ON` y tildar los canales a probar, confirmar la forma de onda esperada (cuadrada,
      con la amplitud/frecuencia configurada) en los plots de la pestaña "Monitor".
   2. Señal continua/DC externa conocida por cada par `INxP`/`INxN`, verificar el valor digital
      esperado según `GAIN` y la referencia de voltaje.
   3. Generador de señales (AC realista), confirmar la cadena completa SPI→USB en condiciones
      dinámicas a 8 kSPS sostenido, sin pérdida de muestras ni desincronización del marcador `0xAA`
      (si se pierde sync se nota como un salto/glitch puntual en el plot).
   - **Nota importante:** esto todavía no se probó contra la placa física con el firmware nuevo
     flasheado — falta esa validación en hardware real.
   - **Pendiente de re-flashear y re-probar (sesión 2026-08-21):** se corrigieron dos bugs reales de
     firmware que impedían usar RLD y lead-off (`CONFIG4.PD_LOFF_COMP` nunca habilitado, y `RLD
     ON`/`OFF` sin seleccionar canales en `RLD_SENSP`/`N`; ver "Presupuesto de throughput USB" y la
     sección "RLD / Lead-off" de la GUI). Falta compilar, flashear y validar en la placa real que
     ahora sí se detecten desconexiones de electrodo y que el RLD reduzca ruido de modo común.
2. Evaluar si conviene throttlear el envío de CDC6 (RLD/lead-off) a una fracción de la tasa de
   muestreo, ya que ese estado cambia mucho más lento que la señal EMG y hoy se manda a la misma
   cadencia que los datos.
3. ~~Conversión a mV en `SingleChannelPlot`~~ — hecho (`code_to_millivolts()`, ver sección
   "Conversión código ADC → mV"). Sigue pendiente: si se agrega un comando para cambiar `VREF_4V` a
   la API de CDC1, hay que actualizar `ADS1298_VREF_VOLTS` o trackearlo igual que la ganancia.
4. Mover `identify_cdc_index`/auto-detección a un hilo (`QThread`) en vez de bloquear la UI con
   `processEvents()`, sobre todo si se agregan más CDC o el escaneo se vuelve más lento.
5. Considerar mover de polling de DRDY a IRQ en el core 1 si el consumo de CPU/energía del polling
   cerrado resulta un problema (hoy es polling puro por simplicidad, ver `acquisition_core1_entry()`).
6. `extract_frames()` en el GUI se resincroniza buscando el próximo `0xAA`, pero no tiene ninguna
   verificación adicional (CRC, contador de secuencia); un `0xAA` que aparezca dentro de un valor de
   canal real puede producir una muestra corrupta puntual — aceptable para una herramienta de
   visualización, pero a tener en cuenta si se necesita integridad de datos garantizada.
