# Plan: visualización binaria en tiempo real + panel de configuración en `emg_multicdc_gui.py`

> Nota: este plan ya fue implementado (ver `CLAUDE.md`, secciones "Herramienta PC:
> `emg_multicdc_gui.py`", para el estado final tal como quedó en el código). Se conserva acá como
> referencia histórica de las decisiones de diseño y su justificación.

## Contexto

El firmware (ya implementado, ver `CLAUDE.md` y `docs/plan_streaming_multicore.md`) hace streaming
binario continuo por CDC2-5 (2 canales EMG de 24 bits cada uno, trama de 7 bytes) y CDC6 (lead-off
status, trama de 3 bytes), y expone una API de configuración completa por comandos de texto en CDC1
(GAIN, CHPOWER, MODE, RLD, RLD_CH, TESTSIG/TESTSIG_AMP/TESTSIG_FREQ, MUX, LOFF_CH/LOFF_MODE/
LOFF_TYPE/LOFF_CURRENT/LOFF_THRESHOLD, START/STOP). El GUI (`emg_multicdc_gui.py`) solo sabía leer
líneas de texto terminadas en `\n` — no podía mostrar los datos binarios ni configurar el ADS1298
sin escribir los comandos a mano en el panel de CDC1. Este plan agregó:
1. Decodificación y visualización en tiempo real de los frames binarios (crítico para poder ver la
   test signal interna del ADS1298 durante la validación en 3 etapas documentada en `CLAUDE.md`).
2. Un panel de configuración con controles (combos, checkboxes, botones) que arman y envían los
   comandos de CDC1, en vez de tipearlos.

Todo el cambio quedó en el único archivo existente `emg_multicdc_gui.py` (sin archivos nuevos,
siguiendo la convención del proyecto de un solo script).

## Diseño

### 1. Nueva dependencia: `pyqtgraph` (+ `numpy`)

Para graficar en tiempo real 8 canales EMG con refrescos frecuentes, se agregó `pyqtgraph` (estándar
de facto para plots en tiempo real con PyQt, con downsampling automático integrado —
`setDownsampling(mode="peak")`, `setClipToView(True)`— necesario para no colapsar la UI a 8 kSPS).
Requisitos actualizados: `pip install pyqt6 pyserial pyqtgraph numpy`.

### 2. Lectura/decodificación de los CDC de datos: proceso independiente por canal (multicore real)

CDC0 y CDC1 (texto, baja tasa, interactivo) siguen usando el `SerialWorker` + `QThread` de siempre —
ahí no hacía falta más que multitasking cooperativo.

Para CDC2-5 (datos) y CDC6 (RLD), cada canal corre en su **propio proceso**
(`multiprocessing.Process`), logrando paralelismo real entre canales y liberando al proceso de la
GUI de tener que decodificar bytes a 8 kSPS × 4 canales:

- `serial_process_worker(name, port, baudrate, frame_mode, out_queue, stop_event)`: función de
  **nivel de módulo** (necesario para que `multiprocessing` la pueda picklear con el método `spawn`
  de Windows). Abre el puerto serie, busca el byte de sync `0xAA` con `extract_frames()` y
  decodifica tramas fijas (7 bytes `frame_mode="data"`, 3 bytes `frame_mode="rld"`), resincronizando
  sola si el sync está corrido. Acumula muestras en arrays `numpy` y las manda por `out_queue`
  (`multiprocessing.Queue`) cada ~30 ms como `("frame"|"rld", array_a, array_b)`, y los cambios de
  estado como `("status", connected, mensaje)`. Corta el loop y cierra el puerto cuando
  `stop_event.is_set()`.
- `MainWindow` tiene un único `QTimer` (`poll_process_queues`, ~30 Hz) que drena sin bloquear
  (`queue.get_nowait()`) todas las colas activas y despacha los batches a `DataChannelPlot`/
  `RldStatusPanel`. Un segundo `QTimer` (`refresh_plots`, ~30 FPS) redibuja los plots, desacoplado
  de la tasa de llegada de datos.
- `ThreadConnection` (CDC0/CDC1) y `ProcessConnection` (CDC2-6) son wrappers con una interfaz mínima
  común (`.port`, `.write_line()`, `.stop()`) guardados en `MainWindow.connections`, para que
  `connect_selected`/`disconnect_all`/`disconnect_one`/`auto_detect_ports` no dupliquen lógica según
  el tipo de conexión.
- El script ya tenía el guard `if __name__ == "__main__":` que requiere `multiprocessing` en
  Windows (método `spawn`).

### 3. Paneles de visualización (reemplazan `CdcPanel` solo para CDC2-6)

- **`DataChannelPlot(QGroupBox)`** (para CDC2-5): un `pyqtgraph.PlotWidget` con 2 curvas (canal A y
  B), ventana deslizante de ~4 segundos (buffer circular `numpy` de tamaño
  `WINDOW_SECONDS * ASSUMED_SAMPLE_RATE`), eje Y en "código ADC (24 bits)" (sin conversión a mV —
  requeriría trackear la ganancia configurada por canal, quedó anotado como pendiente futuro), más
  un label de estado de conexión.
- **`RldStatusPanel(QGroupBox)`** (para CDC6): 8 indicadores para `LOFF_STATP` + 8 para
  `LOFF_STATN` (`QLabel` coloreado verde="OK"/rojo="OFF" según el bit), actualizados en tiempo real.
- CDC0 (debug) y CDC1 (config) siguen usando el `CdcPanel` de texto de siempre, sin cambios.

### 4. Panel de configuración (`ConfigPanel`, pestaña nueva)

Se envolvió la ventana en un `QTabWidget` con dos pestañas: "Monitor" (grid de 7 paneles) y
"Configuración". La pestaña "Configuración" arma y manda los comandos usando el mismo mecanismo ya
existente (`MainWindow.send_to_cdc`, requiere que la conexión de CDC1 esté activa):

- **Tabla por canal (1-8)**: combo de ganancia (`GAIN`), checkbox de encendido (`CHPOWER`), combo de
  mux (`MUX`), checkboxes de lead-off P/N (`LOFF_CH`), checkboxes de RLD P/N (`RLD_CH`).
- **Controles globales**: combo HR/LP (`MODE`), combo RLD ON/OFF (`RLD`), combo test signal
  (`TESTSIG`) + amplitud (`TESTSIG_AMP`) + frecuencia (`TESTSIG_FREQ`), combos de
  `LOFF_MODE`/`LOFF_TYPE`/`LOFF_CURRENT`/`LOFF_THRESHOLD`, botones START/STOP.
- Cada control envía su comando inmediatamente al cambiar (los valores iniciales se fijan ANTES de
  conectar la señal `currentTextChanged`/`toggled`, para no disparar comandos espurios al construir
  la UI). Las respuestas (`OK`/`ERROR: ...`) se ven en el panel de texto de CDC1 en "Monitor".
- `RREG`/`WREG` crudos quedan como estaban, tipeables en el panel de texto de CDC1.

## Archivos tocados

Solo `emg_multicdc_gui.py` (reescritura completa dado el alcance del cambio):
- Funciones de decodificación de frames: `decode_be24_signed`, `extract_frames`,
  `decode_data_frame`, `decode_rld_frame`, y la función de proceso `serial_process_worker`.
- `SerialWorker`/`CdcPanel` (texto, CDC0/CDC1) intactos.
- Nuevas clases `DataChannelPlot`, `RldStatusPanel`, `ConfigPanel`.
- Nuevas clases `ThreadConnection`/`ProcessConnection` (wrappers de conexión unificados).
- `MainWindow`: `QTabWidget`, dos `QTimer` (drenado de colas + refresco de plots),
  `connect_selected` elige hilo o proceso según el CDC, `poll_process_queues`/`refresh_plots` nuevos.
- Docstring actualizado con la nueva dependencia y la arquitectura de un proceso por CDC de datos.

## Verificación realizada

1. `py_compile` sobre el archivo completo: sin errores de sintaxis.
2. Instalación de `pyqtgraph`/`numpy` (no estaban presentes) y verificación de import del módulo
   completo sin excepciones, incluyendo pruebas unitarias rápidas de `extract_frames` y
   `decode_be24_signed` con datos sintéticos (resultados correctos: sign-extension de 24 bits,
   resincronización descartando bytes de basura antes del primer `0xAA`).
3. Smoke test end-to-end: se instanció `MainWindow` dentro de un `QApplication` real, se mostró la
   ventana 2 segundos y se cerró sola sin excepciones (confirma construcción de todos los widgets
   nuevos, timers y señales).
4. **Pendiente** (requiere el firmware nuevo flasheado en la placa física, ver `CLAUDE.md` →
   "Próximos pasos"): la validación en 3 etapas (test signal interna, señal DC externa, generador de
   funciones) contra hardware real, y confirmar en el Administrador de tareas que aparecen procesos
   `python.exe` adicionales por cada CDC de datos conectado.
