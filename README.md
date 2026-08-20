# EMG — Raspberry Pi Pico + ADS1298

Sistema de adquisición de señales EMG (electromiografía) multicanal: firmware para una Raspberry Pi
Pico (RP2040) que controla un ADC frontal ADS1298 por SPI y transmite los datos a una PC por USB, más
una GUI de escritorio en Python (`emg_multicdc_gui.py`) para conectar el equipo, configurarlo y
visualizar las señales en tiempo real.

Para el detalle técnico completo (protocolo USB, pines, arquitectura de streaming, API de
configuración, etc.) ver [`CLAUDE.md`](CLAUDE.md). Esta guía cubre solo lo necesario para dejar el
proyecto andando desde cero.

## 1. Firmware (Raspberry Pi Pico)

### 1.1. Instalar VS Code + extensión de Raspberry Pi Pico

1. Instalar [Visual Studio Code](https://code.visualstudio.com/).
2. Abrir la carpeta del proyecto en VS Code. Al abrirla, VS Code va a sugerir instalar las
   extensiones recomendadas del proyecto (definidas en `.vscode/extensions.json`); aceptar la
   instalación, o instalarlas a mano desde la pestaña de extensiones:
   - **Raspberry Pi Pico** (`raspberry-pi.raspberry-pi-pico`) — la extensión oficial, imprescindible.
   - `ms-vscode.cpptools` / `ms-vscode.cpptools-extension-pack` (soporte de C/C++).
   - `marus25.cortex-debug` (debug sobre el RP2040).
   - `ms-vscode.vscode-serial-monitor` (útil para ver los CDC de texto sin la GUI de Python).

**No hace falta descargar el SDK de Pico manualmente.** La extensión "Raspberry Pi Pico" lo
gestiona sola: la primera vez que la usás (o al abrir este proyecto, que ya trae `CMakeLists.txt`
apuntando a `sdkVersion 2.2.0` / `toolchainVersion 14_2_Rel1`) te va a ofrecer descargar e instalar
automáticamente, bajo `~/.pico-sdk/`:
- Pico SDK 2.2.0
- Toolchain ARM GCC 14_2_Rel1
- CMake y Ninja
- `picotool`

Seguí el asistente de la extensión (ícono de Raspberry Pi en la barra lateral) y esperá a que
termine de descargar todo antes de compilar.

### 1.2. Compilar el firmware

Con la extensión instalada y el SDK descargado:

1. Abrí la paleta de comandos (`Ctrl+Shift+P`) y ejecutá **"Raspberry Pi Pico: Compile Project"**
   (o usá el botón correspondiente en la barra de estado inferior de VS Code, si la extensión lo
   muestra).
2. Esto corre CMake + Ninja y genera, entre otros archivos, `build/EMG.uf2` — el binario que se
   flashea a la Pico.

Si preferís la terminal integrada de VS Code (que ya trae `PICO_SDK_PATH`/`PATH` configurados por
`.vscode/settings.json`), también podés compilar a mano:

```powershell
cmake -G Ninja -B build
cmake --build build
```

### 1.3. Flashear el firmware (modo BOOTSEL)

1. Si la Pico ya estaba conectada, desconectala del cable USB.
2. Mantené presionado el botón **BOOTSEL** de la Pico (el único botón físico en la placa).
3. Sin soltar BOOTSEL, conectá el cable USB a la PC.
4. Soltá BOOTSEL una vez conectado. La Pico va a aparecer en el explorador de archivos de Windows
   como una unidad de almacenamiento masivo llamada **RPI-RP2**.
5. Copiá (arrastrá y soltá) el archivo `build/EMG.uf2` a esa unidad **RPI-RP2**.
6. La Pico se reinicia sola automáticamente apenas termina de copiarse el archivo, y arranca el
   firmware nuevo. La unidad RPI-RP2 desaparece; en su lugar, Windows debería reconocer un
   dispositivo compuesto con **7 interfaces USB CDC** (revisar en el Administrador de dispositivos,
   bajo "Puertos (COM y LPT)").

> La extensión de Pico también ofrece un comando **"Run Project (USB)"** que hace el paso de copiar
> el `.uf2` automáticamente si detecta la placa en modo BOOTSEL — es equivalente al copiado manual
> de arriba.

Repetir el ciclo BOOTSEL + copiar `.uf2` cada vez que se recompile una versión nueva del firmware.

## 2. Software de PC: `emg_multicdc_gui.py`

### 2.1. Instalar Python y la extensión de Python en VS Code

1. Instalar [Python](https://www.python.org/downloads/) (3.10 o superior recomendado), asegurándose
   de tildar **"Add python.exe to PATH"** durante la instalación en Windows.
2. En VS Code, instalar las extensiones de Python (también sugeridas automáticamente al abrir la
   carpeta del proyecto, o instalables a mano):
   - `ms-python.python`
   - `ms-python.vscode-pylance`

### 2.2. Instalar las dependencias del proyecto

Desde una terminal (o la terminal integrada de VS Code) parada en la carpeta del proyecto:

```powershell
pip install pyqt6 pyserial pyqtgraph numpy scipy
```

### 2.3. Conectar la Pico y correr la GUI

1. Conectar la Pico (ya con el firmware flasheado) a la PC por USB normalmente (sin BOOTSEL).
2. Correr la GUI:

   ```powershell
   python emg_multicdc_gui.py
   ```

3. En la pestaña **"Monitor"**, tocar el botón **"Conectar equipo"**: la GUI detecta y conecta
   automáticamente las 7 interfaces del firmware, sin necesidad de elegir puertos COM a mano.
4. Desde la sidebar (izquierda) se eligen los canales a visualizar, la ganancia por canal, la
   escala de los plots (automática o manual, en s/ms y mV/µV/V), los filtros
   (notch/pasaaltos/pasabajos) y el RLD/lead-off. Los plots de los canales tildados ocupan el resto
   de la pantalla sin scrollbar (eje X en segundos): con 1-3 canales se apilan en una sola columna;
   con 4-8 se reparten en dos columnas lado a lado, priorizando siempre ver más escala horizontal
   (tiempo) que vertical.
5. En la barra superior, junto a "Conectar equipo"/"Desconectar equipo", el botón
   **"Iniciar grabación"** guarda a un archivo CSV los canales que estén tildados en ese momento
   (pide elegir la carpeta de destino); **"Detener grabación"** cierra el archivo.
6. La pestaña **"Señales de prueba"** permite activar la test signal interna del ADS1298 para
   validar que la cadena SPI→USB funciona antes de medir una señal real.
7. La pestaña **"Avanzado"** expone el detalle técnico (asignación manual de puertos COM por CDC,
   texto crudo de configuración, lectura/escritura directa de registros, estado completo de
   lead-off/RLD por electrodo) — pensada para debug, no para el uso clínico normal.

## 3. Más información

- [`CLAUDE.md`](CLAUDE.md): documentación técnica completa del protocolo, la arquitectura de
  streaming multicore, el mapeo de pines/CDC y el detalle interno de la GUI.
- `docs/ads1298.pdf`: datasheet del ADS1298.
- `docs/plan_streaming_multicore.md` y `docs/plan_gui_streaming_config.md`: decisiones de diseño
  históricas del streaming y de la GUI.
