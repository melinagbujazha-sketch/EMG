
"""
EMG Multichannel Serial Monitor - PyQt6

Requisitos:
    pip install pyqt6 pyserial pyqtgraph numpy scipy

Uso:
    python emg_multicdc_gui.py

Auto-detección de puertos COM:
    El firmware enumera un único dispositivo USB compuesto (VID 0xCAFE) con 7
    interfaces CDC. En Windows, pyserial no siempre expone el número de
    interfaz USB (MI_XX) ni un "location" fiable para deducir a qué CDC
    pertenece cada puerto COM, así que la detección no se basa en eso.

    En su lugar, el firmware envía un banner de bienvenida por cada CDC
    apenas detecta que una terminal se conectó (transición de DTR a 1),
    incluyendo el texto "(CDC0)".."(CDC6)" (ver send_interface_banner en
    src/usb_manager.cpp). El botón "Auto-detectar CDC" abre brevemente cada
    puerto COM candidato (mismo VID y número de serie), fuerza esa
    transición de DTR y lee el banner para identificar con certeza a qué
    CDC pertenece cada puerto.

Streaming binario (CDC2-6) y arquitectura multicore:
    El firmware manda datos binarios continuos por CDC2-5 (2 canales EMG de
    24 bits cada uno, trama de 7 bytes: 1 byte de sync 0xAA + 3+3 bytes) y
    por CDC6 (lead-off status, trama de 3 bytes: sync + LOFF_STATP +
    LOFF_STATN). Ver CLAUDE.md para el detalle completo del protocolo.

    CDC0 (debug) y CDC1 (config) siguen siendo texto por línea y usan
    QThread + SerialWorker como antes. CDC2-6, en cambio, corren cada uno en
    su propio proceso del sistema operativo (multiprocessing.Process,
    ver serial_process_worker) para lograr paralelismo real entre canales
    sin competir por el GIL de Python ni con la UI. Los procesos mandan los
    datos ya decodificados a la ventana principal por multiprocessing.Queue,
    que un QTimer drena periódicamente sin bloquear el hilo de la UI.

Interfaz clínica (3 pestañas):
    Barra superior (visible en las 3 pestañas): "Conectar equipo"/
    "Desconectar equipo" (sin exponer CDC ni puertos COM al usuario
    clínico), "Iniciar grabación"/"Detener grabación" a CSV, y pantalla
    completa.

    "Monitor" es la pestaña principal: una sidebar a la izquierda (~20% del
    ancho, selección de canales visibles + ganancia por canal, escala de
    los plots, filtros de software y toggles simples de RLD/lead-off) y a
    la derecha el área de plots de los canales tildados, ocupando el 80%
    restante del espacio **sin scrollbar** (siempre se reparten para entrar
    en el espacio disponible, ver MainWindow._rebuild_channel_grid): con
    1-3 canales tildados se apilan en una sola columna; con 4-8 se separan
    en dos columnas lado a lado (ceil(n/2) a la izquierda, floor(n/2) a la
    derecha), cada una apilada por su cuenta. Siempre se prioriza ver más
    escala horizontal (tiempo) que vertical.

    "Señales de prueba" deja activar la test signal interna del ADS1298
    (TESTSIG/AMP/FREQ) y enrutarla a los canales elegidos (MUX TEST/NORMAL
    simplificado), para la validación en 3 etapas descrita en CLAUDE.md.

    "Análisis" (RecordingAnalysisPanel) carga offline un CSV grabado por
    "Iniciar grabación" (no requiere el equipo conectado), lo grafica por
    canal y muestra la amplitud pico a pico (max mV - min mV) del rango de
    tiempo seleccionado arrastrando una región sobre el propio plot.

    "Avanzado" concentra todo lo que antes era la UI de debug, sin recortar
    funcionalidad: mapeo manual de puertos por CDC, paneles de texto crudo de
    CDC0/CDC1 (incluyendo RREG/WREG a mano), el estado completo de
    lead-off/RLD por electrodo, y la tabla granular de configuración por
    canal (ganancia, encendido, mux, lead-off y RLD por P/N).

Filtros de señal (software, no firmware):
    Notch (50/60 Hz), pasaaltos (20/50 Hz) y pasabajos (150/500 Hz) se
    implementan en Python con scipy.signal (Butterworth + notch IIR,
    combinados en un único array SOS por build_filter_sos()) y se aplican en
    SingleChannelPlot.refresh() con sosfiltfilt (fase cero) sobre la ventana
    visible completa en cada redibujado, solo a los canales actualmente
    visibles. No requieren tocar el firmware ni el ADS1298; la ganancia, RLD
    y lead-off sí son comandos reales de CDC1 y siguen aplicándose en el chip.

Escala de los plots:
    Cada SingleChannelPlot autoescala ambos ejes por defecto (pyqtgraph
    autorange). La sidebar "Escala de los plots" permite pasar a modo
    manual: un valor + unidad para el eje X (ventana de tiempo mostrada, en
    s o ms) y para el eje Y (rango simétrico ±valor, en mV/µV/V). Ver
    MainWindow._on_scale_changed / SingleChannelPlot.set_scale_mode.

Grabación a CSV:
    Los botones "Iniciar grabación"/"Detener grabación" de la barra
    superior graban a un CSV "tidy" (columnas tiempo_s, canal, mV) los
    canales tildados al momento de tocar "Iniciar grabación" (elige
    carpeta de destino, nombre de archivo automático con timestamp). El
    valor mV es la señal *filtrada* (mismo notch/pasaaltos/pasabajos de la
    sidebar), aplicada muestra a muestra con estado persistente
    (scipy.signal.sosfilt + sosfilt_zi) para que coincida con lo que se ve
    en pantalla — a diferencia del sosfiltfilt de fase cero que usan los
    plots, que solo tiene sentido sobre una ventana ya completa. Ver
    MainWindow._start_recording/_stop_recording/_record_batch.
"""

import csv
import math
import multiprocessing as mp
import os
import queue
import re
import sys
import time
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from scipy.signal import butter, iirnotch, tf2sos, sosfilt, sosfilt_zi, sosfiltfilt

from PyQt6.QtCore import QObject, QThread, QTimer, pyqtSignal, pyqtSlot, Qt
from PyQt6.QtGui import QFont, QKeySequence, QShortcut
from PyQt6.QtWidgets import (
    QApplication, QWidget, QMainWindow, QVBoxLayout, QHBoxLayout,
    QPushButton, QTextEdit, QLabel, QComboBox, QCheckBox, QLineEdit,
    QGroupBox, QGridLayout, QMessageBox, QSpinBox, QDoubleSpinBox,
    QInputDialog, QTabWidget, QScrollArea, QSplitter, QFileDialog,
)


CDC_NAMES = [
    "DEBUG CDC0",
    "CONFIG CDC1",
    "DATA CH1-CH2 CDC2",
    "DATA CH3-CH4 CDC3",
    "DATA CH5-CH6 CDC4",
    "DATA CH7-CH8 CDC5",
    "RLD CDC6",
]

# VID del firmware EMG (ver src/usb_descriptors.c: CDC_EXAMPLE_VID = 0xCafe).
EMG_VENDOR_ID = "CAFE"

_HWID_SER_RE = re.compile(r"SER=([^&\s]+)")
# pyserial construye el hwid como "USB VID:PID=CAFE:4007 SER=... LOCATION=...".
_HWID_VID_PID_RE = re.compile(r"VID:PID=([0-9A-Fa-f]{4}):([0-9A-Fa-f]{4})")
# Formato crudo de Windows (hardware ID de SetupAPI), por si se necesita leer
# directamente "VID_CAFE&PID_4007&MI_00" en vez del hwid ya formateado.
_HWID_VID_PID_RAW_RE = re.compile(r"VID_([0-9A-Fa-f]{4})&PID_([0-9A-Fa-f]{4})")

# El firmware manda el nombre de la interfaz entre paréntesis, p. ej.
# "DATA STREAM CH1-CH2 (CDC2)" -> ver interface_name() en usb_manager.cpp.
_BANNER_CDC_RE = re.compile(r"\(CDC(\d)\)")

# Respuesta de un RREG: "REG 01=82" (ver handle_config_command en usb_manager.cpp).
_REG_RESPONSE_RE = re.compile(r"^REG\s+([0-9A-Fa-f]{2})=([0-9A-Fa-f]{2})$")

# Registros que se leen automáticamente al conectar CDC1, para reflejar en la
# pestaña de Configuración cómo está configurado realmente el ADS1298 (en vez
# de asumir los valores por defecto). CONFIG1..LOFF, CH1SET..CH8SET,
# RLD_SENSP/N, LOFF_SENSP/N (ver inc/ads1298.h para las direcciones).
CONFIG_READBACK_REGISTERS = [
    0x01, 0x02, 0x03, 0x04,
    0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
    0x0D, 0x0E, 0x0F, 0x10,
]


def parse_hwid(hwid: str):
    """Extrae VID, PID y número de serie de un hwid de pyserial.

    Formato típico en Windows:
        USB VID:PID=CAFE:4007 SER=334E3932361CACA5 LOCATION=1-1:x.4
    """
    vid_pid_match = _HWID_VID_PID_RE.search(hwid) or _HWID_VID_PID_RAW_RE.search(hwid)
    ser_match = _HWID_SER_RE.search(hwid)

    vid = vid_pid_match.group(1) if vid_pid_match else None
    pid = vid_pid_match.group(2) if vid_pid_match else None
    serial_number = ser_match.group(1) if ser_match else None

    return vid, pid, serial_number


def group_ports_by_serial():
    """Agrupa los puertos COM del dispositivo EMG (VID 0xCAFE) por número de
    serie del dispositivo físico.

    Devuelve un dict {numero_de_serie: [lista_de_puertos_com]}.
    """
    boards = {}

    for p in serial.tools.list_ports.comports():
        vid, _pid, serial_number = parse_hwid(p.hwid or "")

        if vid is None or vid.upper() != EMG_VENDOR_ID:
            continue

        key = serial_number or "desconocido"
        boards.setdefault(key, []).append(p.device)

    return boards


def identify_cdc_index(device: str, baudrate: int = 115200, timeout: float = 1.0):
    """Abre brevemente un puerto COM y determina a qué CDC del firmware
    pertenece, leyendo el banner de bienvenida que se envía al forzar una
    transición de DTR (0 -> 1).

    Devuelve el índice de CDC (0-6) o None si no se pudo identificar
    (puerto ocupado, sin respuesta, formato inesperado, etc.).
    """
    try:
        with serial.Serial(device, baudrate, timeout=timeout) as ser:
            ser.dtr = False
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.dtr = True  # dispara tud_cdc_line_state_cb(dtr=true) -> banner
            time.sleep(0.3)
            data = ser.read(512)
    except Exception:
        return None

    text = data.decode(errors="ignore")
    match = _BANNER_CDC_RE.search(text)
    if not match:
        return None

    return int(match.group(1))


def find_emg_boards(progress_cb=None):
    """Detecta los puertos COM del dispositivo EMG y determina a qué CDC
    pertenece cada uno.

    progress_cb, si se indica, se llama con (device, cdc_index_or_None) a
    medida que se identifica cada puerto (útil para reportar avance en la UI).

    Devuelve un dict {numero_de_serie: {cdc_index: puerto_com}}.
    """
    grouped = group_ports_by_serial()
    boards = {}

    for serial_number, devices in grouped.items():
        mapping = {}

        for device in devices:
            cdc_index = identify_cdc_index(device)

            if progress_cb:
                progress_cb(device, cdc_index)

            if cdc_index is not None and cdc_index < len(CDC_NAMES):
                mapping[cdc_index] = device

        if mapping:
            boards[serial_number] = mapping

    return boards


@dataclass
class PortInfo:
    device: str
    description: str


# ==================================================
# Decodificación de frames binarios (CDC2-5 datos, CDC6 RLD)
# ==================================================
# Ver CLAUDE.md, sección "Formato de trama binaria por USB", para el detalle
# del protocolo tal como lo arma el firmware (src/usb_manager.cpp).

FRAME_SYNC_BYTE = 0xAA
DATA_FRAME_SIZE = 7  # sync + 3 bytes canal A + 3 bytes canal B
RLD_FRAME_SIZE = 3   # sync + LOFF_STATP + LOFF_STATN

# Conversión código ADC -> mV (ver ads1298.h / datasheet, Table 13 "Ideal
# Output Code versus Input Signal"): Vin = codigo * VREF / (Gain * (2^23-1)).
# VREF asume la referencia interna por defecto del ADS1298 (2.4 V,
# CONFIG3.VREF_4V=0); la API de CDC1 no expone ningún comando para cambiarla.
ADS1298_VREF_VOLTS = 2.4
ADS1298_FULL_SCALE_CODES = 2 ** 23 - 1
DEFAULT_GAIN = 6  # CHnSET reset = 0x00 -> GAIN[2:0] = 000 -> ganancia 6


def code_to_millivolts(codes: np.ndarray, gain: int) -> np.ndarray:
    """Convierte códigos ADC crudos (int) a milivoltios según la ganancia
    configurada para ese canal."""
    scale = (ADS1298_VREF_VOLTS * 1000.0) / (gain * ADS1298_FULL_SCALE_CODES)
    return codes.astype(np.float64) * scale


def build_filter_sos(notch_hz, hpf_hz, lpf_hz, fs):
    """Arma un único array SOS (second-order sections) combinando los
    filtros activos (ver sidebar "Filtros" en MainWindow). Cada argumento de
    frecuencia puede ser None para desactivar esa etapa. Devuelve None si
    ningún filtro está activo (sin filtrar).

    Notch: scipy.signal.iirnotch (Q=30). Pasaaltos/pasabajos: Butterworth de
    orden 4 (scipy.signal.butter). Se aplican en cascada en el orden
    notch -> HPF -> LPF vía sosfiltfilt (ver SingleChannelPlot.refresh)."""
    sections = []

    if notch_hz:
        b, a = iirnotch(notch_hz, Q=30, fs=fs)
        sections.append(tf2sos(b, a))

    if hpf_hz:
        sections.append(butter(4, hpf_hz, btype="high", fs=fs, output="sos"))

    if lpf_hz:
        sections.append(butter(4, lpf_hz, btype="low", fs=fs, output="sos"))

    if not sections:
        return None

    return np.vstack(sections)


def decode_be24_signed(b0: int, b1: int, b2: int) -> int:
    """Decodifica 3 bytes MSB-first en complemento a 2 (24 bits) a un int con signo."""
    value = (b0 << 16) | (b1 << 8) | b2
    if value & 0x800000:
        value -= 0x1000000
    return value


def extract_frames(buffer: bytearray, frame_size: int):
    """Busca el marcador de sync (0xAA) en buffer y extrae tramas completas
    de tamaño fijo frame_size, mutando buffer in-place.

    Antes de aceptar una trama candidata, valida que el byte donde debería
    estar el sync de la *siguiente* trama (una trama más adelante) también
    sea 0xAA. Esto evita falsos sincronismos cuando un byte de dato real
    vale 0xAA por casualidad (con la señal cerca de cero, como la test
    signal interna del ADS1298, esto pasa con más frecuencia de la
    esperada) — sin esta verificación, un solo byte 0xAA "de casualidad"
    producía una muestra corrupta con valores absurdos (visto en hardware:
    picos de ~-1000 mV con la test signal, que en realidad es de ~1-2 mV de
    amplitud). Si la validación falla, se descarta solo ese byte y se
    reintenta con el próximo candidato."""
    frames = []

    while True:
        sync_idx = buffer.find(FRAME_SYNC_BYTE)
        if sync_idx < 0:
            buffer.clear()
            break

        if sync_idx > 0:
            del buffer[:sync_idx]

        # Hace falta ver el sync de la trama siguiente para validar esta;
        # si todavía no llegaron suficientes bytes, esperamos al próximo
        # batch de lectura del puerto serie.
        if len(buffer) < 2 * frame_size:
            break

        if buffer[frame_size] != FRAME_SYNC_BYTE:
            del buffer[0]
            continue

        frames.append(bytes(buffer[:frame_size]))
        del buffer[:frame_size]

    return frames


def decode_data_frame(frame_bytes: bytes):
    ch_a = decode_be24_signed(frame_bytes[1], frame_bytes[2], frame_bytes[3])
    ch_b = decode_be24_signed(frame_bytes[4], frame_bytes[5], frame_bytes[6])
    return ch_a, ch_b


def decode_rld_frame(frame_bytes: bytes):
    return frame_bytes[1], frame_bytes[2]


def serial_process_worker(name, port, baudrate, frame_mode, out_queue, stop_event):
    """Target de multiprocessing.Process: corre en un proceso del sistema
    operativo aparte del proceso de la GUI. Abre el puerto serie, decodifica
    tramas binarias fijas (frame_mode = "data" o "rld") y manda batches ya
    decodificados por out_queue cada ~30 ms, para no saturar la cola con un
    mensaje por muestra individual a 8 kSPS.

    Debe ser una función de módulo (no un método) para que multiprocessing
    la pueda picklear con el método "spawn" que usa Windows por defecto.
    """
    frame_size = DATA_FRAME_SIZE if frame_mode == "data" else RLD_FRAME_SIZE
    flush_interval = 0.03

    try:
        ser = serial.Serial(port, baudrate, timeout=0.05)
    except Exception as e:
        out_queue.put(("status", False, f"Error: {e}"))
        return

    out_queue.put(("status", True, f"Conectado a {port}"))

    buffer = bytearray()
    batch_a = []
    batch_b = []
    last_flush = time.time()

    try:
        while not stop_event.is_set() and ser.is_open:
            n = ser.in_waiting
            if n:
                buffer.extend(ser.read(n))
                frames = extract_frames(buffer, frame_size)

                for frame_bytes in frames:
                    if frame_mode == "data":
                        ch_a, ch_b = decode_data_frame(frame_bytes)
                    else:
                        ch_a, ch_b = decode_rld_frame(frame_bytes)
                    batch_a.append(ch_a)
                    batch_b.append(ch_b)
            else:
                time.sleep(0.005)

            now = time.time()
            if batch_a and (now - last_flush) >= flush_interval:
                tag = "frame" if frame_mode == "data" else "rld"
                out_queue.put((
                    tag,
                    np.array(batch_a, dtype=np.int32),
                    np.array(batch_b, dtype=np.int32),
                ))
                batch_a = []
                batch_b = []
                last_flush = now

    except Exception as e:
        out_queue.put(("status", False, f"Error de lectura: {e}"))
    finally:
        try:
            ser.close()
        except Exception:
            pass
        out_queue.put(("status", False, "Desconectado"))


# ==================================================
# CDC0/CDC1: texto por línea (sin cambios respecto de antes)
# ==================================================


class SerialWorker(QObject):
    data_received = pyqtSignal(str, str)
    status_changed = pyqtSignal(str, bool, str)

    def __init__(self, name: str, port: str, baudrate: int):
        super().__init__()
        self.name = name
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False

    @pyqtSlot()
    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            self.running = True
            self.status_changed.emit(self.name, True, f"Conectado a {self.port}")

            time.sleep(0.2)
            self.data_received.emit(self.name, f"--- {self.name} conectado en {self.port} ---")

            buffer = bytearray()

            while self.running and self.ser and self.ser.is_open:
                try:
                    n = self.ser.in_waiting
                    if n:
                        data = self.ser.read(n)
                        buffer.extend(data)

                        while b"\n" in buffer:
                            line, _, buffer = buffer.partition(b"\n")
                            text = line.decode(errors="ignore").rstrip("\r")
                            if text:
                                self.data_received.emit(self.name, text)
                    else:
                        time.sleep(0.01)

                except Exception as e:
                    self.data_received.emit(self.name, f"[ERROR lectura] {e}")
                    break

        except Exception as e:
            self.status_changed.emit(self.name, False, f"Error: {e}")

        finally:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass

            self.status_changed.emit(self.name, False, "Desconectado")

    def stop(self):
        self.running = False

    def write_line(self, text: str):
        if self.ser and self.ser.is_open:
            self.ser.write((text + "\n").encode())


class CdcPanel(QGroupBox):
    send_requested = pyqtSignal(str, str)

    def __init__(self, name: str):
        super().__init__(name)
        self.name = name

        self.setMinimumHeight(160)

        self.status = QLabel("Desconectado")
        self.status.setStyleSheet("color: red; font-weight: bold;")

        self.output = QTextEdit()
        self.output.setReadOnly(True)

        self.input = QLineEdit()
        self.input.setPlaceholderText("Escribir mensaje para enviar por este CDC...")

        self.send_button = QPushButton("Enviar")
        self.clear_button = QPushButton("Limpiar")

        bottom = QHBoxLayout()
        bottom.addWidget(self.input)
        bottom.addWidget(self.send_button)
        bottom.addWidget(self.clear_button)

        layout = QVBoxLayout()
        layout.addWidget(self.status)
        layout.addWidget(self.output)
        layout.addLayout(bottom)
        self.setLayout(layout)

        self.send_button.clicked.connect(self._send)
        self.input.returnPressed.connect(self._send)
        self.clear_button.clicked.connect(self.output.clear)

    def _send(self):
        text = self.input.text().strip()
        if text:
            self.send_requested.emit(self.name, text)
            self.append(f"> {text}")
            self.input.clear()

    def append(self, text: str):
        self.output.append(text)

    def set_connected(self, connected: bool, msg: str):
        self.status.setText(msg)
        if connected:
            self.status.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.status.setStyleSheet("color: red; font-weight: bold;")


# ==================================================
# CDC2-5: plot en tiempo real, un widget por canal físico (1-8)
# ==================================================


class SingleChannelPlot(QGroupBox):
    """Panel de visualización en tiempo real para UN canal físico del
    ADS1298. Mantiene un buffer circular y se redibuja con un timer
    separado (ver MainWindow.refresh_plots), desacoplando la tasa de
    llegada de datos (hasta 8 kSPS) de la tasa de refresco de pantalla.
    El eje X se expresa en segundos (tiempo transcurrido dentro de la
    ventana visible, ver refresh()). Por defecto ambos ejes autoescalan
    (pyqtgraph autorange); MainWindow._on_scale_changed puede fijarlos a un
    rango manual (ver set_scale_mode)."""

    WINDOW_SECONDS = 4.0
    ASSUMED_SAMPLE_RATE = 8000  # cap de data rate del firmware, ver CLAUDE.md

    def __init__(self, channel: int):
        super().__init__(f"Canal {channel}")
        self.channel = channel
        self.gain = DEFAULT_GAIN
        self.filter_sos = None
        self.scale_auto = True
        self.manual_x_seconds = self.WINDOW_SECONDS
        self.manual_y_mv = 5.0
        # Mínimos chicos y tipografía/interlineado reducidos: con hasta 4
        # canales apilados en una misma columna (ver
        # MainWindow._rebuild_channel_grid) tienen que entrar sin necesitar
        # scrollbar.
        self.setMinimumHeight(90)
        self.setMinimumWidth(150)
        self.setStyleSheet("QGroupBox { font-size: 10px; margin-top: 4px; }")

        # Cartel de lead-off (ver set_leadoff/MainWindow._update_leadoff_status):
        # vacío y sin fondo la mayor parte del tiempo, solo se pinta cuando
        # ese canal está realmente desconectado. Antes acá se mostraba el
        # estado de conexión del CDC de datos ("Conectado a COMx"/
        # "Desconectado"), quitado porque no aportaba nada que no estuviera
        # ya en el label agregado de la barra superior y confundía con el
        # aviso de lead-off.
        self.leadoff_label = QLabel("")
        self.leadoff_label.setStyleSheet("font-size: 9px;")

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("k")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("left", "mV", **{"font-size": "9px"})
        self.plot_widget.setLabel("bottom", "Tiempo (s)", **{"font-size": "9px"})
        tick_font = QFont()
        tick_font.setPixelSize(9)
        self.plot_widget.getAxis("left").setTickFont(tick_font)
        self.plot_widget.getAxis("bottom").setTickFont(tick_font)
        self.plot_widget.setDownsampling(mode="peak")
        self.plot_widget.setClipToView(True)
        self.plot_widget.enableAutoRange(axis="x", enable=True)
        self.plot_widget.enableAutoRange(axis="y", enable=True)

        self.curve = self.plot_widget.plot(pen=pg.mkPen("y", width=1))

        buffer_size = int(self.WINDOW_SECONDS * self.ASSUMED_SAMPLE_RATE)
        self.buffer = np.zeros(buffer_size, dtype=np.int32)
        self.write_pos = 0
        self.filled = 0

        layout = QVBoxLayout()
        layout.setContentsMargins(4, 2, 4, 4)
        layout.setSpacing(2)
        layout.addWidget(self.leadoff_label)
        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

    def set_leadoff(self, disconnected: bool):
        """Cartel "Electrodo desconectado" bajo el título del canal (ver
        MainWindow._update_leadoff_status, que lo llama con el bit de
        LOFF_STATP/LOFF_STATN de este canal). Solo tiene efecto visible
        mientras "Lead-off activo" está tildado en la sidebar; en cualquier
        otro momento se llama con disconnected=False y queda vacío."""
        if disconnected:
            self.leadoff_label.setText("Electrodo desconectado")
            self.leadoff_label.setStyleSheet(
                "color: white; background-color: #b00020; font-weight: bold; font-size: 9px;"
            )
        else:
            self.leadoff_label.setText("")
            self.leadoff_label.setStyleSheet("font-size: 9px;")

    def set_gain(self, gain: int):
        """Actualiza la ganancia usada para convertir a mV (llamado desde
        MainWindow cuando cambia el combo GAIN de este canal en la pestaña
        de Configuración)."""
        self.gain = gain

    def set_filter(self, sos):
        """Actualiza el filtro (array SOS de scipy, o None para desactivar)
        aplicado a la señal antes de graficarla (ver
        MainWindow._on_filter_changed / build_filter_sos)."""
        self.filter_sos = sos

    def set_scale_mode(self, auto: bool, x_seconds: float, y_mv: float):
        """Fija el modo de escala de los ejes (ver sidebar "Escala de los
        plots"). En modo automático (auto=True) se usa el autorango de
        pyqtgraph como hasta ahora; en modo manual se fija el eje X a los
        últimos x_seconds segundos (dentro de la ventana bufferizada de
        WINDOW_SECONDS) y el eje Y al rango simétrico [-y_mv, y_mv]."""
        self.scale_auto = auto
        self.manual_x_seconds = x_seconds
        self.manual_y_mv = y_mv

        self.plot_widget.enableAutoRange(axis="x", enable=auto)
        self.plot_widget.enableAutoRange(axis="y", enable=auto)

        if not auto:
            xmax = self.WINDOW_SECONDS
            xmin = max(0.0, xmax - x_seconds)
            self.plot_widget.setXRange(xmin, xmax, padding=0)
            self.plot_widget.setYRange(-y_mv, y_mv, padding=0)

    def append_batch(self, values: np.ndarray):
        n = len(values)
        if n == 0:
            return

        buffer_size = len(self.buffer)

        if n >= buffer_size:
            # El batch ya llena (o excede) toda la ventana visible.
            self.buffer[:] = values[-buffer_size:]
            self.write_pos = 0
            self.filled = buffer_size
            return

        end_pos = self.write_pos + n
        if end_pos <= buffer_size:
            self.buffer[self.write_pos:end_pos] = values
        else:
            first_part = buffer_size - self.write_pos
            self.buffer[self.write_pos:] = values[:first_part]
            self.buffer[:n - first_part] = values[first_part:]

        self.write_pos = end_pos % buffer_size
        self.filled = min(buffer_size, self.filled + n)

    def refresh(self):
        if self.filled == 0:
            return

        if self.filled < len(self.buffer):
            data = self.buffer[:self.filled]
        else:
            # Buffer circular lleno: reordenar para que quede en orden temporal.
            data = np.concatenate((self.buffer[self.write_pos:], self.buffer[:self.write_pos]))

        # La conversión a mV usa la ganancia configurada más reciente; si el
        # usuario cambió la ganancia hace poco, las muestras más viejas del
        # buffer quedan convertidas con la ganancia nueva (aproximación
        # aceptable para visualización, no para medición de precisión).
        mv_data = code_to_millivolts(data, self.gain)

        # Filtro de fase cero sobre toda la ventana visible en cada
        # redibujado: evita tener que mantener estado de filtro (zi) entre
        # batches de datos. Si la ventana todavía es muy corta para el
        # padlen que necesita sosfiltfilt (recién arrancando el streaming),
        # se muestra sin filtrar hasta que se acumulen más muestras.
        if self.filter_sos is not None:
            try:
                mv_data = sosfiltfilt(self.filter_sos, mv_data)
            except ValueError:
                pass

        time_axis = np.arange(len(mv_data)) / self.ASSUMED_SAMPLE_RATE
        self.curve.setData(time_axis, mv_data)


# ==================================================
# CDC6: estado de RLD / lead-off
# ==================================================


class RldStatusPanel(QGroupBox):
    """Panel de estado de lead-off (LOFF_STATP/LOFF_STATN) para CDC6.
    Un bit en 1 significa "electrodo desconectado" (ver datasheet ADS1298,
    registros LOFF_STATP/LOFF_STATN)."""

    def __init__(self, name: str):
        super().__init__(name)
        self.name = name
        self.setMinimumHeight(160)

        self.status = QLabel("Desconectado")
        self.status.setStyleSheet("color: red; font-weight: bold;")

        self.statp_labels = []
        self.statn_labels = []

        grid = QGridLayout()
        grid.addWidget(QLabel("<b>Canal</b>"), 0, 0)
        for ch in range(8):
            grid.addWidget(QLabel(str(ch + 1)), 0, ch + 1)

        grid.addWidget(QLabel("<b>LOFF_STATP</b>"), 1, 0)
        for ch in range(8):
            lbl = QLabel("-")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet("background-color: #333; color: white; font-weight: bold;")
            grid.addWidget(lbl, 1, ch + 1)
            self.statp_labels.append(lbl)

        grid.addWidget(QLabel("<b>LOFF_STATN</b>"), 2, 0)
        for ch in range(8):
            lbl = QLabel("-")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet("background-color: #333; color: white; font-weight: bold;")
            grid.addWidget(lbl, 2, ch + 1)
            self.statn_labels.append(lbl)

        layout = QVBoxLayout()
        layout.addWidget(self.status)
        layout.addLayout(grid)
        layout.addStretch()
        self.setLayout(layout)

    def set_connected(self, connected: bool, msg: str):
        self.status.setText(msg)
        color = "green" if connected else "red"
        self.status.setStyleSheet(f"color: {color}; font-weight: bold;")

    def update_status(self, statp: int, statn: int):
        for ch in range(8):
            self._set_bit(self.statp_labels[ch], (statp >> ch) & 1)
            self._set_bit(self.statn_labels[ch], (statn >> ch) & 1)

    @staticmethod
    def _set_bit(label: QLabel, bit: int):
        if bit:
            label.setText("OFF")
            label.setStyleSheet("background-color: #b00020; color: white; font-weight: bold;")
        else:
            label.setText("OK")
            label.setStyleSheet("background-color: #1b5e20; color: white; font-weight: bold;")


# ==================================================
# Pestaña de configuración (comandos de CDC1)
# ==================================================

GAIN_VALUES = ["1", "2", "3", "4", "6", "8", "12"]
MUX_MODES = ["NORMAL", "SHORT", "RLD_MEAS", "SUPPLY", "TEMP", "TEST", "RLD_DRP", "RLD_DRN"]

# Mapeos inversos para decodificar las respuestas de RREG (deben coincidir
# exactamente con ads1298::gainToCode() y los bits de inc/ads1298.h).
GAIN_CODE_TO_VALUE = {0: 6, 1: 1, 2: 2, 3: 3, 4: 4, 5: 8, 6: 12}
TEST_FREQ_CODE_TO_TEXT = {0x00: "SLOW", 0x01: "FAST", 0x03: "DC"}
LOFF_CURRENT_CODE_TO_TEXT = {0x00: "6", 0x01: "12", 0x02: "18", 0x03: "24"}
LOFF_MODE_CODE_TO_TEXT = {0x03: "DC", 0x01: "AC"}


class ConfigPanel(QWidget):
    """Arma y emite los comandos de la API de configuración de CDC1 (ver la
    tabla completa en CLAUDE.md). No manda nada hasta que el usuario
    interactúa con un control (los valores iniciales se fijan antes de
    conectar las señales, para no disparar comandos al construir la UI).

    apply_register() actualiza los controles a partir de una respuesta real
    de RREG, sin volver a mandar comandos (bloquea las señales mientras
    escribe), para reflejar la configuración con la que vino el ADS1298."""

    command_requested = pyqtSignal(str)
    gain_changed = pyqtSignal(int, int)  # (canal 1-8, ganancia)
    reread_requested = pyqtSignal()

    def __init__(self):
        super().__init__()

        self.gain_combos = {}
        self.power_checks = {}
        self.mux_combos = {}
        self.loffp_checks = {}
        self.loffn_checks = {}
        self.rldp_checks = {}
        self.rldn_checks = {}

        self.mode_combo = None
        self.rld_combo = None
        self.testsig_combo = None
        self.amp_combo = None
        self.freq_combo = None
        self.loff_mode_combo = None
        self.loff_type_combo = None
        self.loff_current_combo = None
        self.loff_threshold_combo = None

        layout = QVBoxLayout()
        layout.addWidget(self._build_channel_table())
        layout.addWidget(self._build_global_controls())

        reread_button = QPushButton("Releer configuración actual del ADS1298")
        reread_button.clicked.connect(self.reread_requested.emit)
        layout.addWidget(reread_button)

        layout.addStretch()
        self.setLayout(layout)

    def _send(self, command: str):
        self.command_requested.emit(command)

    def _on_gain_change(self, channel: int, gain_text: str):
        self._send(f"GAIN {channel} {gain_text}")
        self.gain_changed.emit(channel, int(gain_text))

    def _build_channel_table(self) -> QGroupBox:
        box = QGroupBox("Configuración por canal")
        grid = QGridLayout()

        headers = ["Canal", "Ganancia", "Encendido", "Mux", "LOFF P", "LOFF N", "RLD P", "RLD N"]
        for col, text in enumerate(headers):
            grid.addWidget(QLabel(f"<b>{text}</b>"), 0, col)

        for ch in range(1, 9):
            row = ch
            grid.addWidget(QLabel(str(ch)), row, 0)

            gain_combo = QComboBox()
            gain_combo.addItems(GAIN_VALUES)
            gain_combo.setCurrentText("6")
            gain_combo.currentTextChanged.connect(
                lambda value, c=ch: self._on_gain_change(c, value)
            )
            grid.addWidget(gain_combo, row, 1)
            self.gain_combos[ch] = gain_combo

            power_check = QCheckBox()
            power_check.setChecked(True)
            power_check.toggled.connect(
                lambda checked, c=ch: self._send(f"CHPOWER {c} {'ON' if checked else 'OFF'}")
            )
            grid.addWidget(power_check, row, 2)
            self.power_checks[ch] = power_check

            mux_combo = QComboBox()
            mux_combo.addItems(MUX_MODES)
            mux_combo.currentTextChanged.connect(
                lambda value, c=ch: self._send(f"MUX {c} {value}")
            )
            grid.addWidget(mux_combo, row, 3)
            self.mux_combos[ch] = mux_combo

            loffp_check = QCheckBox()
            loffp_check.toggled.connect(
                lambda checked, c=ch: self._send(f"LOFF_CH {c} P {1 if checked else 0}")
            )
            grid.addWidget(loffp_check, row, 4)
            self.loffp_checks[ch] = loffp_check

            loffn_check = QCheckBox()
            loffn_check.toggled.connect(
                lambda checked, c=ch: self._send(f"LOFF_CH {c} N {1 if checked else 0}")
            )
            grid.addWidget(loffn_check, row, 5)
            self.loffn_checks[ch] = loffn_check

            rldp_check = QCheckBox()
            rldp_check.toggled.connect(
                lambda checked, c=ch: self._send(f"RLD_CH {c} P {1 if checked else 0}")
            )
            grid.addWidget(rldp_check, row, 6)
            self.rldp_checks[ch] = rldp_check

            rldn_check = QCheckBox()
            rldn_check.toggled.connect(
                lambda checked, c=ch: self._send(f"RLD_CH {c} N {1 if checked else 0}")
            )
            grid.addWidget(rldn_check, row, 7)
            self.rldn_checks[ch] = rldn_check

        box.setLayout(grid)
        return box

    def _build_global_controls(self) -> QGroupBox:
        box = QGroupBox("Configuración global")
        layout = QGridLayout()

        layout.addWidget(QLabel("Modo:"), 0, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["HR", "LP"])
        self.mode_combo.currentTextChanged.connect(lambda v: self._send(f"MODE {v}"))
        layout.addWidget(self.mode_combo, 0, 1)

        layout.addWidget(QLabel("RLD:"), 0, 2)
        self.rld_combo = QComboBox()
        self.rld_combo.addItems(["OFF", "ON"])
        self.rld_combo.currentTextChanged.connect(lambda v: self._send(f"RLD {v}"))
        layout.addWidget(self.rld_combo, 0, 3)

        layout.addWidget(QLabel("Test signal:"), 1, 0)
        self.testsig_combo = QComboBox()
        self.testsig_combo.addItems(["OFF", "ON"])
        self.testsig_combo.currentTextChanged.connect(lambda v: self._send(f"TESTSIG {v}"))
        layout.addWidget(self.testsig_combo, 1, 1)

        layout.addWidget(QLabel("Amplitud:"), 1, 2)
        self.amp_combo = QComboBox()
        self.amp_combo.addItems(["1", "2"])
        self.amp_combo.currentTextChanged.connect(lambda v: self._send(f"TESTSIG_AMP {v}"))
        layout.addWidget(self.amp_combo, 1, 3)

        layout.addWidget(QLabel("Frecuencia:"), 1, 4)
        self.freq_combo = QComboBox()
        self.freq_combo.addItems(["SLOW", "FAST", "DC"])
        self.freq_combo.currentTextChanged.connect(lambda v: self._send(f"TESTSIG_FREQ {v}"))
        layout.addWidget(self.freq_combo, 1, 5)

        layout.addWidget(QLabel("Lead-off modo:"), 2, 0)
        self.loff_mode_combo = QComboBox()
        self.loff_mode_combo.addItems(["DC", "AC"])
        self.loff_mode_combo.currentTextChanged.connect(lambda v: self._send(f"LOFF_MODE {v}"))
        layout.addWidget(self.loff_mode_combo, 2, 1)

        layout.addWidget(QLabel("Lead-off tipo:"), 2, 2)
        self.loff_type_combo = QComboBox()
        self.loff_type_combo.addItems(["RESISTOR", "CURRENT"])
        self.loff_type_combo.currentTextChanged.connect(lambda v: self._send(f"LOFF_TYPE {v}"))
        layout.addWidget(self.loff_type_combo, 2, 3)

        layout.addWidget(QLabel("Lead-off corriente (nA):"), 3, 0)
        self.loff_current_combo = QComboBox()
        self.loff_current_combo.addItems(["6", "12", "18", "24"])
        self.loff_current_combo.currentTextChanged.connect(lambda v: self._send(f"LOFF_CURRENT {v}"))
        layout.addWidget(self.loff_current_combo, 3, 1)

        layout.addWidget(QLabel("Lead-off umbral (0-7):"), 3, 2)
        self.loff_threshold_combo = QComboBox()
        self.loff_threshold_combo.addItems([str(i) for i in range(8)])
        self.loff_threshold_combo.currentTextChanged.connect(lambda v: self._send(f"LOFF_THRESHOLD {v}"))
        layout.addWidget(self.loff_threshold_combo, 3, 3)

        start_button = QPushButton("START")
        start_button.clicked.connect(lambda: self._send("START"))
        layout.addWidget(start_button, 4, 0)

        stop_button = QPushButton("STOP")
        stop_button.clicked.connect(lambda: self._send("STOP"))
        layout.addWidget(stop_button, 4, 1)

        box.setLayout(layout)
        return box

    # ---- Lectura del estado real del ADS1298 (respuestas de RREG) ----

    def apply_register(self, addr: int, value: int):
        """Actualiza los controles a partir de una respuesta real de RREG
        (ver MainWindow.read_current_configuration /
        MainWindow.on_data_received), sin re-emitir comandos."""
        if addr == 0x01:  # CONFIG1
            self._set_combo_silently(self.mode_combo, "HR" if value & 0x80 else "LP")
        elif addr == 0x02:  # CONFIG2
            self._set_combo_silently(self.testsig_combo, "ON" if value & 0x10 else "OFF")
            self._set_combo_silently(self.amp_combo, "2" if value & 0x04 else "1")
            self._set_combo_silently(self.freq_combo, TEST_FREQ_CODE_TO_TEXT.get(value & 0x03))
        elif addr == 0x03:  # CONFIG3
            self._set_combo_silently(self.rld_combo, "ON" if value & 0x04 else "OFF")
        elif addr == 0x04:  # LOFF
            self._set_combo_silently(self.loff_threshold_combo, str((value & 0xE0) >> 5))
            self._set_combo_silently(self.loff_type_combo, "RESISTOR" if value & 0x10 else "CURRENT")
            self._set_combo_silently(
                self.loff_current_combo, LOFF_CURRENT_CODE_TO_TEXT.get((value & 0x0C) >> 2)
            )
            self._set_combo_silently(self.loff_mode_combo, LOFF_MODE_CODE_TO_TEXT.get(value & 0x03))
        elif 0x05 <= addr <= 0x0C:  # CH1SET..CH8SET
            ch = addr - 0x05 + 1
            gain_text = GAIN_CODE_TO_VALUE.get((value & 0x70) >> 4)
            if gain_text is not None:
                self._set_combo_silently(self.gain_combos.get(ch), str(gain_text))
                # Propaga también a la sidebar de la pestaña "Monitor" (ver
                # MainWindow.on_gain_changed), para que la lectura real de
                # configuración al conectar quede reflejada en ambos lados.
                self.gain_changed.emit(ch, gain_text)
            self._set_check_silently(self.power_checks.get(ch), not bool(value & 0x80))
            mux_code = value & 0x07
            if mux_code < len(MUX_MODES):
                self._set_combo_silently(self.mux_combos.get(ch), MUX_MODES[mux_code])
        elif addr == 0x0D:  # RLD_SENSP
            self._apply_channel_bits(value, self.rldp_checks)
        elif addr == 0x0E:  # RLD_SENSN
            self._apply_channel_bits(value, self.rldn_checks)
        elif addr == 0x0F:  # LOFF_SENSP
            self._apply_channel_bits(value, self.loffp_checks)
        elif addr == 0x10:  # LOFF_SENSN
            self._apply_channel_bits(value, self.loffn_checks)

    @staticmethod
    def _apply_channel_bits(value: int, checks: dict):
        for ch, check in checks.items():
            ConfigPanel._set_check_silently(check, bool((value >> (ch - 1)) & 1))

    @staticmethod
    def _set_combo_silently(combo, text):
        if combo is None or text is None:
            return
        idx = combo.findText(text)
        if idx < 0:
            return
        combo.blockSignals(True)
        combo.setCurrentIndex(idx)
        combo.blockSignals(False)

    @staticmethod
    def _set_check_silently(check, checked: bool):
        if check is None:
            return
        check.blockSignals(True)
        check.setChecked(checked)
        check.blockSignals(False)


# ==================================================
# Pestaña "Señales de prueba": test signal interna del ADS1298
# ==================================================


class TestSignalPanel(QWidget):
    """Controles simplificados para validar la test signal interna del
    ADS1298 (ver CLAUDE.md, sección "Próximos pasos", validación en 3
    etapas). Manda los mismos comandos de CDC1 que la tabla granular de
    "Avanzado" (TESTSIG/TESTSIG_AMP/TESTSIG_FREQ/MUX/START/STOP), pero con
    controles pensados para probar rápido: un checkbox de "enviar test
    signal" por canal en vez del combo completo de 8 modos de MUX.

    Es un panel independiente del ConfigPanel de "Avanzado": ambos mandan
    comandos por el mismo mecanismo (command_requested -> CDC1) pero no
    comparten estado visual entre sí (si se cambia algo desde un lado, el
    combo del otro no se actualiza solo, salvo relectura de configuración)."""

    command_requested = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        self.mux_test_checks = {}

        layout = QVBoxLayout()

        global_box = QGroupBox("Test signal interna")
        global_layout = QGridLayout()

        global_layout.addWidget(QLabel("Test signal:"), 0, 0)
        self.testsig_combo = QComboBox()
        self.testsig_combo.addItems(["OFF", "ON"])
        self.testsig_combo.currentTextChanged.connect(lambda v: self._send(f"TESTSIG {v}"))
        global_layout.addWidget(self.testsig_combo, 0, 1)

        global_layout.addWidget(QLabel("Amplitud:"), 0, 2)
        self.amp_combo = QComboBox()
        self.amp_combo.addItems(["1", "2"])
        self.amp_combo.currentTextChanged.connect(lambda v: self._send(f"TESTSIG_AMP {v}"))
        global_layout.addWidget(self.amp_combo, 0, 3)

        global_layout.addWidget(QLabel("Frecuencia:"), 0, 4)
        self.freq_combo = QComboBox()
        self.freq_combo.addItems(["SLOW", "FAST", "DC"])
        self.freq_combo.currentTextChanged.connect(lambda v: self._send(f"TESTSIG_FREQ {v}"))
        global_layout.addWidget(self.freq_combo, 0, 5)

        start_button = QPushButton("START")
        start_button.clicked.connect(lambda: self._send("START"))
        global_layout.addWidget(start_button, 1, 0)

        stop_button = QPushButton("STOP")
        stop_button.clicked.connect(lambda: self._send("STOP"))
        global_layout.addWidget(stop_button, 1, 1)

        global_box.setLayout(global_layout)
        layout.addWidget(global_box)

        channel_box = QGroupBox("Enviar test signal a estos canales (MUX TEST/NORMAL)")
        channel_layout = QGridLayout()
        for ch in range(1, 9):
            check = QCheckBox(f"Canal {ch}")
            check.toggled.connect(
                lambda checked, c=ch: self._send(f"MUX {c} {'TEST' if checked else 'NORMAL'}")
            )
            row, col = divmod(ch - 1, 4)
            channel_layout.addWidget(check, row, col)
            self.mux_test_checks[ch] = check
        channel_box.setLayout(channel_layout)
        layout.addWidget(channel_box)

        note = QLabel(
            "Activá \"Test signal\" y tildá los canales a probar; la onda "
            "cuadrada resultante se ve en los plots de la pestaña "
            "\"Monitor\". Destildá el canal (vuelve a MUX NORMAL) antes de "
            "medir una señal real."
        )
        note.setWordWrap(True)
        layout.addWidget(note)

        layout.addStretch()
        self.setLayout(layout)

    def _send(self, command: str):
        self.command_requested.emit(command)


# ==================================================
# Pestaña "Análisis": carga y visualización de grabaciones CSV previas
# ==================================================


class RecordingAnalysisPanel(QWidget):
    """Carga un CSV grabado por MainWindow._start_recording (formato tidy:
    columnas tiempo_s, canal, mV, ver docstring de _start_recording) y lo
    visualiza en un plot propio, totalmente independiente de la adquisición
    en vivo (no toca self.connections ni ningún estado de MainWindow).

    El recuadro "Amplitud del rango seleccionado" a la derecha del plot
    muestra, dentro del rango de tiempo elegido: pico a pico (max(mV) -
    min(mV)) y RMS (sqrt(mean(mV^2)), fórmula estándar para una señal
    discreta x[n] con N muestras: RMS = sqrt((1/N) * sum(x[n]^2))). Ese
    rango se elige arrastrando los bordes de una región (pg.LinearRegionItem)
    superpuesta al plot -- no hace falta tipear números; el rango completo
    del canal se ve al cargar el archivo o al tocar "Ver todo el rango"."""

    def __init__(self):
        super().__init__()

        # canal -> (tiempo_s, mV) como np.ndarray, ordenados por tiempo.
        self.channel_data = {}
        self.current_channel = None

        root = QHBoxLayout()
        self.setLayout(root)

        # ---- Columna izquierda: carga de archivo + plot ----
        left = QVBoxLayout()

        load_row = QHBoxLayout()
        self.load_button = QPushButton("Cargar grabación...")
        self.load_button.clicked.connect(self._load_recording)
        load_row.addWidget(self.load_button)

        self.file_label = QLabel("Ninguna grabación cargada")
        load_row.addWidget(self.file_label)
        load_row.addStretch()
        left.addLayout(load_row)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("k")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("left", "mV")
        self.plot_widget.setLabel("bottom", "Tiempo (s)")
        self.curve = self.plot_widget.plot(pen=pg.mkPen("y", width=1))

        # Región arrastrable que define el rango de tiempo sobre el que se
        # calcula la amplitud pico a pico (ver _on_region_changed). Su
        # extensión vertical es infinita (orientation vertical por
        # defecto), pyqtgraph la excluye del autorange del eje Y.
        self.region = pg.LinearRegionItem()
        self.region.setZValue(10)
        self.region.sigRegionChanged.connect(self._on_region_changed)
        self.plot_widget.addItem(self.region)

        left.addWidget(self.plot_widget)
        root.addLayout(left, stretch=4)

        # ---- Columna derecha: selección de canal + amplitud pico a pico ----
        right = QVBoxLayout()

        channel_box = QGroupBox("Canal")
        channel_layout = QVBoxLayout()
        self.channel_combo = QComboBox()
        self.channel_combo.currentTextChanged.connect(self._on_channel_changed)
        channel_layout.addWidget(self.channel_combo)
        channel_box.setLayout(channel_layout)
        right.addWidget(channel_box)

        amplitude_box = QGroupBox("Amplitud del rango seleccionado")
        amplitude_layout = QVBoxLayout()

        self.range_label = QLabel("Rango: -")
        self.range_label.setWordWrap(True)
        amplitude_layout.addWidget(self.range_label)

        amplitude_layout.addWidget(QLabel("Pico a pico:"))
        self.p2p_label = QLabel("- mV")
        self.p2p_label.setStyleSheet("font-size: 22px; font-weight: bold;")
        amplitude_layout.addWidget(self.p2p_label)

        amplitude_layout.addWidget(QLabel("RMS:"))
        self.rms_label = QLabel("- mV")
        self.rms_label.setStyleSheet("font-size: 22px; font-weight: bold;")
        amplitude_layout.addWidget(self.rms_label)

        self.reset_range_button = QPushButton("Ver todo el rango")
        self.reset_range_button.clicked.connect(self._reset_region)
        amplitude_layout.addWidget(self.reset_range_button)

        amplitude_box.setLayout(amplitude_layout)
        right.addWidget(amplitude_box)

        right.addStretch()
        root.addLayout(right, stretch=1)

    # ---- Carga de archivo ----

    def _load_recording(self):
        path, _ = QFileDialog.getOpenFileName(self, "Cargar grabación", "", "CSV (*.csv)")
        if not path:
            return

        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            data = self._read_tidy_csv(path)
        except (OSError, ValueError) as exc:
            QApplication.restoreOverrideCursor()
            QMessageBox.warning(self, "Error al cargar", f"No se pudo leer el archivo:\n{exc}")
            return
        QApplication.restoreOverrideCursor()

        if not data:
            QMessageBox.warning(
                self, "Archivo vacío",
                "El archivo no tiene datos con el formato esperado (tiempo_s, canal, mV).",
            )
            return

        self.channel_data = data
        self.file_label.setText(os.path.basename(path))
        self.file_label.setToolTip(path)

        self.channel_combo.blockSignals(True)
        self.channel_combo.clear()
        self.channel_combo.addItems([str(ch) for ch in sorted(self.channel_data.keys())])
        self.channel_combo.blockSignals(False)

        self._on_channel_changed(self.channel_combo.currentText())

    @staticmethod
    def _read_tidy_csv(path: str):
        """Lee un CSV con columnas tiempo_s, canal, mV y devuelve
        {canal: (tiempo_s, mV)} como np.ndarray, ordenados por tiempo. Las
        filas de un mismo canal ya llegan ordenadas por tiempo en el
        archivo (ver MainWindow._record_batch), pero se ordenan igual acá
        por robustez ante archivos editados a mano."""
        raw = {}  # canal -> ([tiempo_s], [mV])
        with open(path, newline="") as f:
            reader = csv.reader(f)
            next(reader, None)  # descarta encabezado tiempo_s,canal,mV
            for row in reader:
                if len(row) < 3:
                    continue
                t_val, ch_val, mv_val = row[0], row[1], row[2]
                bucket = raw.setdefault(int(ch_val), ([], []))
                bucket[0].append(float(t_val))
                bucket[1].append(float(mv_val))

        result = {}
        for ch, (t_list, mv_list) in raw.items():
            t_arr = np.asarray(t_list, dtype=np.float64)
            mv_arr = np.asarray(mv_list, dtype=np.float64)
            order = np.argsort(t_arr)
            result[ch] = (t_arr[order], mv_arr[order])
        return result

    # ---- Selección de canal / plot ----

    def _on_channel_changed(self, text: str):
        if not text or not self.channel_data:
            return
        ch = int(text)
        self.current_channel = ch
        t_arr, mv_arr = self.channel_data[ch]
        self.curve.setData(t_arr, mv_arr)

        if len(t_arr):
            self.region.blockSignals(True)
            self.region.setRegion((float(t_arr[0]), float(t_arr[-1])))
            self.region.blockSignals(False)
        self._on_region_changed()

    def _reset_region(self):
        if self.current_channel is None or self.current_channel not in self.channel_data:
            return
        t_arr, _ = self.channel_data[self.current_channel]
        if len(t_arr):
            self.region.setRegion((float(t_arr[0]), float(t_arr[-1])))

    def _on_region_changed(self):
        if self.current_channel is None or self.current_channel not in self.channel_data:
            return
        t_arr, mv_arr = self.channel_data[self.current_channel]
        if len(t_arr) == 0:
            return

        xmin, xmax = self.region.getRegion()
        lo = np.searchsorted(t_arr, xmin, side="left")
        hi = np.searchsorted(t_arr, xmax, side="right")

        self.range_label.setText(f"Rango: {xmin:.3f} s – {xmax:.3f} s")

        if hi > lo:
            segment = mv_arr[lo:hi]
            p2p = float(segment.max() - segment.min())
            rms = float(np.sqrt(np.mean(np.square(segment))))
            self.p2p_label.setText(f"{p2p:.4f} mV")
            self.rms_label.setText(f"{rms:.4f} mV")
        else:
            self.p2p_label.setText("- mV")
            self.rms_label.setText("- mV")


# ==================================================
# Conexiones: hilo (texto, CDC0/CDC1) o proceso (binario, CDC2-6)
# ==================================================


class ThreadConnection:
    """Conexión basada en QThread + SerialWorker, usada para CDC0/CDC1."""

    def __init__(self, port: str, thread: QThread, worker: SerialWorker):
        self.port = port
        self.thread = thread
        self.worker = worker

    def write_line(self, text: str):
        self.worker.write_line(text)

    def stop(self):
        self.worker.stop()
        self.thread.quit()
        self.thread.wait(1000)


class ProcessConnection:
    """Conexión basada en multiprocessing.Process, usada para CDC2-6
    (streaming binario). No admite escritura desde la GUI."""

    def __init__(self, port: str, process: mp.Process, out_queue: mp.Queue, stop_event):
        self.port = port
        self.process = process
        self.queue = out_queue
        self.stop_event = stop_event

    def write_line(self, text: str):
        pass

    def stop(self):
        self.stop_event.set()
        self.process.join(timeout=1.0)
        if self.process.is_alive():
            self.process.terminate()


class MainWindow(QMainWindow):
    # Combos de la sidebar de filtros -> parámetro de build_filter_sos().
    _NOTCH_HZ = {"Apagado": None, "50 Hz": 50, "60 Hz": 60}
    _HPF_HZ = {"Apagado": None, "1 Hz": 1, "20 Hz": 20, "50 Hz": 50}
    _LPF_HZ = {"Apagado": None, "125 Hz": 125, "150 Hz": 150, "500 Hz": 500}

    # Combos de unidad de la sidebar "Escala de los plots" -> factor de
    # conversión a las unidades internas (segundos / mV).
    _X_UNIT_TO_SECONDS = {"s": 1.0, "ms": 0.001}
    _Y_UNIT_TO_MV = {"mV": 1.0, "µV": 0.001, "V": 1000.0}

    def __init__(self):
        super().__init__()

        self.setWindowTitle("EMG Multichannel Acquisition")
        self.resize(1400, 950)

        self.panels = {}          # CdcPanel (texto) para CDC0/CDC1, en "Avanzado"
        self.channel_plots = {}   # SingleChannelPlot por canal físico (1-8)
        self.channel_checks = {}  # QCheckBox "canal visible" (1-8), en la sidebar
        self.channel_gain_combos = {}  # QComboBox de ganancia por canal, en la sidebar
        self.visible_channels = []
        self.rld_panel = None     # RldStatusPanel para CDC6, en "Avanzado"
        self.connections = {}     # name -> ThreadConnection | ProcessConnection
        self.port_combos = {}
        self.current_filter_sos = None  # sos vigente, aplicado a los canales visibles

        # Estado de grabación a CSV (ver _build_recording_box / _start_recording).
        self.recording_active = False
        self.recording_channels = []       # canales tildados al iniciar la grabación
        self.recording_file = None
        self.recording_writer = None
        self.recording_sample_counts = {}  # canal -> cantidad de muestras ya grabadas
        self.recording_filter_state = {}   # canal -> (sos, zi) del filtro streaming (ver _record_batch)

        # Qué par de canales físicos entrega cada CDC de datos (ver
        # "Formato de trama binaria por USB" en CLAUDE.md).
        self.pair_channels = {
            CDC_NAMES[2]: (1, 2),
            CDC_NAMES[3]: (3, 4),
            CDC_NAMES[4]: (5, 6),
            CDC_NAMES[5]: (7, 8),
        }

        main = QWidget()
        self.setCentralWidget(main)

        root = QVBoxLayout()
        main.setLayout(root)

        title = QLabel("EMG MULTICHANNEL ACQUISITION")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        root.addWidget(title)

        # Barra superior: conexión de un solo botón (sin exponer CDC/puertos
        # al usuario clínico) + estado agregado + pantalla completa. El flujo
        # manual por CDC (para debug) vive en la pestaña "Avanzado".
        top = QHBoxLayout()

        self.connect_equipment_button = QPushButton("Conectar equipo")
        self.disconnect_equipment_button = QPushButton("Desconectar equipo")
        self.equipment_status_label = QLabel("Equipo desconectado")
        self.equipment_status_label.setStyleSheet("color: red; font-weight: bold;")

        self.start_recording_button = QPushButton("Iniciar grabación")
        self.stop_recording_button = QPushButton("Detener grabación")
        self.stop_recording_button.setEnabled(False)
        self.recording_status_label = QLabel("Sin grabación")

        self.fullscreen_button = QPushButton("Pantalla completa (F11)")

        top.addWidget(self.connect_equipment_button)
        top.addWidget(self.disconnect_equipment_button)
        top.addWidget(self.equipment_status_label)
        top.addSpacing(20)
        top.addWidget(self.start_recording_button)
        top.addWidget(self.stop_recording_button)
        top.addWidget(self.recording_status_label)
        top.addStretch()
        top.addWidget(self.fullscreen_button)

        root.addLayout(top)

        tabs = QTabWidget()
        root.addWidget(tabs)

        tabs.addTab(self._build_monitor_tab(), "Monitor")
        tabs.addTab(self._build_test_signal_tab(), "Señales de prueba")
        tabs.addTab(self._build_analysis_tab(), "Análisis")
        tabs.addTab(self._build_advanced_tab(), "Avanzado")

        self.connect_equipment_button.clicked.connect(self.connect_equipment)
        self.disconnect_equipment_button.clicked.connect(self.disconnect_all)
        self.start_recording_button.clicked.connect(self._start_recording)
        self.stop_recording_button.clicked.connect(self._stop_recording)
        self.fullscreen_button.clicked.connect(self.toggle_fullscreen)
        QShortcut(QKeySequence("F11"), self, activated=self.toggle_fullscreen)

        # Drena las colas de los procesos de CDC2-6 sin bloquear la UI.
        self.queue_timer = QTimer(self)
        self.queue_timer.timeout.connect(self.poll_process_queues)
        self.queue_timer.start(30)

        # Redibuja los plots a una tasa fija, desacoplada de la llegada de datos.
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.refresh_plots)
        self.plot_timer.start(33)

        self.refresh_ports()

    # ---- Construcción de las 3 pestañas ----

    def _build_monitor_tab(self) -> QWidget:
        """Pestaña principal: sidebar (~20% del ancho) con selección de
        canales/ganancia, filtros y RLD/lead-off, más el área de plots de
        los canales visibles ocupando el 80% restante (ver QSplitter), sin
        scrollbar: siempre se reparten para entrar en el espacio disponible
        (ver _rebuild_channel_grid)."""
        splitter = QSplitter(Qt.Orientation.Horizontal)

        sidebar_scroll = QScrollArea()
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setWidget(self._build_sidebar())
        splitter.addWidget(sidebar_scroll)

        # Los 8 plots se crean siempre (siguen recibiendo y bufferizando
        # datos aunque estén ocultos); solo su presencia en channel_grid y
        # su redibujado en refresh_plots() depende de la selección.
        for ch in range(1, 9):
            self.channel_plots[ch] = SingleChannelPlot(ch)

        # channel_grid es un QHBoxLayout con 1 o 2 "columnas" (QWidget con su
        # propio QVBoxLayout interno), reconstruidas enteras en cada llamada
        # a _rebuild_channel_grid según cuántos canales estén tildados.
        self.channel_grid_container = QWidget()
        self.channel_grid = QHBoxLayout()
        self.channel_grid.setContentsMargins(0, 0, 0, 0)
        self.channel_grid.setSpacing(6)
        self.channel_grid_container.setLayout(self.channel_grid)
        splitter.addWidget(self.channel_grid_container)

        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 4)
        splitter.setSizes([280, 1120])  # proporción inicial ~20/80

        self._rebuild_channel_grid()

        return splitter

    def _build_sidebar(self) -> QWidget:
        sidebar = QWidget()
        layout = QVBoxLayout()
        sidebar.setLayout(layout)

        layout.addWidget(self._build_channel_box())
        layout.addWidget(self._build_scale_box())
        layout.addWidget(self._build_filter_box())
        layout.addWidget(self._build_rld_leadoff_box())
        layout.addStretch()

        return sidebar

    def _build_channel_box(self) -> QGroupBox:
        box = QGroupBox("Canales")
        grid = QGridLayout()
        grid.addWidget(QLabel("<b>Ver</b>"), 0, 0)
        grid.addWidget(QLabel("<b>Canal</b>"), 0, 1)
        grid.addWidget(QLabel("<b>Ganancia</b>"), 0, 2)

        for ch in range(1, 9):
            row = ch

            check = QCheckBox()
            check.setChecked(True)
            check.toggled.connect(self._rebuild_channel_grid)
            grid.addWidget(check, row, 0)
            self.channel_checks[ch] = check

            grid.addWidget(QLabel(f"Canal {ch}"), row, 1)

            gain_combo = QComboBox()
            gain_combo.addItems(GAIN_VALUES)
            gain_combo.setCurrentText(str(DEFAULT_GAIN))
            gain_combo.currentTextChanged.connect(
                lambda value, c=ch: self._on_sidebar_gain_changed(c, value)
            )
            grid.addWidget(gain_combo, row, 2)
            self.channel_gain_combos[ch] = gain_combo

        box.setLayout(grid)
        return box

    def _build_scale_box(self) -> QGroupBox:
        box = QGroupBox("Escala de los plots")
        layout = QGridLayout()

        layout.addWidget(QLabel("Modo:"), 0, 0)
        self.scale_mode_combo = QComboBox()
        self.scale_mode_combo.addItems(["Automática", "Manual"])
        self.scale_mode_combo.currentTextChanged.connect(self._on_scale_changed)
        layout.addWidget(self.scale_mode_combo, 0, 1, 1, 2)

        layout.addWidget(QLabel("Eje X:"), 1, 0)
        self.x_scale_spin = QDoubleSpinBox()
        self.x_scale_spin.setRange(0.01, 60.0)
        self.x_scale_spin.setDecimals(2)
        self.x_scale_spin.setValue(SingleChannelPlot.WINDOW_SECONDS)
        self.x_scale_spin.valueChanged.connect(self._on_scale_changed)
        layout.addWidget(self.x_scale_spin, 1, 1)

        self.x_scale_unit_combo = QComboBox()
        self.x_scale_unit_combo.addItems(list(self._X_UNIT_TO_SECONDS.keys()))
        self.x_scale_unit_combo.currentTextChanged.connect(self._on_scale_changed)
        layout.addWidget(self.x_scale_unit_combo, 1, 2)

        layout.addWidget(QLabel("Eje Y:"), 2, 0)
        self.y_scale_spin = QDoubleSpinBox()
        self.y_scale_spin.setRange(0.001, 10000.0)
        self.y_scale_spin.setDecimals(3)
        self.y_scale_spin.setValue(5.0)
        self.y_scale_spin.valueChanged.connect(self._on_scale_changed)
        layout.addWidget(self.y_scale_spin, 2, 1)

        self.y_scale_unit_combo = QComboBox()
        self.y_scale_unit_combo.addItems(list(self._Y_UNIT_TO_MV.keys()))
        self.y_scale_unit_combo.currentTextChanged.connect(self._on_scale_changed)
        layout.addWidget(self.y_scale_unit_combo, 2, 2)

        box.setLayout(layout)

        self._update_scale_controls_enabled()
        return box

    def _build_filter_box(self) -> QGroupBox:
        box = QGroupBox("Filtros (canales visibles)")
        layout = QGridLayout()

        layout.addWidget(QLabel("Notch:"), 0, 0)
        self.notch_combo = QComboBox()
        self.notch_combo.addItems(list(self._NOTCH_HZ.keys()))
        self.notch_combo.currentTextChanged.connect(self._on_filter_changed)
        layout.addWidget(self.notch_combo, 0, 1)

        layout.addWidget(QLabel("Pasaaltos:"), 1, 0)
        self.hpf_combo = QComboBox()
        self.hpf_combo.addItems(list(self._HPF_HZ.keys()))
        self.hpf_combo.currentTextChanged.connect(self._on_filter_changed)
        layout.addWidget(self.hpf_combo, 1, 1)

        layout.addWidget(QLabel("Pasabajos:"), 2, 0)
        self.lpf_combo = QComboBox()
        self.lpf_combo.addItems(list(self._LPF_HZ.keys()))
        self.lpf_combo.currentTextChanged.connect(self._on_filter_changed)
        layout.addWidget(self.lpf_combo, 2, 1)

        box.setLayout(layout)
        return box

    def _build_rld_leadoff_box(self) -> QGroupBox:
        box = QGroupBox("RLD / Lead-off")
        layout = QVBoxLayout()

        self.rld_toggle = QCheckBox("RLD activo")
        self.rld_toggle.toggled.connect(self._on_rld_toggle)
        layout.addWidget(self.rld_toggle)

        self.leadoff_toggle = QCheckBox("Lead-off activo")
        self.leadoff_toggle.toggled.connect(self._on_leadoff_toggle)
        layout.addWidget(self.leadoff_toggle)

        box.setLayout(layout)
        return box

    def _build_test_signal_tab(self) -> QWidget:
        self.test_signal_panel = TestSignalPanel()
        self.test_signal_panel.command_requested.connect(self.send_config_command)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.test_signal_panel)
        return scroll

    def _build_analysis_tab(self) -> QWidget:
        """Pestaña independiente de la adquisición en vivo: carga un CSV de
        una grabación previa (ver RecordingAnalysisPanel) para revisarla
        offline sin necesitar el equipo conectado."""
        self.analysis_panel = RecordingAnalysisPanel()
        return self.analysis_panel

    def _build_advanced_tab(self) -> QWidget:
        """Todo lo que antes era la UI principal de debug: mapeo manual de
        puertos por CDC, paneles de texto crudo, estado detallado de
        lead-off/RLD y la tabla granular de configuración por canal. Nada de
        esto se recorta — solo se relocaliza acá."""
        advanced = QWidget()
        layout = QVBoxLayout()
        advanced.setLayout(layout)

        conn_row = QHBoxLayout()
        self.refresh_button = QPushButton("Buscar puertos")
        self.autodetect_button = QPushButton("Auto-detectar CDC")
        self.connect_button = QPushButton("Conectar seleccionados")
        self.disconnect_button = QPushButton("Desconectar todo")

        self.baud = QSpinBox()
        self.baud.setRange(1200, 2000000)
        self.baud.setValue(115200)
        self.baud.setSingleStep(9600)

        conn_row.addWidget(self.refresh_button)
        conn_row.addWidget(self.autodetect_button)
        conn_row.addWidget(QLabel("Baudrate:"))
        conn_row.addWidget(self.baud)
        conn_row.addWidget(self.connect_button)
        conn_row.addWidget(self.disconnect_button)
        conn_row.addStretch()
        layout.addLayout(conn_row)

        mapping_box = QGroupBox("Asignación de puertos COM")
        mapping = QGridLayout()

        for i, name in enumerate(CDC_NAMES):
            label = QLabel(name)
            combo = QComboBox()
            combo.addItem("No conectar", None)
            self.port_combos[name] = combo

            mapping.addWidget(label, i, 0)
            mapping.addWidget(combo, i, 1)

        mapping_box.setLayout(mapping)
        layout.addWidget(mapping_box)

        top_row = QHBoxLayout()
        for name in (CDC_NAMES[0], CDC_NAMES[1]):
            widget = CdcPanel(name)
            widget.send_requested.connect(self.send_to_cdc)
            self.panels[name] = widget
            top_row.addWidget(widget)

        self.rld_panel = RldStatusPanel(CDC_NAMES[6])
        top_row.addWidget(self.rld_panel)
        layout.addLayout(top_row)

        self.config_panel = ConfigPanel()
        self.config_panel.command_requested.connect(self.send_config_command)
        self.config_panel.gain_changed.connect(self.on_gain_changed)
        self.config_panel.reread_requested.connect(self.read_current_configuration)
        layout.addWidget(self.config_panel)

        layout.addStretch()

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.autodetect_button.clicked.connect(self.auto_detect_ports)
        self.connect_button.clicked.connect(self.connect_selected)
        self.disconnect_button.clicked.connect(self.disconnect_all)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(advanced)
        return scroll

    # ---- Filtros, escala, ganancia y RLD/lead-off desde la sidebar ----

    def _on_filter_changed(self, _value=None):
        notch = self._NOTCH_HZ[self.notch_combo.currentText()]
        hpf = self._HPF_HZ[self.hpf_combo.currentText()]
        lpf = self._LPF_HZ[self.lpf_combo.currentText()]

        self.current_filter_sos = build_filter_sos(
            notch, hpf, lpf, SingleChannelPlot.ASSUMED_SAMPLE_RATE
        )
        for ch in self.visible_channels:
            self.channel_plots[ch].set_filter(self.current_filter_sos)

    def _update_scale_controls_enabled(self):
        manual = self.scale_mode_combo.currentText() == "Manual"
        for widget in (
            self.x_scale_spin, self.x_scale_unit_combo,
            self.y_scale_spin, self.y_scale_unit_combo,
        ):
            widget.setEnabled(manual)

    def _on_scale_changed(self, _value=None):
        """Aplica el modo de escala (automática o manual) elegido en la
        sidebar a los 8 SingleChannelPlot (no solo a los visibles: así, si
        se tilda un canal nuevo más tarde, ya respeta la escala vigente)."""
        self._update_scale_controls_enabled()

        auto = self.scale_mode_combo.currentText() == "Automática"
        x_seconds = self.x_scale_spin.value() * self._X_UNIT_TO_SECONDS[self.x_scale_unit_combo.currentText()]
        y_mv = self.y_scale_spin.value() * self._Y_UNIT_TO_MV[self.y_scale_unit_combo.currentText()]

        for plot in self.channel_plots.values():
            plot.set_scale_mode(auto, x_seconds, y_mv)

    def _on_sidebar_gain_changed(self, channel: int, gain_text: str):
        self.send_config_command(f"GAIN {channel} {gain_text}")
        self.channel_plots[channel].set_gain(int(gain_text))

        combo = self.config_panel.gain_combos.get(channel)
        if combo is not None:
            combo.blockSignals(True)
            combo.setCurrentText(gain_text)
            combo.blockSignals(False)

    def _on_rld_toggle(self, checked: bool):
        # Habilitar CONFIG3.PD_RLD solo enciende el amplificador RLD; sin
        # seleccionar canales en RLD_SENSP/N (reset = 0x00, ninguno) el
        # amplificador queda sin ninguna entrada real y no hace rechazo de
        # modo común de verdad. Se incluyen los 8 canales para que el lazo
        # de realimentación tenga efecto perceptible al conectar el
        # electrodo de RLD.
        value = 1 if checked else 0
        for ch in range(1, 9):
            self.send_config_command(f"RLD_CH {ch} P {value}")
            self.send_config_command(f"RLD_CH {ch} N {value}")
        self.send_config_command(f"RLD {'ON' if checked else 'OFF'}")

    def _on_leadoff_toggle(self, checked: bool):
        value = 1 if checked else 0
        for ch in range(1, 9):
            self.send_config_command(f"LOFF_CH {ch} P {value}")
            self.send_config_command(f"LOFF_CH {ch} N {value}")
        if not checked:
            for plot in self.channel_plots.values():
                plot.set_leadoff(False)

    def _update_leadoff_status(self, statp: int, statn: int):
        """Muestra "Electrodo desconectado" bajo el título de cada
        SingleChannelPlot cuando LOFF_STATP/LOFF_STATN marca ese canal como
        desconectado (ver RldStatusPanel.update_status para el mismo mapeo
        de bits, en "Avanzado"). Solo tiene sentido mientras "Lead-off
        activo" está tildado: si no, los comparadores no están habilitados
        para esos canales y los bits no reflejan nada real."""
        if not self.leadoff_toggle.isChecked():
            return
        for ch in range(1, 9):
            bit = ch - 1
            disconnected = bool((statp >> bit) & 1) or bool((statn >> bit) & 1)
            self.channel_plots[ch].set_leadoff(disconnected)

    # ---- Grabación a CSV ----

    def _start_recording(self):
        """Graba a un CSV "tidy" (columnas tiempo_s, canal, mV) los canales
        que estén tildados en la sidebar en el momento de arrancar (si se
        tildan/destildan canales después, la grabación sigue solo con los
        que se eligieron al principio). El valor mV es la señal *filtrada*
        (mismo notch/pasaaltos/pasabajos elegidos en la sidebar), aplicada
        muestra a muestra con estado persistente (ver _record_batch) para
        que la grabación coincida con lo que se ve en pantalla."""
        if self.recording_active:
            return

        channels = list(self.visible_channels)
        if not channels:
            QMessageBox.warning(
                self, "Grabación", "No hay canales visibles para grabar."
            )
            return

        folder = QFileDialog.getExistingDirectory(
            self, "Elegir carpeta de destino para la grabación"
        )
        if not folder:
            return

        # Milisegundos incluidos en el nombre para no pisar el archivo de una
        # grabación anterior si se inicia otra dentro del mismo segundo.
        timestamp = time.strftime('%Y%m%d_%H%M%S') + f"_{int(time.time() * 1000) % 1000:03d}"
        filename = f"emg_recording_{timestamp}.csv"
        path = os.path.join(folder, filename)

        try:
            self.recording_file = open(path, "w", newline="")
        except OSError as e:
            QMessageBox.warning(self, "Grabación", f"No se pudo crear el archivo:\n{e}")
            return

        self.recording_writer = csv.writer(self.recording_file)
        self.recording_writer.writerow(["tiempo_s", "canal", "mV"])

        self.recording_channels = channels
        self.recording_sample_counts = {ch: 0 for ch in channels}
        self.recording_filter_state = {}
        self.recording_active = True

        self.start_recording_button.setEnabled(False)
        self.stop_recording_button.setEnabled(True)
        self.recording_status_label.setText(
            "Grabando canales " + ", ".join(str(c) for c in channels)
        )
        self.recording_status_label.setToolTip(path)

    def _stop_recording(self):
        if not self.recording_active:
            return

        self.recording_active = False
        path = self.recording_file.name if self.recording_file else ""
        if self.recording_file:
            self.recording_file.close()
        self.recording_file = None
        self.recording_writer = None

        self.start_recording_button.setEnabled(True)
        self.stop_recording_button.setEnabled(False)
        self.recording_status_label.setText("Grabación guardada")
        self.recording_status_label.setToolTip(path)

    def _record_batch(self, channel: int, raw_codes: np.ndarray):
        """Convierte un batch crudo a mV con la ganancia vigente, le aplica
        el mismo filtro elegido en la sidebar (si hay uno activo) de forma
        continua muestra a muestra (scipy.signal.sosfilt con estado zi
        persistente entre batches, a diferencia del sosfiltfilt de fase
        cero que usan los plots), y escribe las filas al CSV. Se reinicia
        el estado del filtro cada vez que cambia el sos vigente (nuevo
        objeto devuelto por build_filter_sos al tocar los combos de
        filtro)."""
        if not self.recording_active or channel not in self.recording_channels:
            return
        if len(raw_codes) == 0:
            return

        gain = self.channel_plots[channel].gain
        mv_values = code_to_millivolts(raw_codes, gain)

        sos = self.current_filter_sos
        if sos is not None:
            state = self.recording_filter_state.get(channel)
            if state is None or state[0] is not sos:
                zi = sosfilt_zi(sos) * mv_values[0]
            else:
                zi = state[1]
            mv_values, zi = sosfilt(sos, mv_values, zi=zi)
            self.recording_filter_state[channel] = (sos, zi)
        else:
            self.recording_filter_state.pop(channel, None)

        start_count = self.recording_sample_counts[channel]
        elapsed = (np.arange(len(raw_codes)) + start_count) / SingleChannelPlot.ASSUMED_SAMPLE_RATE
        self.recording_sample_counts[channel] = start_count + len(raw_codes)

        self.recording_writer.writerows(zip(elapsed, [channel] * len(raw_codes), mv_values))

    # ---- Conexión ----

    def connect_equipment(self):
        """Flujo clínico: auto-detecta y conecta las 7 interfaces del equipo
        en un solo paso, sin que el usuario tenga que saber qué es un CDC ni
        elegir puertos COM a mano (reusa _auto_assign_ports/connect_selected,
        las mismas funciones que usa el flujo manual de "Avanzado")."""
        result = self._auto_assign_ports()

        if result is None:
            if not group_ports_by_serial():
                QMessageBox.warning(
                    self,
                    "Conectar equipo",
                    "No se encontró ningún equipo EMG conectado.\n\n"
                    "Verificá el cable USB y que el equipo esté encendido."
                )
            return

        self.connect_selected()

        if result["missing"]:
            QMessageBox.warning(
                self,
                "Conectar equipo",
                "No se pudieron identificar todas las interfaces del equipo.\n"
                "Faltantes: " + ", ".join(result["missing"]) +
                "\n\nProbá reconectar el cable USB o revisar la pestaña "
                "\"Avanzado\"."
            )

    def _refresh_equipment_status_label(self):
        connected = sum(1 for name in CDC_NAMES if name in self.connections)
        total = len(CDC_NAMES)

        if connected == 0:
            self.equipment_status_label.setText("Equipo desconectado")
            self.equipment_status_label.setStyleSheet("color: red; font-weight: bold;")
        elif connected == total:
            self.equipment_status_label.setText("Equipo conectado")
            self.equipment_status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.equipment_status_label.setText(
                f"Equipo parcialmente conectado ({connected}/{total})"
            )
            self.equipment_status_label.setStyleSheet("color: #b8860b; font-weight: bold;")

    def _auto_assign_ports(self):
        """Escanea los puertos COM del dispositivo EMG (VID 0xCAFE), abre
        brevemente cada uno y lee el banner de bienvenida del firmware para
        determinar con certeza a qué CDC pertenece, completando
        self.port_combos. Reutilizado tanto por el botón clínico "Conectar
        equipo" como por "Auto-detectar CDC" en "Avanzado".

        Devuelve un dict {serial_number, assigned, missing, skipped} con el
        resumen, o None si no hay ninguna placa EMG conectada o el usuario
        canceló la selección de placa (cuando hay más de una)."""
        self.refresh_ports()

        grouped = group_ports_by_serial()
        if not grouped:
            return None

        # Los puertos ya conectados por este programa no se pueden reabrir;
        # se excluyen del escaneo para no interferir con esas conexiones.
        busy_ports = {conn.port for conn in self.connections.values()}

        serial_number = next(iter(grouped))
        if len(grouped) > 1:
            items = list(grouped.keys())
            serial_number, ok = QInputDialog.getItem(
                self,
                "Varias placas detectadas",
                "Se detectó más de un dispositivo EMG conectado.\n"
                "Selecciona el número de serie de la placa a usar:",
                items,
                0,
                False,
            )
            if not ok:
                return None

        candidates = [d for d in grouped[serial_number] if d not in busy_ports]
        skipped = [d for d in grouped[serial_number] if d in busy_ports]

        self.autodetect_button.setEnabled(False)
        self.autodetect_button.setText("Detectando...")
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)

        mapping = {}
        try:
            for device in candidates:
                self.autodetect_button.setText(f"Detectando {device}...")
                QApplication.processEvents()

                cdc_index = identify_cdc_index(device)
                if cdc_index is not None and cdc_index < len(CDC_NAMES):
                    mapping[cdc_index] = device
        finally:
            QApplication.restoreOverrideCursor()
            self.autodetect_button.setText("Auto-detectar CDC")
            self.autodetect_button.setEnabled(True)

        assigned = 0
        missing = []

        for cdc_index, name in enumerate(CDC_NAMES):
            combo = self.port_combos[name]
            device = mapping.get(cdc_index)

            if device is None:
                missing.append(name)
                continue

            idx = combo.findData(device)
            if idx < 0:
                # El puerto no estaba en la lista del combo; se agrega.
                combo.addItem(f"{device}", device)
                idx = combo.findData(device)

            combo.setCurrentIndex(idx)
            assigned += 1

        return {
            "serial_number": serial_number,
            "assigned": assigned,
            "missing": missing,
            "skipped": skipped,
        }

    def auto_detect_ports(self):
        """Botón "Auto-detectar CDC" de la pestaña "Avanzado": igual que
        antes, solo rellena self.port_combos y muestra un resumen detallado
        (sin conectar nada)."""
        result = self._auto_assign_ports()

        if result is None:
            if not group_ports_by_serial():
                QMessageBox.warning(
                    self,
                    "Auto-detección",
                    "No se encontró ningún dispositivo EMG (VID 0xCAFE) en los "
                    "puertos COM.\n\nVerifica que la Pico esté conectada y que "
                    "el firmware haya enumerado correctamente (revisa el "
                    "Administrador de dispositivos de Windows)."
                )
            return

        message = (
            f"Placa (serie {result['serial_number']}): "
            f"se asignaron {result['assigned']}/{len(CDC_NAMES)} interfaces CDC."
        )
        if result["missing"]:
            message += "\nNo se detectaron: " + ", ".join(result["missing"])
        if result["skipped"]:
            message += (
                "\nPuertos omitidos por estar ya conectados en este "
                "programa: " + ", ".join(result["skipped"])
            )

        QMessageBox.information(self, "Auto-detección", message)

    def refresh_ports(self):
        plist = list(serial.tools.list_ports.comports())

        existing = []
        for p in plist:
            existing.append(PortInfo(p.device, p.description))

        for combo in self.port_combos.values():
            current = combo.currentData()
            combo.clear()
            combo.addItem("No conectar", None)

            for p in existing:
                combo.addItem(f"{p.device} - {p.description}", p.device)

            if current:
                idx = combo.findData(current)
                if idx >= 0:
                    combo.setCurrentIndex(idx)

        if not existing:
            QMessageBox.information(self, "Puertos COM", "No se encontraron puertos COM.")

    def connect_selected(self):
        baudrate = self.baud.value()

        used_ports = {}
        for name, combo in self.port_combos.items():
            port = combo.currentData()
            if port:
                if port in used_ports:
                    QMessageBox.warning(
                        self,
                        "Puerto repetido",
                        f"El puerto {port} está asignado a más de un CDC."
                    )
                    return
                used_ports[port] = name

        for idx, name in enumerate(CDC_NAMES):
            combo = self.port_combos[name]
            port = combo.currentData()
            if not port or name in self.connections:
                continue

            if idx in (0, 1):
                self.start_thread_connection(name, port, baudrate)
            else:
                frame_mode = "rld" if idx == 6 else "data"
                self.start_process_connection(name, port, baudrate, frame_mode)

        self._refresh_equipment_status_label()

    def start_thread_connection(self, name: str, port: str, baudrate: int):
        thread = QThread()
        worker = SerialWorker(name, port, baudrate)

        worker.moveToThread(thread)

        thread.started.connect(worker.run)
        worker.data_received.connect(self.on_data_received)
        worker.status_changed.connect(self.on_status_changed)

        self.connections[name] = ThreadConnection(port, thread, worker)
        thread.start()

    def start_process_connection(self, name: str, port: str, baudrate: int, frame_mode: str):
        out_queue = mp.Queue()
        stop_event = mp.Event()

        process = mp.Process(
            target=serial_process_worker,
            args=(name, port, baudrate, frame_mode, out_queue, stop_event),
            daemon=True,
        )

        self.connections[name] = ProcessConnection(port, process, out_queue, stop_event)
        process.start()

    def disconnect_all(self):
        for name in list(self.connections.keys()):
            self.disconnect_one(name)
        self._refresh_equipment_status_label()

    def disconnect_one(self, name: str):
        conn = self.connections.pop(name, None)
        if conn:
            conn.stop()

    def poll_process_queues(self):
        for name, conn in list(self.connections.items()):
            if not isinstance(conn, ProcessConnection):
                continue

            while True:
                try:
                    msg = conn.queue.get_nowait()
                except queue.Empty:
                    break

                tag = msg[0]
                if tag == "status":
                    _, connected, text = msg
                    self._update_connection_status(name, connected, text)
                    if not connected:
                        self.connections.pop(name, None)
                    self._refresh_equipment_status_label()
                elif tag == "frame":
                    _, values_a, values_b = msg
                    channel_a, channel_b = self.pair_channels.get(name, (None, None))
                    if channel_a is not None:
                        self.channel_plots[channel_a].append_batch(values_a)
                        self.channel_plots[channel_b].append_batch(values_b)
                        self._record_batch(channel_a, values_a)
                        self._record_batch(channel_b, values_b)
                elif tag == "rld":
                    _, values_a, values_b = msg
                    if len(values_a) and len(values_b):
                        statp, statn = int(values_a[-1]), int(values_b[-1])
                        if self.rld_panel is not None:
                            self.rld_panel.update_status(statp, statn)
                        self._update_leadoff_status(statp, statn)

    def refresh_plots(self):
        # Solo redibuja los canales que están visibles: los ocultos siguen
        # bufferizando datos (poll_process_queues) pero no gastan tiempo de
        # renderizado hasta que se vuelvan a mostrar.
        for ch in self.visible_channels:
            self.channel_plots[ch].refresh()

    def _rebuild_channel_grid(self):
        """Recalcula qué canales están tildados en la sidebar "Canales" y
        reconstruye channel_grid_container entero para que todos entren en
        el espacio disponible sin scrollbar, priorizando siempre ver más
        escala horizontal (tiempo) que vertical:
        - 1-3 canales visibles: una sola columna, apilados uno arriba del
          otro (ocupando el 100% del ancho disponible si es solo 1 canal).
        - 4-8 canales visibles: dos columnas lado a lado (mitad izquierda y
          mitad derecha), cada una apilada uno arriba del otro por su
          cuenta — la izquierda se lleva ceil(n/2) canales y la derecha
          floor(n/2) (p. ej. 5 canales -> 3 y 2; 7 canales -> 4 y 3).

        Los QWidget "columna" son contenedores transitorios que se recrean
        en cada llamada; los SingleChannelPlot en cambio son permanentes y
        se re-parentan a la columna nueva (vía addWidget) antes de que Qt
        efectivamente destruya la columna vieja (deleteLater() se ejecuta
        recién en el próximo ciclo de eventos, así que el reparenting ya
        ocurrió para entonces)."""
        while self.channel_grid.count():
            item = self.channel_grid.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

        self.visible_channels = [ch for ch in range(1, 9) if self.channel_checks[ch].isChecked()]
        if not self.visible_channels:
            return

        n = len(self.visible_channels)
        if n <= 3:
            columns = [self.visible_channels]
        else:
            left_count = -(-n // 2)  # ceil(n/2)
            columns = [self.visible_channels[:left_count], self.visible_channels[left_count:]]

        for column_channels in columns:
            column_widget = QWidget()
            column_layout = QVBoxLayout()
            column_layout.setContentsMargins(0, 0, 0, 0)
            column_layout.setSpacing(4)
            column_widget.setLayout(column_layout)

            for ch in column_channels:
                plot = self.channel_plots[ch]
                plot.set_filter(self.current_filter_sos)
                column_layout.addWidget(plot, 1)
                plot.show()

            self.channel_grid.addWidget(column_widget)

    @pyqtSlot(int, int)
    def on_gain_changed(self, channel: int, gain: int):
        """Propaga un cambio de ganancia (desde la pestaña "Avanzado" o
        desde una relectura de configuración real del ADS1298) al plot de
        ese canal y al combo de ganancia de la sidebar en "Monitor"."""
        plot = self.channel_plots.get(channel)
        if plot:
            plot.set_gain(gain)

        combo = self.channel_gain_combos.get(channel)
        if combo is not None:
            combo.blockSignals(True)
            combo.setCurrentText(str(gain))
            combo.blockSignals(False)

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def _update_connection_status(self, name: str, connected: bool, text: str):
        # Los CDC de datos (CDC2-5) ya no reflejan su estado de conexión en
        # cada SingleChannelPlot (ver set_leadoff): ese lugar ahora se usa
        # para el aviso de lead-off, y el estado agregado de conexión ya se
        # ve en el label de la barra superior ("Equipo desconectado"/verde/
        # ámbar/rojo).
        panel = self.panels.get(name)
        if panel:
            panel.set_connected(connected, text)
            return

        if self.rld_panel is not None and name == self.rld_panel.name:
            self.rld_panel.set_connected(connected, text)

    @pyqtSlot(str, str)
    def on_data_received(self, name: str, text: str):
        panel = self.panels.get(name)
        if panel:
            panel.append(text)

        if name == CDC_NAMES[1]:
            match = _REG_RESPONSE_RE.match(text.strip())
            if match:
                addr = int(match.group(1), 16)
                value = int(match.group(2), 16)
                self.config_panel.apply_register(addr, value)

    @pyqtSlot(str, bool, str)
    def on_status_changed(self, name: str, connected: bool, msg: str):
        panel = self.panels.get(name)
        if panel:
            panel.set_connected(connected, msg)

        if connected and name == CDC_NAMES[1]:
            # Recién conectado el canal de configuración: leemos el estado
            # real del ADS1298 en vez de asumir los valores por defecto.
            self.read_current_configuration()

        if not connected:
            conn = self.connections.pop(name, None)
            if isinstance(conn, ThreadConnection):
                conn.thread.quit()

        self._refresh_equipment_status_label()

    def read_current_configuration(self):
        """Lee los registros relevantes del ADS1298 por CDC1 (RREG) para que
        la pestaña de Configuración refleje cómo está configurado
        realmente el chip. Las respuestas se procesan a medida que llegan
        en on_data_received (formato "REG XX=YY")."""
        if not isinstance(self.connections.get(CDC_NAMES[1]), ThreadConnection):
            panel = self.panels.get(CDC_NAMES[1])
            if panel:
                panel.append("[ERROR] CDC1 no conectado, no se puede leer la configuración.")
            return

        for addr in CONFIG_READBACK_REGISTERS:
            self.send_to_cdc(CDC_NAMES[1], f"RREG {addr:02X}")

    @pyqtSlot(str, str)
    def send_to_cdc(self, name: str, text: str):
        conn = self.connections.get(name)
        if isinstance(conn, ThreadConnection):
            conn.write_line(text)
        else:
            panel = self.panels.get(name)
            if panel:
                panel.append("[ERROR] CDC no conectado.")

    @pyqtSlot(str)
    def send_config_command(self, command: str):
        self.send_to_cdc(CDC_NAMES[1], command)  # CDC_NAMES[1] == "CONFIG CDC1"

    def closeEvent(self, event):
        self.queue_timer.stop()
        self.plot_timer.stop()
        if self.recording_active:
            self._stop_recording()
        self.disconnect_all()
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
