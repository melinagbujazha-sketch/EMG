
"""
EMG Multichannel Serial Monitor - PyQt6

Requisitos:
    pip install pyqt6 pyserial pyqtgraph numpy

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

Pestaña de configuración:
    La pestaña "Configuración" arma y envía los comandos de la API de CDC1
    (GAIN, CHPOWER, MODE, RLD, RLD_CH, TESTSIG*, MUX, LOFF_*, START/STOP —
    ver la tabla completa en CLAUDE.md) usando controles de UI en vez de
    tipear el texto a mano. RREG/WREG crudos se siguen escribiendo a mano en
    el panel de texto de CDC1 (pestaña "Monitor").
"""

import math
import multiprocessing as mp
import queue
import re
import sys
import time
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports

from PyQt6.QtCore import QObject, QThread, QTimer, pyqtSignal, pyqtSlot, Qt
from PyQt6.QtGui import QKeySequence, QShortcut
from PyQt6.QtWidgets import (
    QApplication, QWidget, QMainWindow, QVBoxLayout, QHBoxLayout,
    QPushButton, QTextEdit, QLabel, QComboBox, QCheckBox, QLineEdit,
    QGroupBox, QGridLayout, QMessageBox, QSpinBox, QInputDialog,
    QTabWidget, QScrollArea,
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


def best_grid_shape(n: int):
    """Elige (filas, columnas) para acomodar n widgets ocupando todo el
    espacio disponible, priorizando (en orden): sin huecos vacíos, forma lo
    más cuadrada posible, y preferir más columnas que filas si hay empate
    (pantallas típicamente más anchas que altas)."""
    if n <= 0:
        return (1, 1)

    best_cols, best_score = 1, None
    for cols in range(1, n + 1):
        rows = -(-n // cols)  # ceil(n / cols)
        waste = rows * cols - n
        score = (waste, abs(rows - cols), -cols)
        if best_score is None or score < best_score:
            best_score = score
            best_cols = cols

    rows = -(-n // best_cols)
    return rows, best_cols


class SingleChannelPlot(QGroupBox):
    """Panel de visualización en tiempo real para UN canal físico del
    ADS1298. Mantiene un buffer circular y se redibuja con un timer
    separado (ver MainWindow.refresh_plots), desacoplando la tasa de
    llegada de datos (hasta 8 kSPS) de la tasa de refresco de pantalla.
    El eje Y se autoescala (pyqtgraph autorange) a la amplitud real de la
    señal; el usuario puede desactivarlo haciendo zoom/pan manual."""

    WINDOW_SECONDS = 4.0
    ASSUMED_SAMPLE_RATE = 8000  # cap de data rate del firmware, ver CLAUDE.md

    def __init__(self, channel: int):
        super().__init__(f"Canal {channel}")
        self.channel = channel
        self.gain = DEFAULT_GAIN
        self.setMinimumHeight(150)
        self.setMinimumWidth(200)

        self.status = QLabel("Desconectado")
        self.status.setStyleSheet("color: red; font-weight: bold;")

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("k")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("left", "mV")
        self.plot_widget.setLabel("bottom", "Muestras")
        self.plot_widget.setDownsampling(mode="peak")
        self.plot_widget.setClipToView(True)
        self.plot_widget.enableAutoRange(axis="y", enable=True)

        self.curve = self.plot_widget.plot(pen=pg.mkPen("y", width=1))

        buffer_size = int(self.WINDOW_SECONDS * self.ASSUMED_SAMPLE_RATE)
        self.buffer = np.zeros(buffer_size, dtype=np.int32)
        self.write_pos = 0
        self.filled = 0

        layout = QVBoxLayout()
        layout.addWidget(self.status)
        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

    def set_connected(self, connected: bool, msg: str):
        self.status.setText(msg)
        color = "green" if connected else "red"
        self.status.setStyleSheet(f"color: {color}; font-weight: bold;")

    def set_gain(self, gain: int):
        """Actualiza la ganancia usada para convertir a mV (llamado desde
        MainWindow cuando cambia el combo GAIN de este canal en la pestaña
        de Configuración)."""
        self.gain = gain

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
        self.curve.setData(code_to_millivolts(data, self.gain))


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
    def __init__(self):
        super().__init__()

        self.setWindowTitle("EMG Multichannel Acquisition - Multi CDC Monitor")
        self.resize(1400, 950)

        self.panels = {}          # CdcPanel (texto) para CDC0/CDC1
        self.channel_plots = {}   # SingleChannelPlot por canal físico (1-8)
        self.channel_checks = {}  # QCheckBox de "canal visible" (1-8)
        self.visible_channels = []
        self.rld_panel = None     # RldStatusPanel para CDC6
        self.connections = {}     # name -> ThreadConnection | ProcessConnection
        self.port_combos = {}

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

        title = QLabel("EMG MULTICHANNEL ACQUISITION - MULTI CDC MONITOR")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        root.addWidget(title)

        top = QHBoxLayout()

        self.refresh_button = QPushButton("Buscar puertos")
        self.autodetect_button = QPushButton("Auto-detectar CDC")
        self.connect_button = QPushButton("Conectar seleccionados")
        self.disconnect_button = QPushButton("Desconectar todo")
        self.fullscreen_button = QPushButton("Pantalla completa (F11)")

        self.baud = QSpinBox()
        self.baud.setRange(1200, 2000000)
        self.baud.setValue(115200)
        self.baud.setSingleStep(9600)

        top.addWidget(self.refresh_button)
        top.addWidget(self.autodetect_button)
        top.addWidget(QLabel("Baudrate:"))
        top.addWidget(self.baud)
        top.addWidget(self.connect_button)
        top.addWidget(self.disconnect_button)
        top.addStretch()
        top.addWidget(self.fullscreen_button)

        root.addLayout(top)

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
        root.addWidget(mapping_box)

        tabs = QTabWidget()
        root.addWidget(tabs)

        monitor_tab = QWidget()
        monitor_layout = QVBoxLayout()

        # Fila superior compacta: debug, configuración (texto crudo) y RLD.
        top_row = QHBoxLayout()
        for idx, name in ((0, CDC_NAMES[0]), (1, CDC_NAMES[1])):
            widget = CdcPanel(name)
            widget.send_requested.connect(self.send_to_cdc)
            self.panels[name] = widget
            top_row.addWidget(widget)

        self.rld_panel = RldStatusPanel(CDC_NAMES[6])
        top_row.addWidget(self.rld_panel)
        monitor_layout.addLayout(top_row)

        # Selector de qué canales físicos (1-8) se muestran en la grilla de
        # abajo; todos empiezan visibles.
        channel_select_box = QGroupBox("Canales visibles")
        channel_select_layout = QHBoxLayout()
        for ch in range(1, 9):
            check = QCheckBox(f"Canal {ch}")
            check.setChecked(True)
            check.toggled.connect(self._rebuild_channel_grid)
            channel_select_layout.addWidget(check)
            self.channel_checks[ch] = check
        channel_select_box.setLayout(channel_select_layout)
        monitor_layout.addWidget(channel_select_box)

        # Los 8 plots se crean siempre (siguen recibiendo y bufferizando
        # datos aunque estén ocultos); solo su presencia en channel_grid y
        # su redibujado en refresh_plots() depende de la selección.
        for ch in range(1, 9):
            self.channel_plots[ch] = SingleChannelPlot(ch)

        self.channel_grid_container = QWidget()
        self.channel_grid = QGridLayout()
        self.channel_grid_container.setLayout(self.channel_grid)
        monitor_layout.addWidget(self.channel_grid_container, stretch=1)

        monitor_tab.setLayout(monitor_layout)

        monitor_scroll = QScrollArea()
        monitor_scroll.setWidgetResizable(True)
        monitor_scroll.setWidget(monitor_tab)
        tabs.addTab(monitor_scroll, "Monitor")

        self._rebuild_channel_grid()

        self.config_panel = ConfigPanel()
        self.config_panel.command_requested.connect(self.send_config_command)
        self.config_panel.gain_changed.connect(self.on_gain_changed)
        self.config_panel.reread_requested.connect(self.read_current_configuration)

        config_scroll = QScrollArea()
        config_scroll.setWidgetResizable(True)
        config_scroll.setWidget(self.config_panel)
        tabs.addTab(config_scroll, "Configuración")

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.autodetect_button.clicked.connect(self.auto_detect_ports)
        self.connect_button.clicked.connect(self.connect_selected)
        self.disconnect_button.clicked.connect(self.disconnect_all)
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

    def auto_detect_ports(self):
        """Escanea los puertos COM, abre brevemente cada uno perteneciente al
        dispositivo EMG (VID 0xCAFE) y lee el banner de bienvenida del
        firmware para determinar con certeza a qué CDC pertenece."""
        self.refresh_ports()

        grouped = group_ports_by_serial()

        if not grouped:
            QMessageBox.warning(
                self,
                "Auto-detección",
                "No se encontró ningún dispositivo EMG (VID 0xCAFE) en los "
                "puertos COM.\n\nVerifica que la Pico esté conectada y que "
                "el firmware haya enumerado correctamente (revisa el "
                "Administrador de dispositivos de Windows)."
            )
            return

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
                return

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

        message = (
            f"Placa (serie {serial_number}): "
            f"se asignaron {assigned}/{len(CDC_NAMES)} interfaces CDC."
        )
        if missing:
            message += "\nNo se detectaron: " + ", ".join(missing)
        if skipped:
            message += (
                "\nPuertos omitidos por estar ya conectados en este "
                "programa: " + ", ".join(skipped)
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
                elif tag == "frame":
                    _, values_a, values_b = msg
                    channel_a, channel_b = self.pair_channels.get(name, (None, None))
                    if channel_a is not None:
                        self.channel_plots[channel_a].append_batch(values_a)
                        self.channel_plots[channel_b].append_batch(values_b)
                elif tag == "rld":
                    _, values_a, values_b = msg
                    if self.rld_panel is not None and len(values_a) and len(values_b):
                        self.rld_panel.update_status(int(values_a[-1]), int(values_b[-1]))

    def refresh_plots(self):
        # Solo redibuja los canales que están visibles: los ocultos siguen
        # bufferizando datos (poll_process_queues) pero no gastan tiempo de
        # renderizado hasta que se vuelvan a mostrar.
        for ch in self.visible_channels:
            self.channel_plots[ch].refresh()

    def _rebuild_channel_grid(self):
        """Recalcula qué canales están tildados en "Canales visibles" y
        reparte el espacio de channel_grid_container entre ellos (ver
        best_grid_shape): 1 canal ocupa toda el área, 4 canales la dividen
        en 2x2, etc."""
        while self.channel_grid.count():
            item = self.channel_grid.takeAt(0)
            widget = item.widget()
            if widget:
                widget.hide()

        self.visible_channels = [ch for ch in range(1, 9) if self.channel_checks[ch].isChecked()]
        if not self.visible_channels:
            return

        _rows, cols = best_grid_shape(len(self.visible_channels))

        for i, ch in enumerate(self.visible_channels):
            row, col = divmod(i, cols)
            self.channel_grid.addWidget(self.channel_plots[ch], row, col)
            self.channel_plots[ch].show()

    @pyqtSlot(int, int)
    def on_gain_changed(self, channel: int, gain: int):
        """Propaga un cambio de ganancia (desde la pestaña de Configuración)
        al plot de ese canal, para que la conversión a mV use el valor
        correcto."""
        plot = self.channel_plots.get(channel)
        if plot:
            plot.set_gain(gain)

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def _update_connection_status(self, name: str, connected: bool, text: str):
        panel = self.panels.get(name)
        if panel:
            panel.set_connected(connected, text)
            return

        pair = self.pair_channels.get(name)
        if pair:
            for ch in pair:
                self.channel_plots[ch].set_connected(connected, text)
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
        self.disconnect_all()
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
