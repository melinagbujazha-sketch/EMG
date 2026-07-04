
"""
EMG Multichannel Serial Monitor - PyQt6

Requisitos:
    pip install pyqt6 pyserial

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
"""

import re
import sys
import time
from dataclasses import dataclass

import serial
import serial.tools.list_ports

from PyQt6.QtCore import QObject, QThread, pyqtSignal, pyqtSlot, Qt
from PyQt6.QtWidgets import (
    QApplication, QWidget, QMainWindow, QVBoxLayout, QHBoxLayout,
    QPushButton, QTextEdit, QLabel, QComboBox, QLineEdit, QGroupBox,
    QGridLayout, QMessageBox, QSpinBox, QInputDialog
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


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("EMG Multichannel Acquisition - Multi CDC Monitor")
        self.resize(1350, 900)

        self.panels = {}
        self.workers = {}
        self.threads = {}
        self.port_combos = {}

        main = QWidget()
        self.setCentralWidget(main)

        root = QVBoxLayout()

        title = QLabel("EMG MULTICHANNEL ACQUISITION - MULTI CDC MONITOR")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        root.addWidget(title)

        top = QHBoxLayout()

        self.refresh_button = QPushButton("Buscar puertos")
        self.autodetect_button = QPushButton("Auto-detectar CDC")
        self.connect_button = QPushButton("Conectar seleccionados")
        self.disconnect_button = QPushButton("Desconectar todo")

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

        grid = QGridLayout()

        for idx, name in enumerate(CDC_NAMES):
            panel = CdcPanel(name)
            panel.send_requested.connect(self.send_to_cdc)
            self.panels[name] = panel

            row = idx // 2
            col = idx % 2
            grid.addWidget(panel, row, col)

        root.addLayout(grid)

        main.setLayout(root)

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.autodetect_button.clicked.connect(self.auto_detect_ports)
        self.connect_button.clicked.connect(self.connect_selected)
        self.disconnect_button.clicked.connect(self.disconnect_all)

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
        busy_ports = set()
        for name, worker in self.workers.items():
            busy_ports.add(worker.port)

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

        for name, combo in self.port_combos.items():
            port = combo.currentData()
            if port and name not in self.workers:
                self.start_worker(name, port, baudrate)

    def start_worker(self, name: str, port: str, baudrate: int):
        thread = QThread()
        worker = SerialWorker(name, port, baudrate)

        worker.moveToThread(thread)

        thread.started.connect(worker.run)
        worker.data_received.connect(self.on_data_received)
        worker.status_changed.connect(self.on_status_changed)

        self.workers[name] = worker
        self.threads[name] = thread

        thread.start()

    def disconnect_all(self):
        for name in list(self.workers.keys()):
            self.disconnect_one(name)

    def disconnect_one(self, name: str):
        worker = self.workers.get(name)
        thread = self.threads.get(name)

        if worker:
            worker.stop()

        if thread:
            thread.quit()
            thread.wait(1000)

        self.workers.pop(name, None)
        self.threads.pop(name, None)

    @pyqtSlot(str, str)
    def on_data_received(self, name: str, text: str):
        panel = self.panels.get(name)
        if panel:
            panel.append(text)

    @pyqtSlot(str, bool, str)
    def on_status_changed(self, name: str, connected: bool, msg: str):
        panel = self.panels.get(name)
        if panel:
            panel.set_connected(connected, msg)

        if not connected:
            self.workers.pop(name, None)
            thread = self.threads.pop(name, None)
            if thread:
                thread.quit()

    @pyqtSlot(str, str)
    def send_to_cdc(self, name: str, text: str):
        worker = self.workers.get(name)
        if worker:
            worker.write_line(text)
        else:
            self.panels[name].append("[ERROR] CDC no conectado.")

    def closeEvent(self, event):
        self.disconnect_all()
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
