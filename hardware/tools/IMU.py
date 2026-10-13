"""Desktop monitor for the Landing Gear System's USB serial interface."""

import math
import queue
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

import serial
from serial.tools import list_ports


def parse_imu_data(data):
    """Return six finite values from an S...E frame, or None for other lines."""
    if not data.startswith("S") or not data.endswith("E"):
        return None
    try:
        values = tuple(map(float, data[1:-1].split("/")))
    except ValueError:
        return None
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        return None
    return values


def moving_average_filter(data_queue, new_value, window_size=10):
    """Average at most window_size samples; this is display smoothing only."""
    data_queue.append(new_value)
    while len(data_queue) > window_size:
        data_queue.popleft()
    return sum(data_queue) / len(data_queue)


def read_serial_lines(port, stop_event, events):
    """Read complete lines in a worker. All Tk operations stay on the UI thread."""
    pending = bytearray()
    try:
        while not stop_event.is_set():
            chunk = port.read(min(port.in_waiting, 4096) or 1)
            if not chunk:
                continue
            pending.extend(chunk)
            while b"\n" in pending:
                line, _, remainder = pending.partition(b"\n")
                pending = bytearray(remainder)
                # Bound the queue if the UI falls behind; favour current readings.
                if events.full():
                    try:
                        events.get_nowait()
                    except queue.Empty:
                        pass
                events.put_nowait(("line", line.decode("utf-8", errors="replace").strip()))
            if len(pending) > 8192:
                raise ValueError("Serial line exceeded 8192 bytes; reconnect to resynchronise.")
    except (serial.SerialException, OSError, ValueError) as error:
        if not stop_event.is_set():
            if events.full():
                try:
                    events.get_nowait()
                except queue.Empty:
                    pass
            events.put_nowait(("error", str(error)))


class IMUApp:
    """Show firmware reports or raw IMU data, and send manual demo commands."""

    def __init__(self, root):
        self.root = root
        self.root.title("Landing Gear System Monitor")
        self.root.geometry("850x760")
        self.root.minsize(720, 650)
        self.ser = None
        self.reading_thread = None
        self.stop_event = threading.Event()
        self.events = queue.Queue(maxsize=500)
        self.last_data_time = None
        self.last_report_time = None
        self.closed = False
        self.port_var = tk.StringVar()
        self.mode_var = tk.StringVar(value="Reports")
        self.status_var = tk.StringVar(value="Disconnected - select a port and connect.")
        self.data_status_var = tk.StringVar(value="No raw IMU frames received.")
        self.queues = [deque() for _ in range(6)]
        self.raw_vars = [tk.StringVar(value="--") for _ in range(6)]
        self.average_vars = [tk.StringVar(value="--") for _ in range(6)]
        self.report_vars = {key: tk.StringVar(value="--") for key in
                            ("Sequence", "Readiness", "Attitude", "Range", "Stability", "Actuators")}
        self.warning_var = tk.StringVar(value="No current report.")

        # Grid layout grows with the window instead of clipping long messages.
        main = ttk.Frame(root, padding=12)
        main.pack(fill="both", expand=True)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(6, weight=1)
        connection = ttk.Frame(main)
        connection.grid(row=0, column=0, sticky="ew")
        ttk.Label(connection, text="Serial port:").pack(side="left")
        self.port_box = ttk.Combobox(connection, textvariable=self.port_var, width=18)
        self.port_box.pack(side="left", padx=6)
        ttk.Button(connection, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(connection, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=6)
        ttk.Label(connection, text="115200 baud").pack(side="left")
        ttk.Label(main, textvariable=self.status_var, wraplength=800).grid(
            row=1, column=0, sticky="w", pady=(6, 10))

        controls = ttk.Frame(main)
        controls.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        self.command_buttons = []
        for label, command in (("Deploy (D)", "D"), ("Stow (S)", "S")):
            button = ttk.Button(controls, text=label, command=lambda c=command: self.send_command(c))
            button.pack(side="left", padx=(0, 6))
            self.command_buttons.append(button)
        ttk.Label(controls, text="Display:").pack(side="left", padx=(12, 6))
        mode_box = ttk.Combobox(controls, textvariable=self.mode_var, state="readonly",
                               values=("Reports", "Raw IMU"), width=12)
        mode_box.pack(side="left")
        mode_box.bind("<<ComboboxSelected>>", self.change_mode)
        ttk.Label(controls, text="Virtual actuators only").pack(side="left", padx=12)

        report = ttk.LabelFrame(main, text="Landing report (Reports mode)", padding=8)
        report.grid(row=3, column=0, sticky="ew")
        report.columnconfigure(1, weight=1)
        for row, (name, variable) in enumerate(self.report_vars.items()):
            ttk.Label(report, text=name + ":", width=12).grid(row=row, column=0, sticky="nw", pady=2)
            ttk.Label(report, textvariable=variable, wraplength=650).grid(
                row=row, column=1, sticky="w", pady=2)
        ttk.Label(report, textvariable=self.warning_var, wraplength=760).grid(
            row=6, column=0, columnspan=2, sticky="w", pady=(6, 0))

        raw = ttk.LabelFrame(main, text="Raw IMU mode | Acceleration: g | Gyroscope: deg/s", padding=8)
        raw.grid(row=4, column=0, sticky="ew", pady=10)
        for column, label in enumerate(("", "Acc X", "Acc Y", "Acc Z", "Gyro X", "Gyro Y", "Gyro Z")):
            ttk.Label(raw, text=label).grid(row=0, column=column, padx=8)
            raw.columnconfigure(column, weight=1)
        for row, (label, variables) in enumerate((("Raw", self.raw_vars),
                                                 ("Mean (10 samples)", self.average_vars)), start=1):
            ttk.Label(raw, text=label).grid(row=row, column=0, sticky="w")
            for column, variable in enumerate(variables, start=1):
                ttk.Label(raw, textvariable=variable).grid(row=row, column=column, padx=8)
        ttk.Label(raw, textvariable=self.data_status_var).grid(row=3, column=0, columnspan=7, sticky="w", pady=4)

        ttk.Label(main, text="Serial messages (calibration, reports, commands and warnings)").grid(
            row=5, column=0, sticky="w")
        self.log = ScrolledText(main, height=10, wrap="word", state="disabled")
        self.log.grid(row=6, column=0, sticky="nsew", pady=6)
        self.refresh_ports()
        self.set_connected_controls(False)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.poll_id = self.root.after(50, self.poll_events)

    def refresh_ports(self):
        ports = [port.device for port in list_ports.comports()]
        self.port_box["values"] = ports
        if not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    def set_connected_controls(self, connected):
        self.connect_button.config(text="Disconnect" if connected else "Connect")
        self.port_box.config(state="disabled" if connected else "normal")
        for button in self.command_buttons:
            button.config(state="normal" if connected else "disabled")

    def toggle_connection(self):
        if self.ser is not None:
            self.disconnect()
        else:
            self.connect_serial()

    def connect_serial(self):
        port_name = self.port_var.get().strip()
        if not port_name:
            self.status_var.set("Select or enter a serial port first.")
            return
        try:
            self.ser = serial.Serial(port_name, 115200, timeout=0.2, write_timeout=0.5)
        except (serial.SerialException, OSError, ValueError) as error:
            self.status_var.set(f"Connection failed: {error}")
            self.ser = None
            return
        self.stop_event = threading.Event()
        self.events = queue.Queue(maxsize=500)
        self.last_data_time = None
        self.last_report_time = None
        for samples in self.queues:
            samples.clear()
        for variable in self.raw_vars + self.average_vars + list(self.report_vars.values()):
            variable.set("--")
        self.warning_var.set("Waiting for calibration and the first report.")
        self.data_status_var.set("Waiting for raw frames; select Raw IMU to request them.")
        self.status_var.set(f"Connected to {port_name}. Keep the IMU level during startup calibration.")
        self.set_connected_controls(True)
        self.reading_thread = threading.Thread(target=read_serial_lines,
                                              args=(self.ser, self.stop_event, self.events), daemon=True)
        self.reading_thread.start()
        self.change_mode()

    def disconnect(self, status="Disconnected."):
        self.stop_event.set()
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None
        if self.reading_thread is not None:
            self.reading_thread.join(timeout=0.5)
            self.reading_thread = None
        self.events = queue.Queue(maxsize=500)
        self.set_connected_controls(False)
        self.status_var.set(status)
        self.data_status_var.set("Disconnected; displayed raw values are historical.")
        self.warning_var.set("Disconnected; displayed report is historical.")

    def send_command(self, command):
        if self.ser is None:
            return
        try:
            self.ser.write((command + "\n").encode("ascii"))
        except (serial.SerialException, OSError) as error:
            self.disconnect(f"Serial write failed: {error}")

    def change_mode(self, _event=None):
        self.last_report_time = None
        self.send_command("V" if self.mode_var.get() == "Raw IMU" else "M")
        for variable in self.report_vars.values():
            variable.set("--")
        self.warning_var.set("Reports paused in Raw IMU mode." if self.mode_var.get() == "Raw IMU"
                             else "Waiting for the next report.")

    def append_log(self, line):
        self.log.config(state="normal")
        self.log.insert("end", line + "\n")
        # Retain recent context without growing memory for the whole session.
        if int(self.log.index("end-1c").split(".")[0]) > 300:
            self.log.delete("1.0", "2.0")
        self.log.see("end")
        self.log.config(state="disabled")

    def handle_line(self, line):
        values = parse_imu_data(line)
        if values is not None:
            self.last_data_time = time.monotonic()
            for index, value in enumerate(values):
                self.raw_vars[index].set(f"{value:.2f}")
                average = moving_average_filter(self.queues[index], value)
                self.average_vars[index].set(f"{average:.2f}")
            self.data_status_var.set("Receiving raw frames (20 Hz); mean is display smoothing only.")
            return
        if not line:
            return
        self.append_log(line)
        # Opening the port can reset an Uno. Reapply the selected display mode
        # when setup announces completion; no firmware-default edit is needed.
        if line.startswith("=== LANDING GEAR DEMO"):
            self.change_mode()
        elif line.startswith("--- Landing Gear Demo"):
            self.last_report_time = time.monotonic()
            self.warning_var.set("No warnings in this report.")
        elif ":" in line:
            name, value = line.split(":", 1)
            name = name.strip()
            if name in self.report_vars:
                self.report_vars[name].set(value.strip())
            elif name in ("IMU warning", "Gear warning"):
                previous = self.warning_var.get()
                warning = name + ": " + value.strip()
                self.warning_var.set(previous + "\n" + warning if previous.startswith(("IMU warning", "Gear warning"))
                                     else warning)

    def poll_events(self):
        # The worker only queues text. GUI updates and commands happen here.
        for _ in range(100):
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "error":
                self.disconnect(f"Serial read failed: {value}. Reconnect when ready.")
                break
            self.handle_line(value)
        if self.ser is not None and self.last_data_time is not None:
            if time.monotonic() - self.last_data_time > 2:
                self.data_status_var.set("No raw frames for 2 seconds; displayed values are historical.")
        if self.ser is not None and self.mode_var.get() == "Reports" and self.last_report_time is not None:
            if time.monotonic() - self.last_report_time > 2:
                self.warning_var.set("No report for 2 seconds; displayed report is historical.")
        if not self.closed:
            self.poll_id = self.root.after(50, self.poll_events)

    def on_close(self):
        self.closed = True
        self.root.after_cancel(self.poll_id)
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = IMUApp(root)
    root.mainloop()
