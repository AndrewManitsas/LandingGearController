"""Offline checks for the desktop monitor; no serial hardware is required."""

import queue
import threading
import tkinter as tk
import unittest
from collections import deque
from unittest.mock import Mock, patch

import serial

from IMU import IMUApp, moving_average_filter, parse_imu_data, read_serial_lines


class ParserTests(unittest.TestCase):
    def test_six_signed_values(self):
        self.assertEqual(parse_imu_data("S0.05/0.11/1.01/-1.38/0.44/0.31E"),
                         (0.05, 0.11, 1.01, -1.38, 0.44, 0.31))

    def test_invalid_frames(self):
        for frame in ("", "Readiness: READY", "S1/2E", "S1/2/3/4/5/6/7E",
                      "S1/2/3/4/5/noE", "S1/2/3/4/5/nanE", "Sinf/2/3/4/5/6E"):
            with self.subTest(frame=frame):
                self.assertIsNone(parse_imu_data(frame))

    def test_average_uses_latest_ten_samples(self):
        samples = deque()
        for value in range(1, 12):
            mean = moving_average_filter(samples, value)
        self.assertEqual(list(samples), list(range(2, 12)))
        self.assertEqual(mean, 6.5)


class SerialReaderTests(unittest.TestCase):
    def test_fragmented_frames_and_multiple_lines(self):
        stop = threading.Event()
        events = queue.Queue(maxsize=10)
        chunks = deque([b"S1/2/", b"3/4/5/6E\r\n  Readiness: READY\n", b"bad\xff\n"])
        port = Mock(in_waiting=10)

        def read(_count):
            if chunks:
                return chunks.popleft()
            stop.set()
            return b""

        port.read.side_effect = read
        read_serial_lines(port, stop, events)
        self.assertEqual(events.get_nowait(), ("line", "S1/2/3/4/5/6E"))
        self.assertEqual(events.get_nowait(), ("line", "Readiness: READY"))
        self.assertEqual(events.get_nowait()[1], "bad\ufffd")

    def test_read_error_reaches_ui(self):
        events = queue.Queue(maxsize=10)
        port = Mock(in_waiting=0)
        port.read.side_effect = serial.SerialException("unplugged")
        read_serial_lines(port, threading.Event(), events)
        self.assertEqual(events.get_nowait(), ("error", "unplugged"))

    def test_bounded_queue_keeps_latest_line(self):
        stop = threading.Event()
        events = queue.Queue(maxsize=1)
        port = Mock(in_waiting=10)
        chunks = deque([b"old\nnew\n"])

        def read(_count):
            if chunks:
                return chunks.popleft()
            stop.set()
            return b""

        port.read.side_effect = read
        read_serial_lines(port, stop, events)
        self.assertEqual(events.get_nowait(), ("line", "new"))


class ViewerTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Tk display unavailable: {error}")
        self.root.withdraw()
        with patch("IMU.list_ports.comports", return_value=[]):
            self.app = IMUApp(self.root)
        self.port = Mock()
        self.serial_patch = patch("IMU.serial.Serial", return_value=self.port)
        self.thread_patch = patch("IMU.threading.Thread")
        self.serial_patch.start()
        self.thread_patch.start()
        self.addCleanup(self.serial_patch.stop)
        self.addCleanup(self.thread_patch.stop)

    def tearDown(self):
        if hasattr(self, "app"):
            self.app.on_close()

    def connect(self):
        self.app.port_var.set("TEST_PORT")
        self.app.connect_serial()

    def test_mode_is_reapplied_after_startup_reset(self):
        self.app.mode_var.set("Raw IMU")
        self.connect()
        self.port.write.assert_called_with(b"V\n")
        self.port.write.reset_mock()
        self.app.handle_line("=== LANDING GEAR DEMO | VIRTUAL ACTUATORS ===")
        self.port.write.assert_called_once_with(b"V\n")

    def test_reports_and_two_warnings(self):
        for line in ("--- Landing Gear Demo | 30 s ---", "  Sequence : DEPLOYED",
                     "  Readiness : UNKNOWN - invalid sensor data",
                     "  IMU warning: Invalid sample.", "  Gear warning: Gear held deployed."):
            self.app.handle_line(line)
        self.assertEqual(self.app.report_vars["Sequence"].get(), "DEPLOYED")
        self.assertIn("UNKNOWN", self.app.report_vars["Readiness"].get())
        self.assertIn("IMU warning", self.app.warning_var.get())
        self.assertIn("Gear warning", self.app.warning_var.get())
        self.app.handle_line("--- Landing Gear Demo | 31 s ---")
        self.assertEqual(self.app.warning_var.get(), "No warnings in this report.")

    def test_commands_and_reconnect(self):
        self.connect()
        self.app.send_command("D")
        self.port.write.assert_called_with(b"D\n")
        self.app.send_command("S")
        self.port.write.assert_called_with(b"S\n")
        self.app.disconnect()
        self.port.close.assert_called_once()
        self.assertIsNone(self.app.ser)
        self.connect()
        self.assertIs(self.app.ser, self.port)
        self.assertTrue(self.app.reading_thread.start.called)

    def test_invalid_data_cannot_update_values(self):
        self.app.handle_line("S1/2/3/4/5/6E")
        self.app.handle_line("Snan/2/3/4/5/6E")
        self.assertEqual(self.app.raw_vars[0].get(), "1.00")
        self.assertEqual(len(self.app.queues[0]), 1)

    def test_missing_reports_are_marked_historical(self):
        self.connect()
        with patch("IMU.time.monotonic", return_value=1):
            self.app.handle_line("--- Landing Gear Demo | 1 s ---")
        with patch("IMU.time.monotonic", return_value=4):
            self.app.poll_events()
        self.assertIn("historical", self.app.warning_var.get())
        self.app.handle_line("--- Landing Gear Demo | 4 s ---")
        self.assertEqual(self.app.warning_var.get(), "No warnings in this report.")

    def test_read_failure_disconnects_and_marks_values_historical(self):
        self.connect()
        self.app.events.put(("error", "unplugged"))
        self.app.poll_events()
        self.assertIsNone(self.app.ser)
        self.assertIn("unplugged", self.app.status_var.get())
        self.assertIn("historical", self.app.warning_var.get())


if __name__ == "__main__":
    unittest.main()
