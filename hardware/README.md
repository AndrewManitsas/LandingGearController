# UAV Landing Gear System: Hardware Bench Prototype

A landing-assistance controller for a UAV, built around an Arduino Uno, an MPU6500 IMU and a VL53L0X time-of-flight distance sensor. The firmware estimates platform attitude, evaluates a small set of landing-readiness rules, and generates virtual stabilizer and landing gear commands.

The hardware demo implements a subset of the [simulation concept](../simulation/README.md) using physical sensors. The current demonstration uses one microcontroller and one range sensor, with manual deployment and displayed actuator commands.

---

## 1. Project Information

- **Institution:** Department of Electrical and Computer Engineering, Aristotle University of Thessaloniki
- **Course:** UAV01 - Sensor Systems for UAVs
- **Authors:** Andreas Manitsas (`amanitsb@ece.auth.gr`), Maria Vrana (`mvranaa@ece.auth.gr`)
- **Implementation:** Hardware bench prototype with virtual actuators
- **Target stack:** Arduino Uno, Arduino IDE, FastIMU, Pololu VL53L0X, Python 3, Tkinter and pyserial

---

## 2. High-Level Concept

```text
               Arduino Uno
                    |
        +-----------+-----------+
        |                       |
        v                       v
  MPU6500 IMU             VL53L0X ToF
  acceleration + gyro     target distance + status
        |                       |
        v                       |
  Pitch / roll estimate --------+
        |                       |
        v                       v
  Virtual stabilizer      Landing-readiness checks
  pitch correction        range, stability, tilt, freshness
                                |
                                v
                          Manual deployment request
                          accepted only while READY
                                |
                                v
                          Latched virtual gear command
                          explicit stow or reset
        |                       |
        +-----------+-----------+
                    v
             USB serial monitoring
             Arduino Serial Monitor / Python app
```

Sensing, decision-making and control run on the Uno. The ESP32 is not required for the current implementation. No ROS, Gazebo, terrain classifier or physical servo control is involved in this build.

---

## 3. Hardware and Connections

### 3.1 Current Hardware

| Component | Role |
| :--- | :--- |
| Arduino Uno | Sensor acquisition, attitude estimation, readiness checks and virtual commands |
| GY-6500 / MPU6500 IMU | Acceleration and angular-rate measurements |
| One VL53L0X breakout | Sensor-to-target distance measurement |
| Breadboard and jumper wires | Shared power, ground and I2C connections |
| USB connection to a computer | Board power, firmware upload and monitoring |

### 3.2 Wiring

The following is the connected one-ToF setup. The IMU is configured at `0x68` and the ToF uses its default address, `0x29`.

| Uno | MPU6500 module | VL53L0X breakout |
| :--- | :--- | :--- |
| 3.3 V | Module power input | VIN |
| GND | GND | GND |
| A4 / SDA | SDA | SDA |
| A5 / SCL | SCL | SCL |

The ToF module is the regulated breakout identified in the [OSOYOO VL53L0X lesson](https://osoyoo.com/2019/04/21/arduino-lesson-vl53l0x-time-of-flight-distance-sensor/). Its VDD, XSHUT and GPIO1 pins are left unconnected. Keep the existing IMU address selection and wiring; other breakout variants may need a different arrangement.

Both SG90 servos remain disconnected. The firmware does not drive servo pins, and no external servo supply is part of this setup. Additional ToFs, HY-SRF05 ultrasonic sensors, GY-271 magnetometers and the ESP-WROOM-32 are not used by the current demo.

The prepared three-ToF configuration is disabled (`TOF_SENSOR_COUNT = 1`). Enabling it requires a suitable power arrangement, shared-bus voltage compatibility, individual XSHUT wiring and fixed sensor geometry. It must not be treated as a working three-sensor terrain estimator.

---

## 4. Firmware Behaviour

### 4.1 Attitude and Stabilizer Command

Startup calibration requires the IMU to remain level and stationary, with +Z pointing upward. Calibration is stored in RAM and repeats after reset; startup takes approximately 20 seconds plus sensor initialization.

The firmware samples at nominal 100 Hz. A complementary filter combines accelerometer tilt with gyro integration using measured elapsed time and a 0.50-second time constant. It reports pitch and roll in degrees. FastIMU measurements are acceleration in **g** and angular rate in **degrees per second**.

The virtual stabilizer command is:

```text
90 degrees + (target pitch - measured pitch)
```

The target pitch is zero, the pitch deadband is +/-1 degree, and the command is limited to 65-115 degrees. Invalid numeric IMU samples neutralize the stabilizer at 90 degrees without clearing deployed gear.

### 4.2 Landing Readiness

| Check | Demo setting |
| :--- | :--- |
| Sensor-to-target distance | 100-500 mm, inclusive |
| Absolute pitch and roll | At most 10 degrees each |
| Stability window | 10 completed, valid ToF measurements |
| Distance spread across the window | At most 20 mm; all samples inside the distance limits |
| Range freshness | At most 250 ms since the last result |
| Qualification time | All checks continuously satisfied for one second after the window is complete |

The ToF runs with a nominal 50 ms period. Polling an existing result does not count it as a new sample. Invalid or stale readings and I2C errors reset the stability window and qualification. Failed tilt, distance or stability checks restart qualification.

Readiness is displayed as `UNKNOWN` while data is missing, invalid or still qualifying, `UNSAFE` when a measured condition fails a rule, and `READY` when the bench checks pass. These thresholds are configurable demonstration settings, not aircraft operating limits.

### 4.3 Landing Sequence and Commands

| Command | Behaviour |
| :--- | :--- |
| `D` | Deploy the virtual gear to 90 degrees while READY. An early request is rejected, not queued. |
| `S` | Explicitly stow the virtual gear at 0 degrees, including during sensor faults. |
| `M` | Select readable serial reports at 2 Hz (default). |
| `V` | Select raw six-value IMU frames at 20 Hz. |

Lowercase commands work too; line endings are ignored. Deployment stays latched when distance, tilt or sensor validity changes. The report shows a warning and the current readiness reason. Repeated `D` leaves deployment latched. Only `S` or board reset clears it; reset starts stowed.

The sequence display distinguishes `STOWED`, `READY TO DEPLOY` and `DEPLOYED`. Readiness is evaluated independently, so gear can be deployed while current readiness is UNKNOWN or UNSAFE. All actuator states describe commands, not measured physical positions.

---

## 5. Repository Layout

```text
LandingGearController/hardware/
|-- firmware/
|   `-- LandingGearController/
|       `-- LandingGearController.ino  # Uno sensor and control firmware
|-- hardware/tools/
|   |-- IMU.py                        # Desktop serial monitor and IMU viewer
|   |-- requirements.txt              # Python dependency: pyserial
|   `-- test_IMU.py                   # Parser, serial and viewer checks
`-- README.md                         # Project overview and setup
```

---

## 6. Build and Execution Workflow

### 6.1 Firmware Upload

1. Connect the Uno, IMU and single ToF as described above.
2. Open `hardware/firmware/LandingGearController/LandingGearController.ino` in Arduino IDE.
3. Install **FastIMU** and **VL53L0X by Pololu** through Library Manager. The ToF driver used here is Pololu version 1.3.1.
4. Select **Arduino Uno** and the board's serial port, then compile and upload.
5. Keep the IMU level and stationary throughout startup calibration.

### 6.2 Arduino Serial Monitor

Open Serial Monitor at **115200 baud**. Readable reports update twice per second and include sequence, readiness and reason, attitude, range/status, stability progress, actuator angles and applicable warnings. Commands receive acknowledgements or a rejection reason.

For a demonstration, aim the ToF at a steady surface 200-300 mm away with the IMU level. Wait for READY, send `D`, then move or tilt the setup to show the warning while gear remains deployed. Send `S` to stow.

### 6.3 Python Desktop Monitor

Use Python 3 with Tkinter on the computer that can access the Uno's serial port. Close Arduino Serial Monitor first; only one application can own the port.

```text
py -3.15 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r hardware/tools/requirements.txt
.\.venv\Scripts\python.exe hardware/tools/IMU.py
```

Run the commands from the repository root on Windows. The first two commands
are one-time setup (replace `-3.15` with your installed Python version if needed).
On subsequent runs, use `.\.venv\Scripts\python.exe hardware/tools/IMU.py`.

Select a serial port (or type its name), then click **Connect**. Refresh rescans available ports. Opening the port can reset the Uno: keep the IMU level during calibration. The app reapplies the selected display mode when firmware startup completes.

- **Reports:** Displays the landing report and recent serial messages. Deploy and Stow buttons send the same manual commands as Serial Monitor; the firmware decides whether deployment is allowed.
- **Raw IMU:** Requests viewer frames and displays six raw measurements plus a ten-sample moving average. The smoothing affects only the display, not firmware decisions. Landing reports are paused in this mode.
- **Disconnect:** Closes the port. After a connection error, reconnect manually once the problem is resolved. Old values are marked historical rather than presented as current data.

The viewer protocol is `S<ax>/<ay>/<az>/<gx>/<gy>/<gz>E`, one frame per line. Non-finite and malformed frames are rejected. A Windows COM port is normally accessed from Windows Python, not automatically from WSL.

### 6.4 Software Checks

```text
.\.venv\Scripts\python.exe -m unittest discover -s hardware/tools -p "test_*.py"
```

The tests cover serial framing and finite-value validation, display averaging, fragmented input, serial errors and display/command integration using a mocked serial connection. They do not require connected hardware.

---

## 7. Demonstration Checks and Limits

| Scenario | Expected behaviour |
| :--- | :--- |
| Level IMU, stable target inside the range window | READY after ten readings and one second of qualification |
| Deploy before READY | Rejected with a reason; request is not saved |
| Deploy while READY | Gear command becomes DEPLOYED at 90 degrees |
| Change distance or tilt after deployment | Readiness changes; deployed command stays latched with a warning |
| Remove the target or interrupt ToF communication | UNKNOWN, stability window reset; deployed command retained |
| Restore acceptable readings | Readiness requires fresh qualification; no automatic deployment |
| Send S | Gear command becomes STOWED at 0 degrees |
| Tilt the IMU about its Y axis | Stabilizer command reacts to pitch and respects its limits |

The connected bench setup and serial reports have been exercised on hardware. The system has no servo-position feedback, touchdown detection, yaw estimate, aerodynamic feedback or automatic landing control. One ToF measures sensor-to-target range, not terrain slope, surface material or tilt-corrected vertical altitude.

Translational acceleration can distort the gravity estimate, and the simple gyro integration is intended for moderate bench tilts. Numeric IMU validation cannot detect all plausible stale readings. ToF validation uses the raw device status, not the full ST software quality assessment. Hysteresis is not implemented; the one-second qualification delays re-entry into READY.

---

## 8. License

This project is licensed under the GNU General Public License v3.0. See [LICENSE](../LICENSE).


Run upload and monitor commands from the repository root in a Windows checkout. The folder tree above describes the hardware subtree; LICENSE and the project overview live at the repository root.

[Back to project overview](../README.md)
