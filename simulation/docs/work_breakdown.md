# Work Breakdown Structure

## 1. Project Phases and Tasks

### Phase 1: Virtual Rig & Sensor Array [COMPLETED]
* **Task 1.1: Airframe & Actuator Modeling** [x]
  * Model fixed-wing airframe in URDF with revolute joints for T-tail stabilator ($\pm 25^\circ$) and retractable main landing gear ($0^\circ\text{--}90^\circ$).
* **Task 1.2: 3x VL53L0X ToF Array Integration** [x]
  * Mount three downward optical rangefinder plugins in an equilateral triangular configuration on the ventral sensor bay.
* **Task 1.3: Downward Sonar Integration** [x]
  * Mount central ultrasonic rangefinder plugin with conical beam profile on the forward belly.
* **Task 1.4: 9-DoF IMU Integration** [x]
  * Attach IMU sensor plugin to fuselage center-of-gravity with noise characteristics.
* **Task 1.5: Middleware Bridge & TF Validation** [x]
  * Configure `ros_gz_bridge`, verify topic publishing rates, and validate joint commands.

---

### Phase 2: Control Domain & Actuation [COMPLETED]
* **Task 2.1: IMU Attitude Estimator Node** [x]
  * Implement C++ complementary filter node (`attitude_estimator`) consuming raw 100 Hz IMU data and publishing filtered pitch/roll attitude.
* **Task 2.2: Pitch Stabilator Controller** [x]
  * Implement PID loop (`stabilator_controller`) driving the horizontal stabilizer to maintain pitch setpoint with anti-windup clamping.
* **Task 2.3: Landing Gear Actuator Controller** [x]
  * Implement rate-limited trajectory node (`landing_gear_controller`) providing smooth deployment and state telemetry.
* **Task 2.4: Pitch Disturbance Benchmarking** [x]
  * Validate step responses and pitch stabilization recovery under setpoint variations.

---

### Phase 3: Synthetic Environments & Sensor Perception [COMPLETED]
* **Task 3.1: World 1 — Flat Rigid Ground (Testbed Alpha)** [x]
  * Model planar runway/landing strip (`SAFE`) in `worlds/testbed_alpha.sdf`.
* **Task 3.2: World 2 — Sloped & Stepped Surface (Testbed Beta)** [x]
  * Model terrain with gradients $>15^\circ$ and sudden drop-offs (`UNSAFE_SLOPE`) in `worlds/testbed_beta.sdf`.
* **Task 3.3: World 3 — Soft / Vegetated Ground (Testbed Gamma)** [x]
  * Model irregular, high-scattering terrain surfaces (`UNSAFE_VEGETATION`) in `worlds/testbed_gamma.sdf`.
* **Task 3.4: Ventral Sensor Processing (`sensing_domain`)** [x]
  * Implement `sensor_processor` C++ node to ingest sonar and ToF streams, compute temporal/spatial moving averages, and extract ground clearance.

---

### Phase 4: Feature Extraction & Edge AI [COMPLETED]
* **Task 4.1: Feature Engineering Pipeline** [x]
  * Implement feature extractor computing $\Delta d$, surface normal vector, terrain slope $\beta$, and rolling variances into a 7D vector.
* **Task 4.2: Terrain Classifier Training & C++ Code Generation** [x]
  * Train Decision Tree classifier in Python (`scripts/train_classifier.py`) and export deterministic, zero-allocation C++ inference rules (`model_rules.hpp`).
* **Task 4.3: Real-Time Inference Node & Actuation Interlock** [x]
  * Implement `terrain_classifier` node in C++ publishing `/intelligence/verdict` and `/intelligence/abort`, and interlock `landing_gear_controller` to inhibit deployment on abort.

---

### Phase 5: Closed-Loop Integration & Validation [COMPLETED]
* **Task 5.1: Master Launch Pipeline** [x]
  * Create `launch/system_launch.py` orchestrating Gazebo, parameter bridge, estimators, AI classifier, and gear controller with `world:=` and `gui:=` parameter support.
* **Task 5.2: Terminal Approach Flight Evaluation** [x]
  * Create `scripts/verify_system.py` automated regression suite evaluating safety interlock behavior across all three testbed scenarios.
* **Task 5.3: Latency & Failure-Mode Profiling** [x]
  * Create `intelligence_domain/benchmark_inference` profiling sub-microsecond inference execution time.

---

[**Back to main README**](../README.md)