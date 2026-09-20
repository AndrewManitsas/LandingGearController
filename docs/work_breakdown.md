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
  * Implement rate-limited trajectory node (`landing_gear_controller`) providing smooth deployment and state telemetry (`STOWED`, `DEPLOYING`, `DEPLOYED`, `RETRACTING`).
* **Task 2.4: Pitch Disturbance Benchmarking** [x]
  * Validate step responses and pitch stabilization recovery under setpoint variations.

---

### Phase 3: Synthetic Environments & Sensor Perception [NEXT]
* **Task 3.1: World 1 — Flat Rigid Ground (Testbed Alpha)** [ ]
  * Model planar runway/landing strip (`SAFE`).
* **Task 3.2: World 2 — Sloped & Stepped Surface (Testbed Beta)** [ ]
  * Model terrain with gradients $>15^\circ$ and sudden drop-offs (`UNSAFE`).
* **Task 3.3: World 3 — Soft / Vegetated Ground (Testbed Gamma)** [ ]
  * Model irregular, high-scattering terrain surfaces (`UNSAFE`).
* **Task 3.4: Ventral Sensor Processing (`sensing_domain`)** [ ]
  * Ingest sonar and ToF streams, compute spatial-temporal moving averages, and extract ground clearance.

---

### Phase 4: Feature Extraction & Edge AI
* **Task 4.1: Feature Engineering Pipeline** [ ]
  * Implement feature extractor computing $\Delta d$ (acoustic-optical difference), surface normal vector, terrain slope $\beta$, and rolling variances.
* **Task 4.2: Terrain Classifier Training** [ ]
  * Train lightweight classifier (Random Forest / MLP) penalizing false-safe errors.
* **Task 4.3: ROS Inference Node** [ ]
  * Wrap trained model into a real-time node publishing `/intelligence/verdict` at $10\text{ Hz}$.

---

### Phase 5: Closed-Loop Integration & Validation
* **Task 5.1: Master Launch Pipeline** [ ]
  * Write unified launch file bringing up Gazebo, bridge, estimators, AI node, and controller.
* **Task 5.2: Terminal Approach Flight Evaluation** [ ]
  * Run end-to-end simulated landing runs over composite terrain to evaluate touchdown safety.
* **Task 5.3: Latency & Failure-Mode Profiling** [ ]
  * Measure sensing-to-actuation latency and evaluate behavior under sensor dropouts.

---