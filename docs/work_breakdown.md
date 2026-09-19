# Work Breakdown Structure & Sprint Strategy

## 1. Plane Board Tasks (Flat Two-Level Structure)

### Phase 1: Virtual Rig & Sensor Array (Gazebo / URDF)
* **Task 1.1: Chassis & Actuator Links Modeling** — Define root frame, landing gear bracket, and sensor gimbal link in URDF/Xacro with revolute joints for SG90 servos ($0^\circ\text{--}180^\circ$ for gear deployment; $\pm 30^\circ$ for pitch stabilization).
* **Task 1.2: 3x VL53L0X ToF Plugin Integration** — Mount three narrow-beam ray/range sensor plugins in an equilateral triangular configuration ($25^\circ$ FOV, $0.03\text{--}2.0\text{ m}$ range, Gaussian noise $\sigma = 1.5\text{ mm}$).
* **Task 1.3: HC-SR04 Ultrasonic Plugin Integration** — Mount a central range sensor plugin with conical beam characteristics ($35^\circ$ FOV, $0.02\text{--}4.0\text{ m}$ range).
* **Task 1.4: MPU-9250 IMU Plugin Configuration** — Attach `gazebo_ros_imu` to the stabilized plate with bias drift and vibration noise parameters.
* **Task 1.5: Sensor Stream & TF Frame Validation** — Verify that sensor topics publish at $\ge 50\text{ Hz}$ with valid coordinate transforms in RViz.

### Phase 2: Control Domain & Actuation (Low-Level Node)
* **Task 2.1: Port IMU Complementary Filter to ROS C++** — Implement calibrated filter ($\alpha = 0.90$) in a C++ node consuming `/imu/data` and publishing `/control/attitude`.
* **Task 2.2: Active Pitch PID Controller Implementation** — Implement PID loop driving the simulated pitch stabilizer servo.
* **Task 2.3: Landing Gear Deployment Service** — Subscribe to `/intelligence/verdict` and trigger the gear deployment joint to $90^\circ$ on `SAFE`.
* **Task 2.4: Disturbance Rejection Benchmarking** — Verify stabilizer recovery under simulated angular momentum pulses.

### Phase 3: Synthetic Environments & Dataset Generation
* **Task 3.1: World 1 — Flat Rigid Ground** — Model planar concrete ground (`SAFE`).
* **Task 3.2: World 2 — Sloped & Stepped Surface** — Build models with gradients $>15^\circ$ and step drop-offs (`UNSAFE`).
* **Task 3.3: World 3 — Soft / Vegetation Ground** — Build irregular terrain models causing acoustic backscatter (`UNSAFE`).
* **Task 3.4: Automated Trajectory & Logging Harness** — Script automated vertical descents ($2.0\text{ m} \to 0.2\text{ m}$), record bag files, and export labeled CSV datasets.

### Phase 4: Feature Extraction & Edge AI (Intelligence Domain)
* **Task 4.1: Feature Engineering Pipeline** — Implement module to compute $\Delta d$, surface normal vector, terrain slope angle $\beta$, and rolling variances.
* **Task 4.2: Model Architecture & Offline Training** — Train MLP/Random Forest with penalized false-safe loss on labeled datasets.
* **Task 4.3: Standalone ROS Inference Node** — Wrap trained model into a node publishing `/intelligence/verdict` at $10\text{ Hz}$.

### Phase 5: Closed-Loop Integration & Validation
* **Task 5.1: Master Launch Configuration** — Create launch file coordinating Gazebo, sensing, intelligence, and control nodes.
* **Task 5.2: Terminal Descent Mission Evaluation** — Execute descent runs over composite terrain to confirm selective gear actuation.
* **Task 5.3: Latency & Failure-Mode Profiling** — Benchmark total loop latency and test edge-case sensor dropouts.

---

## 2. Two-Person Parallel Execution Plan

```text
       Track A (Person 1: Control & Virtual Rig)                     Track B (Person 2: Terrains, Math & AI)
      ------------------------------------------                    ---------------------------------------
Sprint 1: Tasks 1.1 - 1.5 (URDF, Servos, Plugins)             |      Tasks 3.1 - 3.3 (Worlds 1, 2, 3)
                                                              |      Task 4.1 (Feature Math with Synthetic Data)
                                                              |
Sprint 2: Tasks 2.1 - 2.4 (C++ Filter, Pitch PID)             |      Task 3.4 (Automated Data Harvest)
                                                              |      Task 4.2 (Train Classifier)
                                                              |
Sprint 3: Tasks 2.3 & 5.1 (Gear Actuator, Master Launch)      |      Task 4.3 (ROS Inference Node)
                                                              |      Tasks 5.2 - 5.3 (Closed-Loop Testing & Profiling)
```

---

[**Back to main README**](../README.md)