# Work Breakdown Structure & Sprint Strategy

## 1. Plane Board Tasks

### Phase 1: Virtual Rig & Sensor Array
* **Task 1.1: Airframe & Actuator Modeling**
  * Model fixed-wing airframe in URDF with revolute joints for T-tail stabilator ($\pm 25^\circ$) and retractable main landing gear ($0^\circ\text{--}90^\circ$).
* **Task 1.2: 3x VL53L0X ToF Array Integration**
  * Mount three downward optical rangefinder plugins in an equilateral triangular configuration on the ventral sensor bay.
* **Task 1.3: Downward Sonar Integration**
  * Mount central ultrasonic rangefinder plugin with conical beam profile on the forward belly.
* **Task 1.4: 9-DoF IMU Integration**
  * Attach IMU sensor plugin to fuselage center-of-gravity with noise characteristics.
* **Task 1.5: Middleware Bridge & TF Validation**
  * Configure `ros_gz_bridge`, verify topic publishing rates ($\ge 50\text{ Hz}$), and validate TF tree in RViz.

---

### Phase 2: Control Domain & Actuation
* **Task 2.1: IMU Attitude Estimator Node**
  * Implement C++ complementary filter node consuming raw IMU and publishing filtered pitch/roll attitude.
* **Task 2.2: Pitch Stabilator Controller**
  * Implement PID loop driving the horizontal stabilizer to maintain level flight trim.
* **Task 2.3: Landing Gear Actuation Service**
  * Implement gear deployment state logic triggering extension on `SAFE` decision.
* **Task 2.4: Pitch Disturbance Benchmarking**
  * Verify pitch stabilizer recovery under simulated wind gusts and momentum pulses.

---

### Phase 3: Synthetic Environments & Dataset Generation
* **Task 3.1: World 1 — Flat Rigid Ground**
  * Model planar runway/landing strip (`SAFE`).
* **Task 3.2: World 2 — Sloped & Stepped Surface**
  * Model terrain with gradients $>15^\circ$ and sudden drop-offs (`UNSAFE`).
* **Task 3.3: World 3 — Soft / Vegetated Ground**
  * Model irregular, high-scattering terrain surfaces (`UNSAFE`).
* **Task 3.4: Automated Trajectory & Logging Harness**
  * Script automated descent trajectories, record ROS bags, and export labeled feature datasets.

---

### Phase 4: Feature Extraction & Edge AI
* **Task 4.1: Feature Engineering Pipeline**
  * Implement feature extractor computing $\Delta d$ (acoustic-optical difference), surface normal vector, terrain slope $\beta$, and rolling variances.
* **Task 4.2: Terrain Classifier Training**
  * Train lightweight classifier (Random Forest / MLP) penalizing false-safe errors.
* **Task 4.3: ROS Inference Node**
  * Wrap trained model into a real-time node publishing `/intelligence/verdict` at $10\text{ Hz}$.

---

### Phase 5: Closed-Loop Integration & Validation
* **Task 5.1: Master Launch Pipeline**
  * Write unified launch file bringing up Gazebo, bridge, estimators, AI node, and controller.
* **Task 5.2: Terminal Approach Flight Evaluation**
  * Run end-to-end simulated landing runs over composite terrain to evaluate touchdown safety.
* **Task 5.3: Latency & Failure-Mode Profiling**
  * Measure sensing-to-actuation latency and evaluate behavior under sensor dropouts.

---

## 2. Two-Person Parallel Execution Plan

```text
       Track A (Person 1: Control & Virtual Rig)                     Track B (Person 2: Terrains, Math & AI)
      ------------------------------------------                    ---------------------------------------
Sprint 1: Tasks 1.1 - 1.5 (URDF, Servos, Sensors, Bridge)     |      Tasks 3.1 - 3.3 (Worlds 1, 2, 3)
                                                              |      Task 4.1 (Feature Math with Synthetic Data)
                                                              |
Sprint 2: Tasks 2.1 - 2.4 (C++ Attitude Filter, Pitch PID)    |      Task 3.4 (Automated Data Harvest)
                                                              |      Task 4.2 (Train Classifier)
                                                              |
Sprint 3: Tasks 2.3 & 5.1 (Gear Actuator, Master Launch)      |      Task 4.3 (ROS Inference Node)
                                                              |      Tasks 5.2 - 5.3 (Closed-Loop Testing & Profiling)
```