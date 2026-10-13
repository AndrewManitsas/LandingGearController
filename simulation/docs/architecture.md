# System Architecture: The Three-Domain Model

To ensure deterministic real-time actuation without compromising perception capability, the platform employs a decoupled three-domain design modeled after avionics isolation standards.

```text
+-------------------------------------------------------------------------------+
|                               GAZEBO SIMULATOR                                |
|  - Physics Engine (1 ms step size, unpaused)                                  |
|  - Terrain Heightmaps (Alpha Concrete, Beta Slopes/Curbs, Gamma Vegetation)   |
|  - Rig URDF with Revolute Joints (Horizontal Stabilizer & Landing Gear)       |
+-------------------------------------------------------------------------------+
           | Simulated Ray/Acoustic Ranges                ^ Joint Commands
           | Simulated IMU Accelerations/Rates            | (Position Mode)
           v                                              |
+----------------------+     Sensor Packets      +------------------------------+
|    SENSING DOMAIN    |------------------------>|     INTELLIGENCE DOMAIN      |
|  (sensor_processor)  |                         |    (terrain_classifier)      |
| - 3x ToF Array Buff  |                         | - 7D Feature Ingestion       |
| - Sonar Cone Physics |                         | - Deterministic C++ Tree     |
| - Delta-d Dispersion |---+                     | - Nanosecond Edge Inference  |
+----------------------+   |                     +------------------------------+
                           | Synchronous                       |
                           | State Feed                        | SAFE / UNSAFE
                           v                                   v Abort Flag
                     +----------------------------------------------+
                     |                CONTROL DOMAIN                |
                     |   (attitude_estimator, stabilator_ctrl,      |
                     |          landing_gear_controller)            |
                     | - 100 Hz Complementary Filter (Pitch/Roll)   |
                     | - Closed-Loop PID Pitch Stabilizer           |
                     | - Rate-Limited Gear Trajectory & AI Inhibit  |
                     +----------------------------------------------+
```

## 1. Domain Responsibilities

| Domain | Physical Target Hardware | Simulation Implementation | Functional Role | Update Rate |
| :--- | :--- | :--- | :--- | :--- |
| **Sensing Domain** | Raspberry Pi Pico 2 W | `sensor_processor` (C++) | Ingests 3x ToF and sonar feeds, applies temporal moving-average smoothing, estimates surface normal vector, and extracts 7D feature vectors. | 40 Hz |
| **Intelligence Domain** | Raspberry Pi Zero W | `terrain_classifier` (C++) | Ingests 7D feature vector, evaluates deterministic compiled decision boundaries, publishes discrete verdicts (SAFE/UNSAFE), and issues gear deployment aborts. | 40 Hz |
| **Control Domain** | Arduino Uno | `attitude_estimator, stabilator_controller, landing_gear_controller` (C++) | Executes deterministic control loops: attitude complementary filter, closed-loop pitch PID stabilization, and rate-limited landing gear deployment with safety abort override. | 50-100 Hz |

## 2. Safety & Computational Isolation

**Resource Contention Prevention:** High-rate pitch stabilization (100 Hz) operates asynchronously from terrain classification (40 Hz), preventing sensor processing or feature evaluation delays from interfering with tailplane dynamics. 

**Deterministic Fail-Safe:** The landing gear controller enforces a hardware-style interlock: if the Intelligence Domain asserts /intelligence/abort, the deployment trajectory immediately reverses to the stowed angle (0.0 rad), preventing landing gear deployment regardless of operator requests.

**Zero-Allocation Edge Inference:** The trained classification logic compiles directly into static C++ decision trees without dynamic memory allocations (malloc/new), ensuring zero runtime jitter on resource-constrained embedded microcontrollers.

---

[**Back to main README**](../README.md)