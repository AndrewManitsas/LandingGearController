# System Architecture: The Three-Domain Model

To ensure deterministic real-time actuation without compromising perception capability, the platform employs a decoupled three-domain design modeled after avionics isolation standards.

```text
+-------------------------------------------------------------------------------+
|                               GAZEBO SIMULATOR                                |
|  - Physics Engine (ODE/Bullet)                                                |
|  - Terrain Heightmaps (Safe Asphalt, High-Gradient Slope, Irregular Grass)    |
|  - Robot SDF/URDF Model with 2x Revolute Joints (SG90 Servos)                |
+-------------------------------------------------------------------------------+
           | Simulated Ray/Acoustic Ranges                ^ Joint Commands
           | Simulated IMU Accelerations/Rates            | (Effort/Position)
           v                                              |
+----------------------+     Sensor Packets      +------------------------------+
|    SENSING DOMAIN    |------------------------>|     INTELLIGENCE DOMAIN      |
|  (ROS 2 Sensor Hub)  |                         |    (Python / TinyML Node)    |
| - 3x ToF Range Noise |                         | - Feature Extraction Engine  |
| - Sonar Cone Physics |                         | - Edge Classifier (ML Model) |
| - IMU Preprocessing  |---+                     | - Ground Station Telemetry   |
+----------------------+   |                     +------------------------------+
                           | Synchronous                       |
                           | State Feed                        | SAFE / UNSAFE
                           v                                   v Status Flag
                     +----------------------------------------------+
                     |                CONTROL DOMAIN                |
                     |         (High-Rate Controller Node)          |
                     | - Complementary Filter (Pitch/Roll/Yaw)      |
                     | - Closed-Loop PID Pitch Stabilizer           |
                     | - Safety-Critical Gear Deployment Logic      |
                     +----------------------------------------------+
```

## 1. Domain Responsibilities

| Domain | Physical Target Hardware | Simulation Implementation | Functional Role | Update Rate |
| :--- | :--- | :--- | :--- | :--- |
| **Sensing Domain** | Raspberry Pi Pico 2 W | `sensor_hub_node` (C++) | Ingests raw sensor feeds, applies sensor noise models, performs moving-average smoothing, and publishes feature packets. | 100 Hz |
| **Intelligence Domain** | Raspberry Pi Zero W | `ml_classifier_node` (Python/TFLite) | Evaluates spatial and acoustic metrics, executes ML inference, outputs landing suitability (`SAFE`/`UNSAFE`), and streams telemetry. | 10–20 Hz |
| **Control Domain** | Arduino Uno | `flight_actuator_node` (C++) | Executes deterministic real-time loops: complementary attitude filter, closed-loop pitch PID stabilization, and landing gear deployment state machine. | 100 Hz |

## 2. Safety & Computational Isolation

**Resource Contention Prevention:** Isolates high-latency ML inference from high-rate attitude control loops. 

**Deterministic Fail-Safe:** If the Intelligence Domain drops a frame or hangs, the Control Domain continues stabilizing the gimbal and preserves the default retracted gear state.

---

[**Back to main README**](../README.md)