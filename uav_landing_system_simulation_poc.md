# Technical Specification & Simulation Architecture: Autonomous UAV Landing Gear System

## 1. Executive Summary & Project Context

* **Project Title:** Landing Gear System: A Multi-Domain Architecture for Terrain Classification and Landing Gear Actuation
* **Authors:** Andreas Manitsas, Maria Vrana (ECE, Aristotle University of Thessaloniki)
* **Status:** Hardware-in-the-Loop (HIL) to Software-in-the-Loop (SITL) Proof of Concept (PoC) Transition
* **Objective:** Validate the sensor fusion, active stabilization, and terrain classification pipeline within a high-fidelity physics simulator (Gazebo / ROS 2) while maintaining strict adherence to the decoupled three-domain architecture (Sensing, Intelligence, and Control).

Due to the temporary unavailability of physical microcontrollers and sensors, the project methodology pivots to a simulated digital twin. This approach provides deterministic, repeatable validation across complex terrain types (sloped, vegetative, debris-laden) while preserving the identical communication interfaces and algorithms developed for embedded hardware.

---

## 2. Digital Twin Architecture & Domain Mapping

The system design preserves the multi-domain separation modeled after aerospace standards (e.g., ARINC 653 concepts) by dedicating independent nodes and message bridges to each computational domain.

```
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

### 2.1 Domain Responsibilities

| Domain | Original Target | Simulation Implementation | Functional Role |
| :--- | :--- | :--- | :--- |
| **Sensing Domain** | Raspberry Pi Pico 2 W | `sensing_hub_node` (C++/ROS 2) | Simulates raw I2C/analog sensor sampling at $100\text{ Hz}$, applies sensor noise models, and formats packets. |
| **Intelligence Domain** | Raspberry Pi Zero W | `ml_classifier_node` (Python/TFLite) | Runs asynchronously at $10\text{--}20\text{ Hz}$. Calculates high-level statistical features and outputs landing suitability (`SAFE` / `UNSAFE`). |
| **Control Domain** | Arduino Uno | `flight_actuator_node` (C++/ROS 2) | Executes the real-time loop at $100\text{ Hz}$. Runs the complementary filter, PID pitch compensation, and deterministic servo state machine. |

---

## 3. Physical & Sensor Mathematical Modeling

### 3.1 Inertial Measurement Unit (MPU-9250 Emulation)
The simulated IMU produces body-frame linear accelerations $\vec{a} = [a_x, a_y, a_z]^T$ and angular velocities $\vec{\omega} = [\omega_x, \omega_y, \omega_z]^T$.

1. **Noise Characteristics:**
   $$\vec{a}_{\text{meas}} = \vec{a}_{\text{true}} + \vec{b}_a + \vec{\eta}_a$$
   $$\vec{\omega}_{\text{meas}} = \vec{\omega}_{\text{true}} + \vec{b}_g + \vec{\eta}_g$$
   where $\vec{b}$ represents dynamic zero-rate bias drift and $\vec{\eta} \sim \mathcal{N}(0, \sigma^2)$ represents white Gaussian noise.

2. **Complementary Filter Formulation:**
   * Accelerometer Roll ($\phi_{\text{acc}}$) and Pitch ($\theta_{\text{acc}}$):
     $$\phi_{\text{acc}} = \text{atan2}(a_y, a_z) \cdot \left(\frac{180^\circ}{\pi}\right)$$
     $$\theta_{\text{acc}} = \text{atan2}\left(-a_x, \sqrt{a_y^2 + a_z^2}\right) \cdot \left(\frac{180^\circ}{\pi}\right)$$
   * Dynamic Attitude Fusion ($\alpha = 0.90$):
     $$\theta_k = \alpha \cdot (\theta_{k-1} + \omega_{y, k} \cdot \Delta t) + (1 - \alpha) \cdot \theta_{\text{acc}, k}$$
     $$\phi_k = \alpha \cdot (\phi_{k-1} + \omega_{x, k} \cdot \Delta t) + (1 - \alpha) \cdot \phi_{\text{acc}, k}$$
     $$\psi_k = \psi_{k-1} + \omega_{z, k} \cdot \Delta t \quad \text{(Integrated heading)}$$

### 3.2 Laser Time-of-Flight Sensors (3x VL53L0X Array)
The 3 ToF sensors are arranged in an equilateral triangular configuration on the bottom plate with radius $r = 0.06\text{ m}$ relative to the vehicle center:
* Sensor 1: Fore ($x_1 = r, y_1 = 0$)
* Sensor 2: Aft-Left ($x_2 = -r/2, y_2 = r\sqrt{3}/2$)
* Sensor 3: Aft-Right ($x_3 = -r/2, y_3 = -r\sqrt{3}/2$)

Each sensor has a narrow Field of View ($\approx 25^\circ$) modeled with narrow ray castings:
$$d_{\text{ToF}, i} = d_{\text{true}, i} + \eta_{\text{ToF}}, \quad \eta_{\text{ToF}} \sim \mathcal{N}(0, \sigma_{\text{laser}}^2), \; \sigma_{\text{laser}} \approx 0.002\text{ m}$$

**Terrain Gradient Vector Calculation:**
From the three localized spatial points $\vec{P}_i = [x_i, y_i, d_{\text{ToF}, i}]^T$, the surface normal unit vector $\hat{n}$ is extracted:
$$\vec{v}_{12} = \vec{P}_2 - \vec{P}_1, \quad \vec{v}_{13} = \vec{P}_3 - \vec{P}_1$$
$$\vec{N} = \vec{v}_{12} \times \vec{v}_{13}, \quad \hat{n} = \frac{\vec{N}}{\|\vec{N}\|}$$
The estimated terrain inclination angle $\theta_{\text{terrain}}$ is:
$$\theta_{\text{terrain}} = \arccos(\hat{n} \cdot \hat{k}) = \arccos(n_z)$$

### 3.3 Ultrasonic Transducer (HC-SR04 Wide Cone Emulation)
Ultrasonic range detection relies on acoustic reflections over a broad $30^\circ\text{--}40^\circ$ cone. Unlike optical beams that measure direct point distances, the acoustic return captures the closest surface point within its cone footprint:
$$d_{\text{sonar}} = \min_{\vec{p} \in \text{Cone}} \|\vec{p}_{\text{sensor}} - \vec{p}\| + \eta_{\text{sonar}}$$

**Differential Texture Signature ($\Delta d$):**
$$\Delta d = d_{\text{sonar}} - \bar{d}_{\text{ToF}} = d_{\text{sonar}} - \frac{1}{3}\sum_{i=1}^3 d_{\text{ToF}, i}$$

* On **flat, rigid ground**: $\Delta d \approx 0$.
* On **porous/tall vegetation**: The optical beam penetrates toward the root/lower soil level, while the wide acoustic wave reflects off the upper grass canopy, resulting in $d_{\text{sonar}} < \bar{d}_{\text{ToF}} \implies \Delta d < 0$.
* On **broken/stepped ground**: Discrepancies between beam geometries result in elevated spatial-temporal variance $\sigma_{\Delta d}^2$.

---

## 4. Edge AI Feature Vector & Dataset Matrix

The Intelligence Domain evaluates a rolling buffer of $W = 10$ samples ($0.1\text{ s}$ window at $100\text{ Hz}$) to construct a 7-dimensional feature vector $\vec{x}$:

$$\vec{x} = \begin{bmatrix}
\bar{d}_{\text{ToF}} & \text{Mean altitude from 3 ToF sensors} \\
\sigma^2(d_{\text{ToF}}) & \text{Spatial dispersion across the 3 ToF points} \\
\Delta d & \text{Differential acoustic-optical distance} \\
\sigma^2(\Delta d) & \text{Temporal variance of acoustic differential} \\
\theta_{\text{terrain}} & \text{Estimated ground slope angle} \\
\|\vec{\omega}\| & \text{Norm of angular rate (turbulence/instability indicator)} \\
\text{Tilt Error} & \|\theta_k - \theta_{\text{terrain}}\|
\end{bmatrix}$$

### Terrain Classification Targets

```
                                 [Feature Vector x]
                                         |
                                         v
                         +-------------------------------+
                         |   Edge AI Classifier (TinyML) |
                         +-------------------------------+
                                  /             \
                                 /               \
                   Score >= 0.5 /                 \ Score < 0.5
                               v                   v
                    +--------------------+   +---------------------+
                    |   SAFE (Flag = 1)  |   |  UNSAFE (Flag = 0)  |
                    | Flat Asphalt/Tiles |   | Slopes / Tall Grass |
                    +--------------------+   +---------------------+
                               |                       |
                               v                       v
                      Deploy Landing Gear       Hold Retracted State
                      Continue Descent          Abort / Hover Mode
```

---

## 5. Control Domain Execution Architecture

The Control Domain operates deterministically, independent of the classifier's inference latency.

### 5.1 Active Pitch Stabilization Loop
To counteract descent-induced pitch disturbances and ensure the sensor array remains parallel to the terrain, a closed-loop PID controller drives the simulated horizontal stabilizer servo (revolute joint):

$$e_\theta(t) = \theta_{\text{target}} - \theta_{\text{estimated}}(t)$$
$$u_{\text{servo}}(t) = K_p e_\theta(t) + K_i \int_0^t e_\theta(\tau) d\tau + K_d \frac{d e_\theta(t)}{dt}$$

* $u_{\text{servo}}$ is bounded within $[-45^\circ, +45^\circ]$ matching the physical SG90 mechanical limit.
* Anti-windup clamping prevents integral saturation during high-amplitude gusts.

### 5.2 Landing Gear Actuator State Machine

```
      +---------------------+
      |      RETRACTED      |<------------------------------------+
      +---------------------+                                     |
                 |                                                |
                 | [SAFE == 1] AND [Altitude <= H_deploy]         | [SAFE == 0] OR
                 v                                                | [Altitude > H_deploy]
      +---------------------+                                     |
      |      DEPLOYING      |                                     |
      +---------------------+                                     |
                 |                                                |
                 | [Servo Target Reached (t >= t_transit)]        |
                 v                                                |
      +---------------------+                                     |
      |   DEPLOYED (LOCKED) |-------------------------------------+
      +---------------------+
```

---

## 6. Verification and Validation Plan

### Phase 1: Synthetic Data Pipeline
1. Run Gazebo automated sweeps over 3 parametric test beds:
   * **Testbed Alpha (Safe):** Smooth plane, $0^\circ\text{--}5^\circ$ slope.
   * **Testbed Beta (Unsafe - Slope):** Rigid plane with varying inclinations ($10^\circ, 15^\circ, 20^\circ, 30^\circ$).
   * **Testbed Gamma (Unsafe - Irregular/Vegetation):** Uneven heightmaps with random high-frequency elevation variance ($\pm 0.08\text{ m}$).
2. Log 5,000 instances of the feature vector $\vec{x}$ labeled with binary ground truth.

### Phase 2: Model Training & Quantization
1. Train a lightweight Multilayer Perceptron (MLP) or Random Forest baseline in Python.
2. Target Model Architecture: $7 \to 16 \to 8 \to 1$ fully connected layers with ReLU activations and Sigmoid output.
3. Quantize using TensorFlow Lite (int8/float32) to verify computational footprints compatible with edge microcontrollers.

### Phase 3: Closed-Loop SITL Demonstration
1. Release simulated UAV from an initial altitude $h_0 = 2.0\text{ m}$ in Gazebo with simulated aerodynamic downwash and horizontal wind gusts ($2\text{ m/s}$).
2. Verify:
   * Stabilization loop holds sensor orientation within $\pm 3.0^\circ$ of vertical.
   * Gear actuation deploys reliably over Testbed Alpha at $h \le 0.5\text{ m}$.
   * System flags `UNSAFE` and preserves retracted gear state over Testbeds Beta and Gamma.