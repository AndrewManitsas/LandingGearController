# Intelligent Autonomous UAV Landing Gear System: Simulation Proof of Concept

An autonomous landing decision and active attitude stabilization system for Unmanned Aerial Vehicles (UAVs). This project implements a multi-sensor fusion and Edge AI pipeline that classifies terminal descent terrain as `SAFE` or `UNSAFE` while actively maintaining stabilized sensor orientation relative to the ground plane.

Originally conceptualized around a three-microcontroller physical architecture (Raspberry Pi Pico 2 W, Raspberry Pi Zero W, and Arduino Uno), this repository hosts the **Software-in-the-Loop (SITL) digital twin** developed in ROS 2 and Gazebo.

---

## 1. Project Information

* **Institution:** Department of Electrical and Computer Engineering, Aristotle University of Thessaloniki
* **Course:** UAV01 - Sensor Systems for UAVs
* **Authors:** 
  * Andreas Manitsas (`amanitsb@ece.auth.gr`)
  * Maria Vrana (`mvranaa@ece.auth.gr`)
* **Execution Status:** Software-in-the-Loop (SITL) Proof of Concept (v1.0.0 Verified)
* **Target Stack:** Ubuntu 24.04 LTS, ROS 2 Jazzy Jalisco, Gazebo Harmonic, Python 3 (`pandas`, `scikit-learn`, `numpy`)

---

## 2. High-Level Concept

```text
                +---------------------------------------+
                |        UAV Terminal Descent           |
                +---------------------------------------+
                                   |
                     +-------------+-------------+
                     v                           v
        +-------------------------+ +-------------------------+
        |   Active Stabilization  | |  Multi-Sensor Fusion    |
        | (IMU + Pitch Stabilizer)| | (3x ToF + Sonar Range)  |
        +-------------------------+ +-------------------------+
                     |                           |
                     |                           v
                     |              +-------------------------+
                     |              |    Edge AI Inference    |
                     |              |  Terrain Classification |
                     |              +-------------------------+
                     |                           |
                     v                           v
        +-----------------------------------------------------+
        |                 Control & Actuation                 |
        |   - Pitch Trim Adjustment                           |
        |   - Gear Deploy (SAFE) vs. Inhibit/Abort (UNSAFE)   |
        +-----------------------------------------------------+
```

---

## 3. Documentation Hub

Detailed mathematical formulations, interface contracts, and task breakdowns are organized into dedicated guides:

| Document | Description |
| :--- | :--- |
| [**System Architecture**](docs/architecture.md) | Decoupled Three-Domain computational model (Sensing, Intelligence, Control), hardware-to-simulation mapping, and safety interlock principles. |
| [**Sensors & Physics Modeling**](docs/sensors_and_physics.md) | Overflow-safe complementary filter math, ToF surface normal extraction, acoustic-optical delta ($\Delta d$), and 7D feature vector formulation. |
| [**Work Breakdown Structure**](docs/work_breakdown.md) | 5-phase project lifecycle tracking completed milestones. |
| [**ROS 2 Interfaces & FSM**](docs/ros_interfaces.md) | Complete topic catalog, QoS profiles, TF2 coordinate hierarchy, and landing gear safety actuator logic. |
| [**Testing & Validation Plan**](docs/testing_and_validation.md) | Synthetic testbeds (Alpha, Beta, Gamma), automated regression test harness, and sub-microsecond latency benchmarks. |

---

## 4. Repository Layout

```text
LandingGearController/
├── config/
│   └── bridge_config.yaml         # ros_gz_bridge topic mapping
├── data/
│   └── dataset.csv                # Labeled 7D feature vectors collected across testbeds
├── docs/                          # Modular technical specifications
│   ├── architecture.md
│   ├── sensors_and_physics.md
│   ├── work_breakdown.md
│   ├── ros_interfaces.md
│   └── testing_and_validation.md
├── launch/
│   └── system_launch.py           # Unified orchestrator (supports world:= and gui:=)
├── models/                        # Airframe URDF and sensor definitions
│   └── urdf/landing_rig.urdf
├── scripts/
│   ├── record_dataset.py          # Multi-world 7D feature harvesting harness
│   ├── train_classifier.py        # Model trainer & C++ inference rule generator
│   └── verify_system.py           # Automated regression verification harness
├── src/
│   ├── control_domain/            # Pitch PID, attitude estimator, and gear controller (C++)
│   ├── sensing_domain/            # Temporal filtering, surface normal estimation, and 7D extraction (C++)
│   └── intelligence_domain/       # Deterministic C++ decision tree inference and latency benchmark
├── worlds/                        # Gazebo world definitions (Alpha, Beta, Gamma)
│   ├── testbed_alpha.sdf
│   ├── testbed_beta.sdf
│   └── testbed_gamma.sdf
└── README.md                      # Primary repository overview
```

---

## 5. Build & Execution Workflow

### 5.1 Compilation

```text
# Source ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# Build workspace with symlink install
cd ~/repos/LandingGearController
colcon build --symlink-install
source install/setup.bash
```

### 5.2 Unified System Launch

Execute nodes across separate terminal sessions or through launch scripts:

```text
# Testbed Alpha (Nominal flat runway, interactive GUI)
ros2 launch launch/system_launch.py world:=alpha gui:=true

# Testbed Beta (20-degree sloped ramp, headless execution)
ros2 launch launch/system_launch.py world:=beta gui:=false

# Testbed Gamma (Irregular vegetation mounds, interactive GUI)
ros2 launch launch/system_launch.py world:=gamma gui:=true
```

### 5.3 Automated Verification & Benchmarking

Execute the headless regression suite across all three testbeds:

```text
python3 scripts/verify_system.py
```

Run the deterministic C++ inference latency benchmark:

```text
ros2 run intelligence_domain benchmark_inference
```

---

## 6. License

This project is licensed under the GNU General Public License v3.0 (GPL-3.0). See the [LICENSE](LICENSE) file for details.