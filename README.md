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
* **Execution Status:** Software-in-the-Loop (SITL) Proof of Concept
* **Target Stack:** Ubuntu 24.04 LTS, ROS 2 Jazzy, Gazebo Harmonic, colcon

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
        |   - Gear Deploy (SAFE) vs. Abort/Hover (UNSAFE)     |
        +-----------------------------------------------------+
```

---

## 3. Documentation Hub

Detailed mathematical formulations, interface contracts, and task breakdowns are organized into dedicated guides:

| Document | Description |
| :--- | :--- |
| [**System Architecture**](docs/architecture.md) | Detailed Three-Domain computational model (Sensing, Intelligence, Control), hardware-to-simulation mapping, and safety decoupling principles. |
| [**Sensors & Physics Modeling**](docs/sensors_and_physics.md) | Overflow-safe complementary filter math, ToF surface normal extraction, acoustic-optical delta ($\Delta d$), and 7D feature vector formulation. |
| [**Work Breakdown Structure**](docs/work_breakdown.md) | Flat 5-phase task hierarchy. |
| [**ROS 2 Interfaces & FSM**](docs/ros_interfaces.md) | ROS 2 topic names, message structures, publication rates, TF2 transform tree, and actuator finite-state machine. |
| [**Testing & Validation Plan**](docs/testing_and_validation.md) | Synthetic testbeds (Alpha, Beta, Gamma), closed-loop verification metrics, and latency benchmarking criteria. |

---

## 4. Repository Layout

```text
LandingGearController/
├── config/
│   └── bridge_config.yaml       # ros_gz_bridge topic mapping
├── docs/                        # Modular technical specifications
│   ├── architecture.md
│   ├── sensors_and_physics.md
│   ├── work_breakdown.md
│   ├── ros_interfaces.md
│   └── testing_and_validation.md
├── models/                      # Airframe URDF and sensor definitions
│   └── urdf/landing_rig.urdf
├── src/
│   ├── control_domain/          # Pitch PID, attitude estimator, and gear controller (C++)
│   ├── sensing_domain/          # Sonar and ToF array pre-processing (C++)
│   └── intelligence_domain/     # Terrain feature extraction & Edge AI inference (Python)
├── worlds/                      # Gazebo world definitions (Alpha, Beta, Gamma)
└── README.md                    # Primary repository overview
```

---

## 5. Build & Execution Workflow

### 5.1 Build C++ Nodes

```text
# Source ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# Build all packages
cd ~/repos/LandingGearController
colcon build --symlink-install
source install/setup.bash
```

Compiled binaries are output to `./bin/sensor_hub_node` and `./bin/flight_actuator_node`.

### 5.2 Running the System

Execute nodes across separate terminal sessions or through launch scripts:

```text
# Terminal 1: Simulation World
gz sim empty.sdf

# Terminal 2: ROS-Gazebo Parameter Bridge
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=config/bridge_config.yaml

# Terminal 3: Attitude Estimator Node
ros2 run control_domain attitude_estimator

# Terminal 4: Stabilator Pitch PID Controller
ros2 run control_domain stabilator_controller

# Terminal 5: Landing Gear Actuator Controller
ros2 run control_domain landing_gear_controller
```

### 5.3 Clean Build Artifacts

```text
make clean
```

---

## 6. License

This project is licensed under the GNU General Public License v3.0 (GPL-3.0). See the [LICENSE](LICENSE) file for details.