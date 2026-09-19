# Intelligent Autonomous UAV Landing Gear System: Simulation Proof of Concept

An autonomous landing decision and active attitude stabilization system for Unmanned Aerial Vehicles (UAVs). This project implements a multi-sensor fusion and Edge AI pipeline that classifies terminal descent terrain as `SAFE` or `UNSAFE` while actively maintaining perpendicular sensor orientation relative to the ground plane

Originally designed around a three-microcontroller physical architecture (Raspberry Pi Pico 2 W, Raspberry Pi Zero W, and Arduino Uno), this repository contains the **Software-in-the-Loop (SITL) digital twin** developed in ROS 2 and Gazebo.

---

## 1. Project Information

* **Institution:** Department of Electrical and Computer Engineering, Aristotle University of Thessaloniki
* **Course:** UAV01 - Sensor Systems for UAVs
* **Authors:** 
  * Andreas Manitsas (`amanitsb@ece.auth.gr`)
  * Maria Vrana (`mvranaa@ece.auth.gr`)
* **Execution Status:** Software-in-the-Loop (SITL) Proof of Concept
* **Target Stack:** Ubuntu 24.04 LTS, ROS 2 (Humble), Gazebo Classic 11 / Fortress, GNU Make

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
| [**Work Breakdown & Sprints**](docs/work_breakdown.md) | Flat 5-phase task hierarchy (formatted for Plane issue tracking), 2-person work division, and the 3-sprint execution roadmap. |
| [**ROS 2 Interfaces & FSM**](docs/ros_interfaces.md) | ROS 2 topic names, message structures, publication rates, TF2 transform tree, and actuator finite-state machine. |
| [**Testing & Validation Plan**](docs/testing_and_validation.md) | Synthetic testbeds (Alpha, Beta, Gamma), closed-loop verification metrics, and latency benchmarking criteria. |

---

## 4. Repository Layout

```text
uav_landing_gear/
├── Makefile                     # Root GNU build file (replaces CMake/colcon)
├── README.md                    # Primary repository overview
├── requirements.txt             # Python dependencies (scikit-learn, numpy, etc.)
├── docs/                        # Modular technical specifications
│   ├── architecture.md
│   ├── sensors_and_physics.md
│   ├── work_breakdown.md
│   ├── ros_interfaces.md
│   └── testing_and_validation.md
├── config/                      # PID gains, sensor noise, and threshold params
├── launch/                      # System orchestration and Gazebo launch files
├── models/                      # URDF/Xacro descriptions and sensor plugins
├── worlds/                      # Gazebo world definitions (Safe / Unsafe testbeds)
├── src/
│   ├── sensing_domain/          # Ingestion & moving-average filtering (Pico equiv.)
│   ├── intelligence_domain/     # Feature extraction & ML inference (Zero W equiv.)
│   └── control_domain/          # Complementary filter & pitch PID (Uno equiv.)
├── bin/                         # Output directory for compiled C++ binaries
└── build/                       # Intermediate object files (.o)
```

---

## 5. Build & Execution Workflow

### 5.1 Build C++ Nodes

```text
# Source ROS 2 Humble environment
source /opt/ros/humble/setup.bash

# Compile sensing and control domain executables
make -j4
```

Compiled binaries are output to `./bin/sensor_hub_node` and `./bin/flight_actuator_node`.

### 5.2 Running the System

Execute nodes across separate terminal sessions or through launch scripts:

```text
# Terminal 1: Launch Gazebo Environment with Rig
ros2 launch uav_landing_gear simulation.launch.py world:=flat_safe

# Terminal 2: Sensing Domain (Pico 2 W Equivalent)
./bin/sensor_hub_node

# Terminal 3: Control Domain (Arduino Uno Equivalent)
./bin/flight_actuator_node

# Terminal 4: Intelligence Domain (RPi Zero W Equivalent)
python3 src/intelligence_domain/inference_node.py
```

### 5.3 Clean Build Artifacts

```text
make clean
```