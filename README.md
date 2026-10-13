# UAV Landing Gear Controller: Simulation and Hardware Prototype

This course project explores UAV landing gear assistance through two complementary prototypes. The ROS 2/Gazebo simulation implements terrain feature extraction, classification and simulated flight control. The Arduino Uno bench prototype uses real IMU and distance measurements to demonstrate landing-readiness checks, virtual stabilizer commands and a manually controlled, latched gear sequence.

Each implementation has its own setup and limitations. Simulation results do not establish real aircraft performance. The hardware prototype displays actuator commands without driving servos.

This project was done as a semester deliverable for the course *UAV01 Sensor Systems for UAVs*, which is part of the *MSc Unmanned Aerial Vehicles* in the *Department of Electrical and Computer Engineering, Aristotle University of Thessaloniki*.

---

## 1. Project Information

- **Institution:** Department of Electrical and Computer Engineering, Aristotle University of Thessaloniki
- **Course:** UAV01 - Sensor Systems for UAVs
- **Authors:** Andreas Manitsas (`amanitsb@ece.auth.gr`), Maria Vrana (`mvranaa@ece.auth.gr`)
- **Implementations:** Software simulation and a standalone hardware bench demo
- **License:** [GPL-3.0](LICENSE)

## 2. Implementations

| Aspect | Simulation | Hardware demo |
| :--- | :--- | :--- |
| Environment | Ubuntu / WSL, ROS 2 Jazzy, Gazebo Harmonic | Arduino Uno, Arduino IDE, Python monitor on Windows |
| Sensors | Three simulated ToFs, sonar and IMU | One VL53L0X and MPU6500 |
| Decision | Compiled decision tree with synthetic training data | Distance, tilt, stability and freshness rules |
| Stabilizer | Simulated PID control | Virtual proportional pitch correction |
| Gear | Abort forces stowing, simulated joint trajectory | Manual deployment stays latched until S or reset |
| Scope | Broader simulated landing concept | Real sensing and a smaller control demonstration |

The original three-board plan assigned sensing to Pico 2 W, classification to Pi Zero W and control to Uno. The simulation represents those responsibilities as ROS nodes on a PC. The hardware demo runs its sensing and rules on one Uno. Neither prototype connects the physical board to Gazebo as hardware-in-the-loop.

## 3. Documentation

| Guide | Contents |
| :--- | :--- |
| [Simulation setup](simulation/README.md) | Build, launch, scenarios, data collection and tests |
| [Hardware setup](hardware/README.md) | Wiring, calibration, readiness, commands and Python app |
| [Implementation comparison](docs/implementation-comparison.md) | Shared purpose, algorithm differences and current limits |
| [Simulation architecture](simulation/docs/architecture.md) | ROS nodes and responsibilities |
| [Sensors and feature definitions](simulation/docs/sensors_and_physics.md) | Actual runtime feature mapping and design differences |
| [ROS interfaces](simulation/docs/ros_interfaces.md) | Topics, message types and control sequence |
| [Testing and evidence](simulation/docs/testing_and_validation.md) | Checks, historical results and validation limits |
| [Implementation scope](simulation/docs/work_breakdown.md) | Existing simulation components and remaining work |
| [Earlier design specification](simulation/docs/design/original_simulation_specification.md) | Historical reference, not the current implementation contract |
| [Presentation](presentation/landing_gear_presentation_updated.pptx) | Combined simulation and hardware demonstration |

## 4. Repository Layout

```text
LandingGearController/
|-- simulation/
|   |-- README.md
|   |-- config/             # Gazebo / ROS bridge configuration
|   |-- data/               # Synthetic labelled dataset
|   |-- docs/               # Simulation implementation guides
|   |-- launch/             # Gazebo and ROS orchestration
|   |-- models/             # URDF rig
|   |-- scripts/            # Recording, training, flight and verification
|   |-- src/                # Sensing, intelligence and control ROS packages
|   `-- worlds/             # Alpha, Beta, Gamma and approach scenarios
|-- hardware/
|   |-- README.md
|   |-- firmware/           # Uno sketch
|   `-- tools/              # Python monitor, requirements and offline tests
|-- docs/                   # Combined overview and GitHub About text
|-- presentation/           # Course presentation
|-- LICENSE
`-- README.md
```

## 5. Getting Started

For simulation, follow [the ROS/Gazebo workflow](simulation/README.md). For the connected Uno, follow [the hardware workflow](hardware/README.md). Keep separate WSL and Windows checkouts of this same repository when using both environments. Python virtual environments and build outputs remain local to each checkout.

The hardware demonstration consists of waiting for readiness with a level IMU and stable target, deploying with D, changing distance or tilt to show a warning while gear remains deployed, and stowing with S. The simulation offers broader terrain and approach scenarios with a different abort policy.

## 6. Current Limits

The hardware uses one range sensor and virtual actuator commands. It has no terrain-material classifier, physical gear feedback or touchdown detector. Simulation classification scores use synthetic, correlated data and per-world labels. They do not measure performance on real terrain. Both implementations are course prototypes rather than validated aircraft systems.

## 7. License

This project is licensed under the GNU General Public License v3.0. See [LICENSE](/LICENSE).