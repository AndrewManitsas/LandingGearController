# ROS 2 / Gazebo Simulation

This implementation explores the broader landing-assistance concept with three simulated ToFs, sonar, an IMU, a compiled terrain classifier, pitch control and rate-limited gear commands. It runs on the simulation host. The embedded boards in the original proposal describe intended responsibilities, not deployed hardware.

## 1. Requirements

Ubuntu 24.04 / WSL Ubuntu, ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_bridge`, `ros_gz_sim`, colcon and the ROS dependencies declared in `src/*/package.xml`. Dataset training uses Python 3 with numpy, pandas and scikit-learn. No hardware serial connection is required.

## 2. Build

Run from the repository root in WSL / Ubuntu:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/repos/LandingGearController
colcon build --base-paths simulation/src --symlink-install
source install/setup.bash
```

Build, install and log directories remain at the root in this workflow. Source the ROS environment and this checkout's `install/setup.bash` in every new terminal. Launch resource paths resolve relative to the launch file, so the checkout itself need not be at the example location.

## 3. Launch and Scenarios

From the repository root:

```bash
ros2 launch simulation/launch/system_launch.py world:=approach gui:=true flight:=true
ros2 launch simulation/launch/system_launch.py world:=alpha gui:=true flight:=false
ros2 launch simulation/launch/system_launch.py world:=beta gui:=true flight:=false
ros2 launch simulation/launch/system_launch.py world:=gamma gui:=true flight:=false
```

Run one scenario at a time. `world` defaults to `approach`, `gui` and `flight` default to `true`. `gui:=false` selects a headless server. `flight:=false` disables the carrier and flight supervisor for static sensing and manual gear checks. `flight:=true` enables the supervisor's arming, descent, flare, touchdown and go-around sequence.

| World | Intended demonstration |
| :--- | :--- |
| Alpha | Flat runway, SAFE class |
| Beta | Sloped and stepped terrain, UNSAFE_SLOPE class |
| Gamma | Synthetic mounds/canopy, UNSAFE_VEGETATION class |
| Approach | Forward-flight landing sequence with the carrier and supervisor |

The gear controller stows on `/intelligence/abort`. The supervisor requests deployment at or below its decision height when abort is clear. This differs from the hardware demo's manual, latched deployment policy.

## 4. Data and Classifier

From the root, use a separate sourced ROS terminal while the matching static scenario runs:

```bash
python3 simulation/scripts/record_dataset.py --help
python3 simulation/scripts/record_dataset.py --world alpha --samples 500
python3 simulation/scripts/train_classifier.py
```

The recorder appends to `simulation/data/dataset.csv` by default. Select the label matching the running world. Training reads that dataset and overwrites the generated header under `simulation/src/intelligence_domain/include/intelligence_domain/`. It uses a stratified 75/25 split, random seed 42 and a depth limit of four. Rebuild after changing the header. Neither training nor recording is part of ordinary demo startup.

The legacy CSV column names do not all match the current runtime features. Read [the feature mapping](docs/sensors_and_physics.md) before collecting new data or interpreting the classifier. Historical classification scores should not be assumed to validate a newly generated dataset.

## 5. Checks

```bash
python3 -m unittest discover -s simulation/tests -p 'test_*.py'
python3 simulation/scripts/verify_system.py
ros2 run intelligence_domain benchmark_inference
```

The path tests run without ROS or Gazebo. The scenario verification requires both and explicitly disables flight so the supervisor does not compete with its manual deployment request. The existing verification script force-terminates Gazebo/ROS processes by name during cleanup. Run it in a dedicated simulation session with no unrelated jobs using those processes.

Read [testing and evidence](docs/testing_and_validation.md) for what each check establishes. Host timing is not an embedded-controller benchmark.

## 6. Guides

- [Architecture](docs/architecture.md)
- [Sensors and implemented features](docs/sensors_and_physics.md)
- [ROS interfaces and sequence](docs/ros_interfaces.md)
- [Testing and evidence](docs/testing_and_validation.md)
- [Implementation scope](docs/work_breakdown.md)
- [Original design reference](docs/design/original_simulation_specification.md)

[Back to project overview](../README.md)
