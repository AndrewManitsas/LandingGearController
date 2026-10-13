# ROS 2 / Gazebo Simulation

This implementation explores the broader landing-assistance concept with three simulated ToFs, sonar, an IMU, a compiled terrain classifier, pitch control and rate-limited gear commands. It runs on the simulation host. The embedded boards in the original proposal describe intended responsibilities, not deployed hardware.

## 1. Requirements

Ubuntu 24.04 / WSL Ubuntu, ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_bridge`, `ros_gz_sim`, colcon and the ROS dependencies declared in `src/*/package.xml`. Dataset training uses Python 3 with numpy, pandas and scikit-learn. No hardware serial connection is required.

## 2. Build

Run from the simulation folder in WSL / Ubuntu. Install ROS 2 Jazzy and configure its apt repositories first, then:

```bash
cd ~/repos/LandingGearController/simulation
make setup
```

Setup installs ROS/Gazebo integration, colcon, rosdep and Python training dependencies using apt, initializes rosdep if needed, resolves package dependencies and builds the workspace. It requires sudo and network access. It does not install ROS itself or configure Windows hardware tooling.

For subsequent use:

```bash
make help
make check
make build
make run
make alpha
make beta GUI=false
make beta FLIGHT=false
make gamma
make run WORLD=approach GUI=true FLIGHT=true
make test
make benchmark
make verify
make clean
make rebuild
```

Build and launch targets source ROS automatically. Run/benchmark/verify perform an incremental build and source this checkout's install environment. Alpha/Beta/Gamma presets enable flight by default, matching the manual launch default. Use `make beta FLIGHT=false` for static sensing checks. Run launches approach with flight enabled by default. Exit a running simulation with Ctrl+C before starting another.

Clean deletes only the simulation workspace's `build/`, `install/` and `log/` directories. It preserves source, datasets, presentation and Python environments, and refuses cleanup targets that are symlinks or regular files. Stop active simulation/build jobs before cleaning. Rebuild cleans first and then builds. Offline test and clean require only Python and make. Build and run do not install packages.

The manual equivalent remains:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
```

Generated directories stay inside `simulation/`. From the repository root, use `make -C simulation <target>`. Make targets handle sourcing in their own shell, so they do not modify your terminal environment. For manual ROS commands below, source ROS and `install/setup.bash` in that terminal. Resource paths resolve relative to the launcher, independently of checkout location.

## 3. Launch and Scenarios

From the simulation folder:

```bash
ros2 launch launch/system_launch.py world:=approach gui:=true flight:=true
ros2 launch launch/system_launch.py world:=alpha gui:=true flight:=true
ros2 launch launch/system_launch.py world:=beta gui:=true flight:=true
ros2 launch launch/system_launch.py world:=gamma gui:=true flight:=true
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

From the simulation folder, use a separate sourced ROS terminal while the matching static scenario runs:

```bash
python3 scripts/record_dataset.py --help
python3 scripts/record_dataset.py --world alpha --samples 500
python3 scripts/train_classifier.py
```

The recorder appends to `data/dataset.csv` by default. Select the label matching the running world. Training reads that dataset and overwrites the generated header under `src/intelligence_domain/include/intelligence_domain/`. It uses a stratified 75/25 split, random seed 42 and a depth limit of four. Rebuild after changing the header. Neither training nor recording is part of ordinary demo startup.

The legacy CSV column names do not all match the current runtime features. Read [the feature mapping](docs/sensors_and_physics.md) before collecting new data or interpreting the classifier. Historical classification scores should not be assumed to validate a newly generated dataset.

## 5. Checks

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 scripts/verify_system.py
ros2 run intelligence_domain benchmark_inference
```

The path tests run without ROS or Gazebo. The scenario verification requires both and explicitly disables flight so the supervisor does not compete with its manual deployment request. Verification waits for valid samples from all range sensors and fresh features before evaluating results. It saves each launch log under `log/verification/` and stops only the process group it launched. Stop any separately running simulation before verification to avoid shared-world/topic collisions.

Read [testing and evidence](docs/testing_and_validation.md) for what each check establishes. Host timing is not an embedded-controller benchmark.

## 6. Guides

- [Architecture](docs/architecture.md)
- [Sensors and implemented features](docs/sensors_and_physics.md)
- [ROS interfaces and sequence](docs/ros_interfaces.md)
- [Testing and evidence](docs/testing_and_validation.md)
- [Implementation scope](docs/work_breakdown.md)
- [Original design reference](docs/design/original_simulation_specification.md)

[Back to project overview](../README.md)
