# ROS 2 Interfaces & Actuation State Machine

## 1. Active Topic Specifications

### Sensor & Perception Interfaces
| Topic Name | Message Type | Publisher Domain | Subscriber Domain | Rate / QoS | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/sensing/imu/data_raw` | `sensor_msgs/msg/Imu` | Gazebo Bridge | `attitude_estimator` | 100 Hz (Best Effort) | Raw 6-DOF linear accelerations and angular velocities |
| `/sensing/sonar/scan` | `sensor_msgs/msg/LaserScan` | Gazebo Bridge | `sensor_processor` | nominal 50 Hz (Best Effort) | Ventral ultrasonic range return (conical beam profile) |
| `/sensing/tof/fore` | `sensor_msgs/msg/LaserScan` | Gazebo Bridge | `sensor_processor` | nominal 50 Hz (Best Effort) | Fore optical rangefinder distance |
| `/sensing/tof/port` | `sensor_msgs/msg/LaserScan` | Gazebo Bridge | `sensor_processor` | nominal 50 Hz (Best Effort) | Port (aft-left) optical rangefinder distance |
| `/sensing/tof/starboard` | `sensor_msgs/msg/LaserScan` | Gazebo Bridge | `sensor_processor` | nominal 50 Hz (Best Effort) | Starboard (aft-right) optical rangefinder distance |
| `/sensing/altitude_agl` | `std_msgs/msg/Float64` | `sensor_processor` | Telemetry / Monitor | nominal 50 Hz (Reliable) | Mean of the latest three ToF ranges ($h_{\text{AGL}}$) |
| `/sensing/features` | `std_msgs/msg/Float32MultiArray` | `sensor_processor` | `terrain_classifier` | nominal 50 Hz (Reliable publisher, SensorData subscriber) | Seven feature slots from the latest retained scans; see sensors_and_physics.md |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo Bridge | All Nodes | Continuous | Simulation clock for synchronization (`use_sim_time: true`) |

### Flight & Actuation Control Interfaces
| Topic Name | Message Type | Publisher Domain | Subscriber Domain | Rate / QoS | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/control/attitude` | `geometry_msgs/msg/Vector3Stamped` | `attitude_estimator` | `stabilator_controller` | 100 Hz (Reliable) | Filtered Euler angles (`x`: roll $\phi$, `y`: pitch $\theta$) in radians |
| `/control/pitch_setpoint` | `std_msgs/msg/Float64` | Flight supervisor / manual ROS publisher | `stabilator_controller` | Supervisor nominal 50 Hz / manual publisher-defined | Target pitch angle in radians (default: $0.0\text{ rad}$) |
| `/control/elevator_cmd` | `std_msgs/msg/Float64` | `stabilator_controller` | Gazebo Bridge | 100 Hz (Reliable) | Stabilator joint position command ($[-0.4363, +0.4363]\text{ rad}$) |
| `/control/gear_deploy` | `std_msgs/msg/Bool` | Flight supervisor / manual ROS publisher | `landing_gear_controller` | Supervisor nominal 50 Hz / manual publisher-defined | High-level deploy trigger (`true` = deploy, `false` = stow) |
| `/control/gear_cmd` | `std_msgs/msg/Float64` | `landing_gear_controller` | Gazebo Bridge | 50 Hz (Reliable) | Rate-limited gear joint command ($[0.0, 1.5708]\text{ rad}$) |
| `/control/gear_status` | `std_msgs/msg/Float64` | `landing_gear_controller` | Telemetry / Monitor | 50 Hz (Reliable) | Controller-generated trajectory angle in radians, not joint feedback |

### Intelligence & Classification Interfaces
| Topic Name | Message Type | Publisher Domain | Subscriber Domain | Rate / QoS | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/intelligence/verdict` | `std_msgs/msg/Int32` | `terrain_classifier` | Mission Supervisor | nominal 50 Hz (Reliable) | Classification: `0 = SAFE`, `1 = UNSAFE_SLOPE`, `2 = UNSAFE_VEG` |
| `/intelligence/abort` | `std_msgs/msg/Bool` | `terrain_classifier` | `landing_gear_controller` | nominal 50 Hz (Reliable) | Safety interlock override (`true` = abort/inhibit deployment) |

---

## 2. Original Coordinate-Frame Sketch

```text
base_link (proposed frame sketch; inspect current URDF for actual geometry)
  |-- horizontal_stabilizer_joint (Revolute joint around Y-axis)
  |     `-- horizontal_stabilizer (T-tail elevator surface)
  |-- landing_gear_joint (Revolute joint around Y-axis)
  |     `-- landing_gear_leg (Main landing gear strut and wheel)
  |-- sensor_plate (Rigid mounting plane on ventral fuselage)
        |-- tof_fore_frame (Offset: +0.06m, 0.0m, -0.05m)
        |-- tof_port_frame (Offset: -0.03m, +0.052m, -0.05m)
        |-- tof_starboard_frame (Offset: -0.03m, -0.052m, -0.05m)
        |-- sonar_frame (Central: 0.0m, 0.0m, -0.05m)
        `-- imu_sensor_link (Rigidly coupled at airframe center of gravity)
```

## 3. Landing Gear Request and Abort Logic

```text
                  +---------------------------+
                  |  Operator Command         |
                  |  (/control/gear_deploy)   |
                  +---------------------------+
                                |
                                v
                   [ Is /intelligence/abort == true? ]
                               /         \
                      YES     /           \     NO
                             v             v
       +----------------------------+   +----------------------------+
       |   FORCE INHIBIT / STOW     |   |   FOLLOW REQUEST       |
       |   Target = 0.0 rad         |   |   Deploy: 1.5708 / Stow: 0.0      |
       +----------------------------+   +----------------------------+
                     \                             /
                      \                           /
                       v                         v
               +-----------------------------------------+
               |  Rate Limiter (max_step = rate * dt)    |
               |  Speed limit: 0.50 rad/s (~3.14s sweep) |
               +-----------------------------------------+
                                   |
                                   v
                      /control/gear_cmd (Float64)
```

---

[Simulation setup](../README.md) / [Project overview](../../README.md)

## 4. Flight Mode

`flight_supervisor` publishes `/control/pitch_setpoint`, `/control/gear_deploy`, `/flight/touchdown` (Bool) and `/flight/supervisor_state` (String) on its nominal 50 Hz wall timer. Its states are ARMING, APPROACH_DESCENT, FLARE, TOUCHDOWN and GO_AROUND. Default decision/flare/touchdown heights are 0.50/0.26/0.16 m. These thresholds are model settings, not confirmed physical touchdown measurements. `flight_carrier.py` uses `/flight/x_cmd` and `/flight/z_cmd` (Float64) for the Gazebo rail.

The current bridge maps `/clock` as well as sensor and joint topics. Inspect [bridge_config.yaml](../config/bridge_config.yaml) for exact Gazebo paths. ROS QoS depth 10 alone does not make a topic latched. The gear controller follows a false deploy request by stowing even when abort is clear.

Known type mismatch: the estimator publishes Vector3Stamped on /control/attitude, while flight_supervisor currently subscribes using Vector3. Its pitch_angle_ member does not drive the present state decisions, but that subscription needs alignment before it can consume attitude.
