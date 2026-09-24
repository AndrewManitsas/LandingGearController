# ROS 2 Interfaces & Actuation State Machine

## 1. Active Topic Specifications

### Sensor & Perception Interfaces
| Topic Name | Message Type | Publisher Domain | Subscriber Domain | Rate / QoS | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/sensing/imu/data_raw` | `sensor_msgs/msg/Imu` | Gazebo Bridge | `attitude_estimator` | 100 Hz (Best Effort) | Raw 6-DOF linear accelerations and angular velocities |
| `/sensing/sonar/scan` | `sensor_msgs/msg/LaserScan` | Gazebo Bridge | `sensor_processor` | 40 Hz (Best Effort) | Ventral ultrasonic range return (conical beam profile) |
| `/sensing/tof/fore` | `sensor_msgs/msg/LaserScan` | Gazebo Bridge | `sensor_processor` | 40 Hz (Best Effort) | Fore optical rangefinder distance |
| `/sensing/tof/port` | `sensor_msgs/msg/LaserScan` | Gazebo Bridge | `sensor_processor` | 40 Hz (Best Effort) | Port (aft-left) optical rangefinder distance |
| `/sensing/tof/starboard` | `sensor_msgs/msg/LaserScan` | Gazebo Bridge | `sensor_processor` | 40 Hz (Best Effort) | Starboard (aft-right) optical rangefinder distance |
| `/sensing/altitude_agl` | `std_msgs/msg/Float64` | `sensor_processor` | Telemetry / Monitor | 40 Hz (Reliable) | Temporal moving-average ground clearance ($h_{\text{AGL}}$) |
| `/sensing/features` | `std_msgs/msg/Float32MultiArray` | `sensor_processor` | `terrain_classifier` | 40 Hz (SensorData) | Synchronous 7D extracted terrain feature vector |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo Bridge | All Nodes | Continuous | Simulation clock for synchronization (`use_sim_time: true`) |

### Flight & Actuation Control Interfaces
| Topic Name | Message Type | Publisher Domain | Subscriber Domain | Rate / QoS | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/control/attitude` | `geometry_msgs/msg/Vector3` | `attitude_estimator` | `stabilator_controller` | 100 Hz (Reliable) | Filtered Euler angles (`x`: roll $\phi$, `y`: pitch $\theta$) in radians |
| `/control/pitch_setpoint` | `std_msgs/msg/Float64` | GCS / Mission Planner | `stabilator_controller` | Latched | Target pitch angle in radians (default: $0.0\text{ rad}$) |
| `/control/elevator_cmd` | `std_msgs/msg/Float64` | `stabilator_controller` | Gazebo Bridge | 100 Hz (Reliable) | Stabilator joint position command ($[-0.4363, +0.4363]\text{ rad}$) |
| `/control/gear_deploy` | `std_msgs/msg/Bool` | Operator / Mission | `landing_gear_controller` | Latched | High-level deploy trigger (`true` = deploy, `false` = stow) |
| `/control/gear_cmd` | `std_msgs/msg/Float64` | `landing_gear_controller` | Gazebo Bridge | 50 Hz (Reliable) | Rate-limited gear joint command ($[0.0, 1.5708]\text{ rad}$) |
| `/control/gear_status` | `std_msgs/msg/Float64` | `landing_gear_controller` | Telemetry / Monitor | 50 Hz (Reliable) | Real-time gear joint position telemetry in radians |

### Intelligence & Classification Interfaces
| Topic Name | Message Type | Publisher Domain | Subscriber Domain | Rate / QoS | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/intelligence/verdict` | `std_msgs/msg/Int32` | `terrain_classifier` | Mission Supervisor | 40 Hz (Reliable) | Classification: `0 = SAFE`, `1 = UNSAFE_SLOPE`, `2 = UNSAFE_VEG` |
| `/intelligence/abort` | `std_msgs/msg/Bool` | `terrain_classifier` | `landing_gear_controller` | 40 Hz (Reliable) | Safety interlock override (`true` = abort/inhibit deployment) |

---

## 2. Coordinate Frames (TF2 Hierarchy)

```text
base_link (UAV Airframe Center of Mass)
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

## 3. Landing Gear Actuator State Machine (FSM)

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
       |   FORCE INHIBIT / STOW     |   |   EXECUTE TRAJECTORY       |
       |   Target = 0.0 rad         |   |   Target = 1.5708 rad      |
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

[**Back to main README**](../README.md)