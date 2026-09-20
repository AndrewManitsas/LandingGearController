# ROS 2 Interfaces & Actuation State Machine

## 1. Active Topic Specifications

### Sensor & Perception Interfaces
| Topic Name | Message Type | Publisher Domain | Subscriber Domain | Rate / QoS | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/sensing/imu/data_raw` | `sensor_msgs/msg/Imu` | Gazebo Bridge | `attitude_estimator` | 100 Hz (Best Effort) | Raw 6-DOF linear accelerations and angular velocities |
| `/sensing/sonar/scan` | `sensor_msgs/msg/LaserScan` | Gazebo Bridge | `sensing_domain` | 40 Hz (Best Effort) | Ventral ultrasonic range return (conical beam profile) |
| `/sensing/tof/fore` | `sensor_msgs/msg/LaserScan` | Gazebo Bridge | `sensing_domain` | 50 Hz (Best Effort) | Fore optical rangefinder distance |
| `/sensing/tof/port` | `sensor_msgs/msg/LaserScan` | Gazebo Bridge | `sensing_domain` | 50 Hz (Best Effort) | Aft-left optical rangefinder distance |
| `/sensing/tof/starboard` | `sensor_msgs/msg/LaserScan` | Gazebo Bridge | `sensing_domain` | 50 Hz (Best Effort) | Aft-right optical rangefinder distance |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo Bridge | All Nodes | Continuous | Simulation clock for synchronization (`use_sim_time: true`) |

### Flight & Actuator Control Interfaces
| Topic Name | Message Type | Publisher Domain | Subscriber Domain | Rate / QoS | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/control/attitude` | `geometry_msgs/msg/Vector3Stamped` | `attitude_estimator` | `stabilator_controller`, GCS | 100 Hz (Reliable) | Filtered Euler angles (`x`: roll $\phi$, `y`: pitch $\theta$) in radians |
| `/control/pitch_setpoint` | `std_msgs/msg/Float64` | GCS / Mission Planner | `stabilator_controller` | Latched | Target pitch angle in radians (default: $0.0\text{ rad}$) |
| `/control/elevator_cmd` | `std_msgs/msg/Float64` | `stabilator_controller` | Gazebo Bridge | 100 Hz (Reliable) | Stabilator joint position command ($[-0.4363, +0.4363]\text{ rad}$) |
| `/control/gear_deploy` | `std_msgs/msg/Bool` | `supervisory_domain` | `landing_gear_controller` | Latched | High-level deploy trigger (`true` = deploy, `false` = stow) |
| `/control/gear_cmd` | `std_msgs/msg/Float64` | `landing_gear_controller` | Gazebo Bridge | 50 Hz (Reliable) | Rate-limited gear joint trajectory command ($[0.0, 1.5708]\text{ rad}$) |
| `/control/gear_status` | `std_msgs/msg/String` | `landing_gear_controller` | `supervisory_domain` | 50 Hz (Reliable) | Discrete actuator state (`STOWED`, `DEPLOYING`, `DEPLOYED`, `RETRACTING`) |

### Intelligence & Classification Interfaces (Phase 4)
| Topic Name | Message Type | Publisher Domain | Subscriber Domain | Rate / QoS | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/sensing/features` | `std_msgs/msg/Float32MultiArray` | `sensing_domain` | `intelligence_domain` | 40 Hz | Synchronous 7D extracted terrain feature vector |
| `/intelligence/verdict` | `std_msgs/msg/Int8` | `intelligence_domain` | `supervisory_domain` | 10 Hz | Discrete classification: `1 = SAFE`, `0 = UNSAFE` |

---

## 2. Coordinate Frames (TF2 Hierarchy)

```text
base_link (UAV Body Center)
  |-- horizontal_stabilizer_joint (Revolute joint around Y-axis)
  |     `-- horizontal_stabilizer (T-tail elevator surface)
  |-- landing_gear_joint (Revolute joint around Y-axis)
  |     `-- landing_gear_leg (Main landing gear strut and wheel)
  |-- sensor_plate (Rigid frame on ventral bay)
        |-- tof_fore_frame (Offset: +0.06m, 0.0m)
        |-- tof_port_frame (Offset: -0.03m, +0.052m)
        |-- tof_starboard_frame (Offset: -0.03m, -0.052m)
        |-- sonar_frame (Central: 0.0m, 0.0m)
        `-- imu_sensor_link (Rigidly coupled to airframe center of gravity)
```

## 3. Landing Gear Actuator State Machine (FSM)

```text
      +---------------------+
      |      STOWED         |<------------------------------------+
      +---------------------+                                     |
                 |                                                |
                 | [gear_deploy == true]                          | [gear_deploy == false]
                 v                                                |
      +---------------------+                                     |
      |      DEPLOYING      |                                     |
      +---------------------+                                     |
                 |                                                |
                 | [Position >= 1.5708 rad (90 deg)]              |
                 v                                                |
      +---------------------+                                     |
      |      DEPLOYED       |-------------------------------------+
      +---------------------+
```

---

[**Back to main README**](../README.md)