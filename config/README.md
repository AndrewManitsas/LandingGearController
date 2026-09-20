# Bridge Configuration & Topic Interface

This directory contains the runtime configuration for bridging telemetry and actuator commands between **Gazebo Harmonic** and **ROS 2 Jazzy** via `ros_gz_bridge`.

---

## 1. Topic Mapping Table

| Subsystem / Interface | Gazebo Topic | ROS 2 Topic | ROS 2 Message Type | Direction |
| :--- | :--- | :--- | :--- | :--- |
| **Pitch Stabilator** | `/model/uav_landing_rig/joint/horizontal_stabilizer_joint/0/cmd_pos` | `/control/elevator_cmd` | `std_msgs/msg/Float64` | `ROS_TO_GZ` |
| **Landing Gear** | `/model/uav_landing_rig/joint/landing_gear_joint/0/cmd_pos` | `/control/gear_cmd` | `std_msgs/msg/Float64` | `ROS_TO_GZ` |
| **9-DoF IMU** | `/model/uav_landing_rig/imu` | `/sensing/imu/data_raw` | `sensor_msgs/msg/Imu` | `GZ_TO_ROS` |
| **Ventral Sonar** | `/model/uav_landing_rig/sonar_down` | `/sensing/sonar/scan` | `sensor_msgs/msg/LaserScan` | `GZ_TO_ROS` |
| **Fore ToF 1** | `/model/uav_landing_rig/tof_1` | `/sensing/tof/fore` | `sensor_msgs/msg/LaserScan` | `GZ_TO_ROS` |
| **Port ToF 2** | `/model/uav_landing_rig/tof_2` | `/sensing/tof/port` | `sensor_msgs/msg/LaserScan` | `GZ_TO_ROS` |
| **Starboard ToF 3** | `/model/uav_landing_rig/tof_3` | `/sensing/tof/starboard` | `sensor_msgs/msg/LaserScan` | `GZ_TO_ROS` |

---

## 2. Actuator Command Limits

Target commands published to the control topics must stay within the joint limits defined in the URDF:

| Joint | ROS 2 Topic | Min Limit | Max Limit | Physical Behavior |
| :--- | :--- | :--- | :--- | :--- |
| **Horizontal Stabilator** | `/control/elevator_cmd` | `-0.4363 rad` (-25°) | `+0.4363 rad` (+25°) | Pitch trim (trailing edge down / up) |
| **Landing Gear** | `/control/gear_cmd` | `0.0000 rad` (0°) | `+1.5708 rad` (+90°) | `0.0` = Stowed/Retracted, `1.5708` = Fully Deployed |

---

## 3. Launching the Bridge

Execute the parameter bridge from the repository root:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=config/bridge_config.yaml
```