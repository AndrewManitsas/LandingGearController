# ROS 2 Interfaces & Actuation State Machine

## 1. Topic Specifications

| Topic Name | Message Type | Publisher Domain | Subscriber Domain | Update Rate | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/range/tof_1` | `sensor_msgs/Range` | Gazebo Sim | Sensing Domain | 50 Hz | Fore ToF distance measurement |
| `/range/tof_2` | `sensor_msgs/Range` | Gazebo Sim | Sensing Domain | 50 Hz | Aft-left ToF distance measurement |
| `/range/tof_3` | `sensor_msgs/Range` | Gazebo Sim | Sensing Domain | 50 Hz | Aft-right ToF distance measurement |
| `/range/ultrasonic` | `sensor_msgs/Range` | Gazebo Sim | Sensing Domain | 40 Hz | Central acoustic return distance |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo Sim | Sensing / Control | 100 Hz | Raw 6-DOF linear accelerations and angular rates |
| `/sensing/features` | `std_msgs/Float32MultiArray` | Sensing Domain | Intelligence Domain | 40 Hz | Synchronous 7D extracted feature vector |
| `/control/attitude` | `geometry_msgs/Vector3` | Control Domain | Intelligence / GCS | 100 Hz | Filtered Euler angles ($\phi, \theta, \psi$) |
| `/control/pitch_cmd` | `std_msgs/Float32` | Control Domain | Gazebo Pitch Servo | 100 Hz | Angular position effort command for pitch trim |
| `/intelligence/verdict` | `std_msgs/Int8` | Intelligence Domain | Control Domain | 10 Hz | Discrete classification: 1 = SAFE, 0 = UNSAFE |
| `/control/gear_cmd` | `std_msgs/Float32` | Control Domain | Gazebo Gear Servo | Event | Landing gear command ($0^\circ = \text{Stowed}$, $90^\circ = \text{Deployed}$) |

---

## 2. Coordinate Frames (TF2 Hierarchy)

```text
base_link (UAV Body Center)
  |-- pitch_stabilizer_mount (Revolute joint around Y-axis)
        |-- sensor_plate (Rigid frame carrying sensor cluster)
        |     |-- tof_1_frame (Offset: +0.06m, 0.0m)
        |     |-- tof_2_frame (Offset: -0.03m, +0.052m)
        |     |-- tof_3_frame (Offset: -0.03m, -0.052m)
        |     |-- sonar_frame (Central: 0.0m, 0.0m)
        |     `-- imu_frame   (Rigidly coupled to plate)
        `-- landing_gear_link (Revolute joint around X/Y-axis)
```

## 3. Landing Gear Actuator State Machine (FSM)

```text
      +---------------------+
      |      RETRACTED      |<------------------------------------+
      +---------------------+                                     |
                 |                                                |
                 | [SAFE == 1] AND [Altitude <= H_deploy]         | [SAFE == 0] OR
                 v                                                | [Altitude > H_deploy]
      +---------------------+                                     |
      |      DEPLOYING      |                                     |
      +---------------------+                                     |
                 |                                                |
                 | [Servo Position Reached (t >= t_transit)]      |
                 v                                                |
      +---------------------+                                     |
      |   DEPLOYED (LOCKED) |-------------------------------------+
      +---------------------+
```

---

[**Back to main README**](../README.md)