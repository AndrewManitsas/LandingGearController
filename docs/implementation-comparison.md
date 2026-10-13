# Simulation and Hardware Comparison

Both implementations explore whether sensing can inform landing gear commands, but their inputs, decisions and actuation differ.

| Responsibility | Simulation | Hardware demo |
| :--- | :--- | :--- |
| Attitude | ROS IMU, radians, complementary filter with default alpha 0.90 | MPU6500, degrees, measured dt and 0.50 s time constant |
| Range | Three optical scans plus sonar scan | One VL53L0X at nominal 20 ranges/s |
| Decision | Generated C++ tree using runtime features 6 and 1 | 100-500 mm, tilt within 10 degrees, ten readings within 20 mm spread, range age <=250 ms, then one second qualification |
| Stabilizer | PID command to simulated joint | Proportional virtual command centred at 90 degrees, limited to 65-115 |
| Deployment | Supervisor or manual ROS request, abort forces stow | D accepted only while READY, S explicitly stows |
| Loss of readiness | Unsafe classification asserts abort | Warning, already deployed gear stays latched |
| Feedback | Gear status reports the controller's generated trajectory | Displayed gear state is a command, no position sensor |
| Monitoring | ROS topics, Gazebo and optional telemetry scripts | USB reports at 2 Hz or raw IMU frames at 20 Hz, Python desktop app |

The simulation's flight supervisor uses range thresholds for its approach and touchdown states. The hardware has no touchdown detection or automatic landing supervisor. The boards and simulator are not connected as HIL. The one-ToF hardware cannot estimate a terrain plane or classify surface material.

The simulation documentation distinguishes the original proposed feature vector from what `sensor_processor.cpp` currently publishes. No claim of certified isolation, worst-case execution time or physical flight safety follows from these prototypes.

[Project overview](../README.md)
