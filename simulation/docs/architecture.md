# Simulation Architecture

ROS nodes run on the simulation host and separate sensing, classification and control responsibilities. The original board plan assigned these domains to Pico 2 W, Pi Zero W and Arduino Uno. The current physical prototype instead performs sensing and rule-based control on one Uno.

| Component | Implemented responsibility |
| :--- | :--- |
| Gazebo and `ros_gz_bridge` | Simulated rig, range scans, IMU, clock and joint commands |
| `sensor_processor` | Latest range values, approximate slope and seven runtime features, nominal 50 Hz wall timer |
| `terrain_classifier` | Generated C++ decision tree on feature arrival, verdict and abort publications |
| `attitude_estimator` | Pitch/roll complementary filter on IMU arrival |
| `stabilator_controller` | Pitch PID and bounded joint command |
| `landing_gear_controller` | Manual/supervisor request, abort override and rate-limited trajectory |
| `flight_supervisor` | Arming, approach, flare, touchdown and go-around decisions |
| `flight_carrier.py` | Flight-rail motion when flight mode is enabled |

The launcher enables the carrier and supervisor with `flight:=true`. Static verification uses `flight:=false` so its gear request has a single owner. `flight_dynamics_node` is built by the control package but is not started by the current launcher.

## Control Boundaries

An asserted abort makes the simulated gear controller target the stowed angle regardless of deployment request. The hardware uses a separate latched policy described in [the comparison](../../docs/implementation-comparison.md).

Separate ROS nodes do not establish avionics isolation, hard real-time deadlines or worst-case latency guarantees. The generated tree has no explicit dynamic allocation in its prediction function; that property does not imply zero runtime jitter for the complete ROS/Gazebo system. Sensor fallback values and missing freshness checks limit the simulation's fault handling.

[Simulation setup](../README.md) / [Project overview](../../README.md)
