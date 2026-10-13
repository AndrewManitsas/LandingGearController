# Simulation Implementation Scope

This inventory describes source components, not certification or a blanket validation status. Runtime evidence belongs in [testing and evidence](testing_and_validation.md). The physical implementation has its own [hardware guide](../../hardware/README.md).

## Existing Components

| Area | Repository implementation |
| :--- | :--- |
| Virtual rig | URDF rig, flight rail, simulated IMU/range sensors, stabilizer and gear joints |
| Terrain scenarios | Alpha, Beta, Gamma and approach worlds |
| ROS interfaces | Bridge configuration and three ROS packages |
| Attitude/control | Complementary filter, PID stabilator, rate-limited gear and abort override |
| Flight sequence | Carrier script and supervisor states |
| Perception | Latest-range aggregation and approximate slope, seven runtime feature slots |
| Classifier | Synthetic dataset, Python training and generated C++ tree |
| Tooling | Launch, recording, telemetry, path checks, scenario harness and host benchmark |

## Remaining Verification and Development

- Reproduce end-to-end scenario results on the merged layout and preserve logs.
- Align sensor geometry with slope baselines and verify feature semantics against data collection.
- Add simulation freshness/invalid-data handling before treating defaults as usable measurements.
- Evaluate independent scenarios rather than only random splits of correlated synthetic samples.
- Measure timing on intended hardware before making embedded performance claims.
- Integrate additional physical sensors and powered actuation only with suitable wiring, power and mechanics.

The hardware demo already implements a smaller sensing and landing sequence. It is neither the original three-board deployment nor a hardware-in-the-loop connection to Gazebo.

[Simulation setup](../README.md) / [Project overview](../../README.md)
