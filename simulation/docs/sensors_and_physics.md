# Simulation Sensors and Implemented Features

This guide describes the current source code. The [original specification](design/original_simulation_specification.md) describes earlier intended algorithms that are not all implemented.

## 1. Attitude

`attitude_estimator.cpp` computes accelerometer roll with `atan2(ay, az)` and pitch with `atan2(-ax, sqrt(ay*ay + az*az))`. It integrates gyro rates in rad/s and combines them with the gravity estimate using default alpha 0.90. The first valid sample initializes attitude. Non-positive dt or a gap over 0.2 s uses a 0.01 s fallback. `/control/attitude` uses `Vector3Stamped`, with roll in x and pitch in y, in radians.

The hardware filter uses degrees and alpha calculated from a 0.50 s time constant and measured dt. Neither estimator establishes yaw or physical aircraft stabilization in the bench prototype.

## 2. Range Processing

The ToF callbacks take the minimum finite, in-range sample from each scan. Empty or wholly invalid ToF scans fall back to 0.85 m. The processor retains the most recent value for each sensor without enforcing synchronous arrival or age limits.

The sonar callback averages valid scan rays and computes their population variance. If no valid ray exists, it retains the previous sonar values. This is variance across a scan, not across time. ToF variance uses the three latest sensor distances with a denominator of three. Both variances have a floor of 0.00001 m². The current processor has no ten-sample temporal buffer.

## 3. Slope Approximation

The runtime calculates:

```text
mean_tof = (fore + port + starboard) / 3
pitch_slope = atan2(fore - (port + starboard) / 2, 0.070)
roll_slope = atan2(port - starboard, 0.078)
total_slope = sqrt(pitch_slope² + roll_slope²)
delta_d = sonar_mean - mean_tof
```

Distances are in metres and slopes in radians. The two baseline constants are the values in the code. Their consistency with the model geometry needs verification. The implementation does not use the proposed three-point cross-product normal or compensate the terrain estimate with filtered IMU attitude. A universal 15-degree rollover threshold is not part of the runtime classifier.

## 4. Runtime Feature Mapping

CSV/header names remain unchanged for compatibility. Some names describe proposed quantities rather than what the processor now supplies.

| Index | Legacy name | Actual runtime quantity | Unit |
| :--- | :--- | :--- | :--- |
| 0 | `mean_tof` | Mean of the latest three ToF distances | m |
| 1 | `spatial_variance` | Population variance of those three distances, with floor | m² |
| 2 | `delta_d` | Sonar mean minus mean ToF | m |
| 3 | `delta_d_variance` | Variance of valid sonar rays within one scan, with floor | m² |
| 4 | `beta_slope` | Total slope approximation | rad |
| 5 | `omega_norm` | Absolute roll-slope approximation, not IMU angular rate | rad |
| 6 | `tilt_error` | Total slope approximation again, not filtered attitude error | rad |

The processor's wall timer is 20 ms (nominal 50 Hz). The classifier responds to incoming features. The generated tree tests feature 6 against 0.137855 rad and then feature 1 against 0.000783 m². It returns SAFE for low feature 6, otherwise UNSAFE_SLOPE for low feature 1 and UNSAFE_VEGETATION for high feature 1. Class names reflect synthetic dataset labels, not verified surface identification.

Negative acoustic-optical differences can occur in the synthetic canopy scenario. No real-material classification or optical penetration experiment has been established by this implementation. Before retraining, record the actual feature definitions alongside the dataset and check that collection, training and inference use the same semantics.

[Simulation setup](../README.md) / [Project overview](../../README.md)
