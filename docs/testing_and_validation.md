# Verification, Validation & Benchmarking Plan

## 1. Synthetic Test Environments (Gazebo)

* **Testbed Alpha (Safe):** Smooth, planar concrete world with inclination $\beta \le 5^\circ$ and rigid acoustic reflection ($\Delta d \approx 0$).
* **Testbed Beta (Unsafe — Slopes & Curbs):** Rigid planes featuring gradients $\beta \in [10^\circ, 15^\circ, 20^\circ, 30^\circ]$ and discrete step drop-offs ($>0.10\text{ m}$).
* **Testbed Gamma (Unsafe — Soft Vegetation):** Irregular terrain meshes with high-frequency elevation variance ($\pm 0.08\text{ m}$) producing acoustic-optical discrepancies ($\Delta d < -0.05\text{ m}$).

---

## 2. Validation Acceptance Criteria

| Benchmark Metric | Target Threshold | Validation Method |
| :--- | :--- | :--- |
| **Pitch Stability Under Gusts** | Residual sensor plate error $\le \pm 3.0^\circ$ | Injected $2.0\text{ m/s}$ cross-wind step input in Gazebo during vertical descent. |
| **False-Safe Classification Rate** | **0.0%** (Zero tolerance for false positives on unsafe ground) | 1,000 descent trials evaluated across Testbeds Beta and Gamma. |
| **Safe Terrain Recall** | $\ge 98.0\%$ detection rate over Testbed Alpha | Descent tests evaluated between $0.4\text{ m} \le h \le 1.5\text{ m}$. |
| **Deployment Timing** | Gear deployed and locked at $h \ge 0.2\text{ m}$ | High-speed descent trial ($v_z = 0.5\text{ m/s}$) over Testbed Alpha. |
| **Total Decision Loop Latency** | $\le 50\text{ ms}$ (20 Hz execution ceiling) | Latency profile measured from raw sensor message ingestion to `/intelligence/verdict` publication. |

---

[**Back to main README**](../README.md)