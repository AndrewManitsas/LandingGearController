# Verification, Validation & Benchmarking Plan

## 1. Synthetic Test Environments (Gazebo)

* **Testbed Alpha (`worlds/testbed_alpha.sdf`):** Smooth, planar concrete runway ($0^\circ$ grade) with rigid acoustic reflection ($\Delta d \approx 0$). Labeled `SAFE` (`0`).
* **Testbed Beta (`worlds/testbed_beta.sdf`):** $20^\circ$ inclined ramp ($0.349\text{ rad}$) and discontinuous $0.15\text{ m}$ stepped curbs. Labeled `UNSAFE_SLOPE` (`1`).
* **Testbed Gamma (`worlds/testbed_gamma.sdf`):** Irregular surface mounds ($\pm 0.08\text{ m}$ elevation variance) with elevated central canopy causing optical penetration ($\Delta d < -0.05\text{ m}$). Labeled `UNSAFE_VEGETATION` (`2`).

---

## 2. Quantitative Acceptance Criteria

| Benchmark Metric | Target Threshold | Validation Result | Status |
| :--- | :--- | :--- | :--- |
| **Pitch Stability Under Perturbation** | Dynamic error $\le \pm 3.0^\circ$ | Stabilator PID restores level pitch under descent disturbances. | **PASS** |
| **False-Safe Classification Rate** | **0.0%** (Zero tolerance for false-safe deployment) | 0 false-safe classifications across Beta and Gamma runs. | **PASS** |
| **Safe Terrain Recall** | $\ge 98.0\%$ detection rate on Testbed Alpha | 100% successful gear deployments over Testbed Alpha. | **PASS** |
| **Safety Interlock Actuation** | Gear deployment locked at $0.0\text{ rad}$ when aborted | Interlock successfully stows gear upon detecting unsafe slope or canopy. | **PASS** |
| **Edge AI Inference Latency** | $< 1.0\ \mu\text{s}$ execution ceiling | Mean latency: $\approx 14.2\text{ ns}$ (P99: $28.0\text{ ns}$) on x86_64 host. | **PASS** |

---

## 3. Automated Regression Harness (`scripts/verify_system.py`)

The automated regression test suite orchestrates multi-scenario validation:
1. Boots Gazebo headlessly (`gz sim -r -s`) to prevent rendering overhead.
2. Waits 5.0 seconds for physics settling, touchdown ground contact, and sensor filter convergence.
3. Asserts the steady-state terrain classification verdict and abort flag.
4. Commands gear deployment and evaluates whether the safety interlock properly permits actuation or forces stowing.
5. Cleans up process groups to prevent orphaned background nodes.

Execute the suite:
```bash
python3 scripts/verify_system.py
```

Expected output:
```text
==================================================
           SYSTEM VERIFICATION SUMMARY            
==================================================
  Testbed ALPHA  : PASSED (SUCCESS)
  Testbed BETA   : PASSED (SUCCESS)
  Testbed GAMMA  : PASSED (SUCCESS)
==================================================
```

## 4. Deterministic Latency Benchmarking

The compiled C++ decision tree inference executable benchmarks execution time over 1,000,000 iterations using `std::chrono::high_resolution_clock:`

```bash
ros2 run intelligence_domain benchmark_inference
```

```text
========================================================
  EDGE AI INFERENCE LATENCY BENCHMARK (1000000 runs)
========================================================
  Min Latency:         6.00 ns
  Mean Latency:       14.20 ns
  Median (P50):       13.00 ns
  95th Percentile:    21.00 ns
  99th Percentile:    28.00 ns
  Max Latency:        65.00 ns
--------------------------------------------------------
  Estimated Execution Frequency: > 70,000,000 Hz
========================================================
```

---

[**Back to main README**](../README.md)