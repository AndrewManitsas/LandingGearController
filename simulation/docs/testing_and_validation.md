# Simulation Testing and Evidence

## 1. Offline Path Checks

From the simulation folder:

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
```

These checks exercise relocated resource paths and confirm that the verification harness requests static mode. They do not start Gazebo or validate flight behaviour.

## 2. Scenario Verification

After building and sourcing the workspace, run:

```bash
python3 scripts/verify_system.py
```

The harness launches Alpha, Beta and Gamma headlessly with `flight:=false`, waits five seconds for startup, samples the verdict/abort, sends manual deployment requests and waits 3.5 seconds for the gear trajectory. It checks class 0/1/2, abort false/true/true, and generated gear status above 1.2 rad for Alpha or below 0.05 rad for the unsafe worlds. This does not measure physical gear position, worst-case fault handling, pitch recovery or touchdown quality.

The existing cleanup uses forceful process-name termination and may stop unrelated Gazebo/ROS jobs. Use a dedicated simulation session. Its fixed startup wait and lack of process-readiness checks are limitations of this harness.

## 3. Previously Reported Results

Earlier documentation reports Alpha/Beta/Gamma passes and an x86 host inference mean of approximately 14.2 ns (P99 28 ns). The supplied presentation reports 375/375 correct samples on a synthetic test split, with 125 per class. These are historical reported results, not fresh measurements of the merged checkout or guarantees on embedded hardware. Preserve the run configuration and logs when reproducing them.

The dataset has 1,500 synthetic samples labelled by world. Training uses a random stratified 75/25 split, seed 42 and maximum depth four. Correlated samples and per-world labels make the classification score optimistic. The compiled tree uses two of seven feature slots. Runtime semantics differ from some legacy column names; consult [the feature guide](sensors_and_physics.md).

## 4. Host Benchmark

```bash
ros2 run intelligence_domain benchmark_inference
```

The executable times prediction on three synthetic vectors over one million iterations. Clock resolution, compiler optimisation, host scheduling and measurement overhead affect results. It does not measure the ROS pipeline, servo response, microcontroller execution or a worst-case deadline. Its representative vectors carry legacy feature descriptions and are timing inputs rather than a correctness test suite.

## 5. Hardware Evidence

The separate bench demonstration exercises IMU attitude, range validity, qualification, manual deployment, retention after readiness loss and explicit stow. The Python monitor has offline tests under `hardware/tools/`. Physical servo motion, terrain classification, sensor-array geometry and aircraft landing remain outside that evidence.

[Simulation setup](../README.md) / [Project overview](../../README.md)
