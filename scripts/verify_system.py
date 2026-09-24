#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float64, Int32

class VerificationEvaluator(Node):
    def __init__(self):
        super().__init__('verification_evaluator')
        self.verdict = None
        self.abort = None
        self.gear_status = 0.0

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.create_subscription(Int32, '/intelligence/verdict', self.verdict_cb, 10)
        self.create_subscription(Bool, '/intelligence/abort', self.abort_cb, 10)
        self.create_subscription(Float64, '/control/gear_status', self.gear_status_cb, 10)
        self.deploy_pub = self.create_publisher(Bool, '/control/gear_deploy', 10)

    def reset(self):
        self.verdict = None
        self.abort = None
        self.gear_status = 0.0

    def verdict_cb(self, msg: Int32):
        self.verdict = msg.data

    def abort_cb(self, msg: Bool):
        self.abort = msg.data

    def gear_status_cb(self, msg: Float64):
        self.gear_status = msg.data

    def trigger_deployment(self):
        msg = Bool()
        msg.data = True
        self.deploy_pub.publish(msg)

def clean_all_processes():
    # Terminate Gazebo physics engines
    subprocess.run(['killall', '-9', 'gz-sim-server', 'ruby', 'gz'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # Terminate orphaned ROS 2 nodes
    subprocess.run(['pkill', '-9', '-f',
                    'landing_gear_controller|sensor_processor|terrain_classifier|attitude_estimator|stabilator_controller|parameter_bridge|ros_gz'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)

def test_scenario(evaluator: VerificationEvaluator, world_name: str,
                  expected_verdict: int, expected_abort: bool, expect_deploy: bool):
    print(f"\n==================================================")
    print(f"  RUNNING TEST SCENARIO: {world_name.upper()}")
    print(f"==================================================")
    clean_all_processes()
    evaluator.reset()

    cmd = [
        'ros2', 'launch', 'launch/system_launch.py',
        f'world:={world_name}', 'gui:=false'
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

    # 1. Allow 5.0 seconds for Gazebo startup, model spawning (2.5s timer),
    # touchdown settling under gravity, and sensor buffer population.
    print("  Waiting 5.0s for model spawn, touchdown settling, and filter convergence...")
    settle_start = time.time()
    while time.time() - settle_start < 5.0:
        rclpy.spin_once(evaluator, timeout_sec=0.1)

    # 2. Check converged perception inference
    actual_verdict = evaluator.verdict
    actual_abort = evaluator.abort

    # 3. Command gear deployment
    print("  Commanding gear deployment (/control/gear_deploy = true)...")
    for _ in range(5):
        evaluator.trigger_deployment()
        rclpy.spin_once(evaluator, timeout_sec=0.05)

    # 4. Wait 3.5 seconds for rate-limited actuator trajectory (0.5 rad/s -> ~3.14s for 90 deg)
    act_start = time.time()
    while time.time() - act_start < 3.5:
        rclpy.spin_once(evaluator, timeout_sec=0.1)

    final_gear_pos = evaluator.gear_status

    # 5. Clean teardown of launched process group
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=2)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
    clean_all_processes()

    # Assertions
    verdict_ok = (actual_verdict == expected_verdict)
    abort_ok = (actual_abort == expected_abort)
    gear_ok = (final_gear_pos > 1.2) if expect_deploy else (final_gear_pos < 0.05)

    print(f"  Verdict: Expected={expected_verdict}, Received={actual_verdict} -> {'PASS' if verdict_ok else 'FAIL'}")
    print(f"  Abort:   Expected={expected_abort}, Received={actual_abort} -> {'PASS' if abort_ok else 'FAIL'}")
    print(f"  Gear:    Target={'DEPLOYED (>1.2)' if expect_deploy else 'INHIBITED (<0.05)'}, Actual={final_gear_pos:.3f} rad -> {'PASS' if gear_ok else 'FAIL'}")

    passed = verdict_ok and abort_ok and gear_ok
    status_str = "SUCCESS" if passed else "FAILED"
    return passed, status_str

def main():
    clean_all_processes()
    rclpy.init()
    evaluator = VerificationEvaluator()
    results = {}

    scenarios = [
        # (world, expected_verdict, expected_abort, expect_deploy)
        ('alpha', 0, False, True),   # Safe runway: deployment allowed
        ('beta',  1, True,  False),  # Unsafe slope: deployment inhibited
        ('gamma', 2, True,  False)   # Unsafe vegetation: deployment inhibited
    ]

    for world, exp_verdict, exp_abort, exp_deploy in scenarios:
        ok, msg = test_scenario(evaluator, world, exp_verdict, exp_abort, exp_deploy)
        results[world] = (ok, msg)

    evaluator.destroy_node()
    rclpy.shutdown()

    print("\n==================================================")
    print("           SYSTEM VERIFICATION SUMMARY            ")
    print("==================================================")
    all_passed = True
    for world, (ok, msg) in results.items():
        all_passed = all_passed and ok
        status = "PASSED" if ok else "FAILED"
        print(f"  Testbed {world.upper():<6} : {status} ({msg})")
    print("==================================================")

    sys.exit(0 if all_passed else 1)

if __name__ == '__main__':
    main()
