#!/usr/bin/env python3
"""Verify static terrain classification and gear interlocks using real scans."""
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, Float32MultiArray, Int32

SIMULATION_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(SIMULATION_DIR, 'launch', 'system_launch.py')
SENSOR_TOPICS = ('/sensing/tof/fore', '/sensing/tof/port',
                 '/sensing/tof/starboard', '/sensing/sonar/scan')


class VerificationEvaluator(Node):
    def __init__(self):
        super().__init__('verification_evaluator')
        self.reset()
        self.create_subscription(Int32, '/intelligence/verdict', self.verdict_cb, 10)
        self.create_subscription(Bool, '/intelligence/abort', self.abort_cb, 10)
        self.create_subscription(Float64, '/control/gear_status', self.gear_status_cb, 10)
        self.create_subscription(Float32MultiArray, '/sensing/features', self.features_cb, 10)
        for topic in SENSOR_TOPICS:
            self.create_subscription(LaserScan, topic,
                                     lambda msg, name=topic: self.scan_cb(name, msg),
                                     rclpy.qos.qos_profile_sensor_data)
        self.deploy_pub = self.create_publisher(Bool, '/control/gear_deploy', 10)

    def reset(self):
        self.verdict = None
        self.abort = None
        self.gear_status = None
        self.features = None
        self.feature_time = 0.0
        self.message_times = {}
        self.scan_counts = {topic: 0 for topic in SENSOR_TOPICS}
        self.scan_times = {topic: 0.0 for topic in SENSOR_TOPICS}

    def verdict_cb(self, msg):
        self.verdict = msg.data
        self.message_times['verdict'] = time.monotonic()

    def abort_cb(self, msg):
        self.abort = msg.data
        self.message_times['abort'] = time.monotonic()

    def gear_status_cb(self, msg):
        self.gear_status = msg.data
        self.message_times['gear'] = time.monotonic()

    def features_cb(self, msg):
        if len(msg.data) >= 7 and all(math.isfinite(v) for v in msg.data):
            self.features = list(msg.data)
            self.feature_time = time.monotonic()

    def scan_cb(self, topic, msg):
        if any(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges):
            self.scan_counts[topic] += 1
            self.scan_times[topic] = time.monotonic()

    def ready(self):
        now = time.monotonic()
        return (all(self.scan_counts[t] >= 10 and now - self.scan_times[t] < 2 for t in SENSOR_TOPICS)
                and self.features is not None and now - self.feature_time < 2
                and all(k in self.message_times and now - self.message_times[k] < 2
                        for k in ('verdict', 'abort', 'gear')))

    def trigger_deployment(self):
        self.deploy_pub.publish(Bool(data=True))


def stop_launch(proc):
    """Stop only this launch's session, including its child simulator processes."""
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass
    # Launch may exit while a child is still stopping. Remove only its group.
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    proc.wait()


def test_scenario(evaluator, world_name, expected_verdict, expected_abort, expect_deploy):
    print(f'\n=== VERIFY {world_name.upper()} ===', flush=True)
    evaluator.reset()
    log_dir = Path(SIMULATION_DIR) / 'log/verification'
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f'{world_name}.log'
    cmd = ['ros2', 'launch', LAUNCH_FILE,
           f'world:={world_name}', 'gui:=false', 'flight:=false']
    with log_path.open('w') as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            print('Waiting for ten valid scans per sensor and fresh telemetry (up to 60 s)...', flush=True)
            deadline = time.monotonic() + 60
            stable_since = None
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    print(f'Launch exited early. Inspect {log_path}', flush=True)
                    return False, 'LAUNCH FAILED'
                rclpy.spin_once(evaluator, timeout_sec=0.05)
                if evaluator.ready():
                    if stable_since is None:
                        stable_since = time.monotonic()
                    if time.monotonic() - stable_since >= 1:
                        break
                else:
                    stable_since = None
            else:
                print('Sensor readiness timed out. Valid scan counts:', evaluator.scan_counts, flush=True)
                print(f'Inspect {log_path}', flush=True)
                return False, 'SENSORS NOT READY'

            print('Features:', [round(v, 6) for v in evaluator.features], flush=True)
            actual_verdict, actual_abort = evaluator.verdict, evaluator.abort
            # Keep a single request owner. Wait for the trajectory, not a fixed sleep.
            deadline = time.monotonic() + 15
            last_request = 0.0
            matched_since = None
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now - last_request >= 0.2:
                    evaluator.trigger_deployment()
                    last_request = now
                rclpy.spin_once(evaluator, timeout_sec=0.05)
                if proc.poll() is not None:
                    print(f'Launch exited during deployment. Inspect {log_path}', flush=True)
                    return False, 'LAUNCH FAILED'
                gear = evaluator.gear_status
                gear_ok = gear is not None and (gear > 1.2 if expect_deploy else gear < 0.05)
                if (evaluator.ready() and gear_ok and evaluator.verdict == expected_verdict
                        and evaluator.abort == expected_abort):
                    if matched_since is None:
                        matched_since = now
                    if now - matched_since >= 1:
                        break
                else:
                    matched_since = None

            actual_verdict, actual_abort = evaluator.verdict, evaluator.abort
            gear = evaluator.gear_status
            passed = (matched_since is not None and time.monotonic() - matched_since >= 1
                      and evaluator.ready() and actual_verdict == expected_verdict
                      and actual_abort == expected_abort
                      and gear is not None and (gear > 1.2 if expect_deploy else gear < 0.05))
            print(f'Verdict: expected={expected_verdict}, received={actual_verdict}', flush=True)
            print(f'Abort: expected={expected_abort}, received={actual_abort}', flush=True)
            print(f'Gear: {gear} rad, expect {"deployed" if expect_deploy else "stowed"}', flush=True)
            print(f'{"PASS" if passed else "FAIL"}. Launch log: {log_path}', flush=True)
            return passed, 'SUCCESS' if passed else 'FAILED'
        finally:
            stop_launch(proc)


def main():
    rclpy.init()
    evaluator = VerificationEvaluator()
    results = {}
    try:
        for world, verdict, abort, deploy in [('alpha', 0, False, True),
                                               ('beta', 1, True, False),
                                               ('gamma', 2, True, False)]:
            results[world] = test_scenario(evaluator, world, verdict, abort, deploy)
    finally:
        evaluator.destroy_node()
        rclpy.shutdown()
    print('\n=== SYSTEM VERIFICATION SUMMARY ===')
    for world, (passed, status) in results.items():
        print(f'{world.upper()}: {"PASSED" if passed else "FAILED"} ({status})')
    return 0 if all(passed for passed, _ in results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
