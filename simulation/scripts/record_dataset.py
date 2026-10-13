#!/usr/bin/env python3
import argparse
import csv
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray

LABEL_MAP = {
    'alpha': 0,  # SAFE
    'beta': 1,   # UNSAFE_SLOPE
    'gamma': 2   # UNSAFE_VEGETATION
}

HEADER = [
    'mean_tof',
    'spatial_variance',
    'delta_d',
    'delta_d_variance',
    'beta_slope',
    'omega_norm',
    'tilt_error',
    'label'
]

class DatasetRecorder(Node):
    def __init__(self, world_name: str, max_samples: int, output_file: str):
        super().__init__('dataset_recorder')
        self.world_name = world_name.lower()
        self.max_samples = max_samples
        self.output_file = output_file
        self.sample_count = 0
        self.label = LABEL_MAP.get(self.world_name, 0)

        # Ensure directory exists
        os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)
        file_exists = os.path.exists(output_file)

        self.csv_file = open(output_file, mode='a', newline='')
        self.writer = csv.writer(self.csv_file)
        if not file_exists:
            self.writer.writerow(HEADER)
            self.csv_file.flush()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            Float32MultiArray,
            '/sensing/features',
            self.features_callback,
            qos
        )

        self.get_logger().info(
            f"Recording started: world='{self.world_name}' (label={self.label}) "
            f"target={self.max_samples} samples -> '{self.output_file}'"
        )

    def features_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 7:
            return

        row = list(msg.data[:7]) + [self.label]
        self.writer.writerow(row)
        self.csv_file.flush()
        self.sample_count += 1

        if self.sample_count % 50 == 0 or self.sample_count >= self.max_samples:
            self.get_logger().info(f"Progress: [{self.sample_count}/{self.max_samples}] samples recorded.")

        if self.sample_count >= self.max_samples:
            self.get_logger().info("Target sample count reached. Exiting recording cleanly.")
            self.csv_file.close()
            rclpy.shutdown()
            sys.exit(0)

def main():
    parser = argparse.ArgumentParser(description="Record labeled 7D feature vectors from /sensing/features to CSV")
    parser.add_argument('--world', type=str, required=True, choices=['alpha', 'beta', 'gamma'],
                        help="World scenario being sampled (alpha, beta, gamma)")
    parser.add_argument('--samples', type=int, default=500,
                        help="Number of samples to record (default: 500)")
    parser.add_argument('--output', type=str, default="data/dataset.csv",
                        help="Target output CSV file path")
    args = parser.parse_args()

    rclpy.init()
    node = DatasetRecorder(args.world, args.samples, args.output)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user. Closing dataset file.")
    finally:
        if not node.csv_file.closed:
            node.csv_file.close()

if __name__ == '__main__':
    main()