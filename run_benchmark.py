#!/usr/bin/env python3
import argparse
import sys
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from battery_disassembly_arm.srv import RunPlannerBenchmark
from geometry_msgs.msg import Pose, Quaternion


BOLT_APPROACH_POSES: List[Tuple[float, float, float]] = [
    (0.385, 0.535, 1.09),
    (0.385, 0.065, 1.09),
    (-0.385, 0.535, 1.09),
    (-0.385, 0.065, 1.09),
    (0.0, 0.535, 1.09),
    (0.0, 0.065, 1.09),
]


class BenchmarkClient(Node):
    def __init__(self) -> None:
        super().__init__("run_benchmark_client")
        self.client = self.create_client(RunPlannerBenchmark, "/run_rrt_benchmark")

    def wait_for_service(self) -> bool:
        self.get_logger().info("Waiting for /run_rrt_benchmark service...")
        return self.client.wait_for_service(timeout_sec=10.0)

    def call(self, bolt_id: int, runs: int, csv_path: str, export_csv: bool) -> int:
        req = RunPlannerBenchmark.Request()
        req.runs_per_mode = runs
        req.start_joint_values = [0.0, -1.5707963, 0.0, -1.5707963, 0.0, 0.0]

        x, y, z = BOLT_APPROACH_POSES[bolt_id - 1]
        req.target_pose = Pose()
        req.target_pose.position.x = x
        req.target_pose.position.y = y
        req.target_pose.position.z = z
        req.target_pose.orientation.w = 1.0

        req.target_orientation = Quaternion()
        req.target_orientation.w = 1.0

        req.export_csv = export_csv
        req.csv_path = csv_path

        self.get_logger().info(
            f"Calling benchmark: bolt_id={bolt_id}, target=({x:.3f},{y:.3f},{z:.3f}), runs={runs}"
        )

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=300.0)

        if not future.done() or future.result() is None:
            self.get_logger().error("Benchmark service call timeout or empty response")
            return 2

        res = future.result()
        print("\n===== Benchmark Result =====")
        print(f"success: {res.success}")
        print(f"message: {res.message}")
        print(res.markdown_table)
        print("mode_avg_time_ms:", list(res.mode_avg_time_ms))
        print("mode_avg_nodes:", list(res.mode_avg_nodes))
        print("mode_avg_path_length_rad:", list(res.mode_avg_path_length_rad))
        print("mode_orientation_satisfaction_rate:", list(res.mode_orientation_satisfaction_rate))
        print("mode_collision_success_rate:", list(res.mode_collision_success_rate))

        return 0 if res.success else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run RRT benchmark for selected bolt target")
    parser.add_argument("--bolt_id", type=int, default=1, choices=range(1, 7), help="Bolt index in [1..6]")
    parser.add_argument("--runs", type=int, default=10, help="Runs per algorithm mode")
    parser.add_argument("--csv", type=str, default="rrt_benchmark_results.csv", help="Output CSV path")
    parser.add_argument("--no_csv", action="store_true", help="Disable CSV export")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = BenchmarkClient()
    try:
        if not node.wait_for_service():
            node.get_logger().error("Service /run_rrt_benchmark not available")
            return 2
        return node.call(
            bolt_id=args.bolt_id,
            runs=args.runs,
            csv_path=args.csv,
            export_csv=not args.no_csv,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
