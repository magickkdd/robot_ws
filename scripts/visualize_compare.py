#!/usr/bin/env python3
"""Call planning services to visualize raw RRT* and smoothed quintic trajectory in RViz.

This script only triggers planning and marker rendering. It does not command controllers.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from battery_disassembly_arm.srv import RRTStarPlanPose, PlanJointTrajectory


class CompareVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("visualize_compare_client")
        self.rrt_client = self.create_client(RRTStarPlanPose, "/rrt_star_plan_pose")
        self.quintic_client = self.create_client(PlanJointTrajectory, "/plan_joint_trajectory")

    def wait_services(self) -> bool:
        self.get_logger().info("等待服务:/rrt_star_plan_pose,/plan_joint_trajectory")
        while rclpy.ok() and not self.rrt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("等待/rrt_star_plan_pose...")
        while rclpy.ok() and not self.quintic_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("等待/plan_joint_trajectory...")
        return rclpy.ok()

    def run_once(self) -> bool:
        pose = Pose()
        pose.position.x = 0.385
        pose.position.y = 0.535
        pose.position.z = 1.09
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0

        rrt_req = RRTStarPlanPose.Request()
        rrt_req.target_pose = pose

        self.get_logger().info("调用RRT*位姿规划服务,仅用于可视化")
        rrt_future = self.rrt_client.call_async(rrt_req)
        rclpy.spin_until_future_complete(self, rrt_future)
        if rrt_future.result() is None:
            self.get_logger().error("RRT*服务调用失败")
            return False

        rrt_res = rrt_future.result()
        if not rrt_res.success:
            self.get_logger().error(f"RRT*规划失败:{rrt_res.message}")
            return False

        self.get_logger().info(f"RRT*路径点数:{rrt_res.num_points},Marker ns=raw_path")

        q_req = PlanJointTrajectory.Request()
        q_req.path_joint_values = rrt_res.path_joint_values
        q_req.num_points = rrt_res.num_points
        q_req.dof = rrt_res.dof
        q_req.max_velocity = 2.0
        q_req.max_acceleration = 2.0

        self.get_logger().info("调用Quintic轨迹服务,仅用于可视化")
        q_future = self.quintic_client.call_async(q_req)
        rclpy.spin_until_future_complete(self, q_future)
        if q_future.result() is None:
            self.get_logger().error("Quintic服务调用失败")
            return False

        q_res = q_future.result()
        if not q_res.success:
            self.get_logger().error(f"Quintic规划失败:{q_res.message}")
            return False

        self.get_logger().info(
            f"Quintic轨迹点数:{len(q_res.trajectory.points)},Marker ns=smoothed_trajectory"
        )
        self.get_logger().info("已完成静态可视化触发,未向控制器发送任何轨迹")
        return True


def main() -> None:
    rclpy.init()
    node = CompareVisualizer()
    ok = False
    try:
        if node.wait_services():
            ok = node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
