#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState
import time
import numpy as np


class RobotLatencyTester(Node):
    def __init__(self):
        super().__init__("robot_latency_tester")

        self.duration_pub = self.create_publisher(Float32, "/ur10e/movement_duration", 10)
        self.pose_pub = self.create_publisher(Twist, "/ur10e/point_pose", 10)
        self.ik_complete_sub = self.create_subscription(Bool, "/ur10e/ik_complete", self.ik_complete_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        self.traj_latencies = []
        self.movement_latencies = []
        self.step_count = 0
        self.current_x = -1000.0
        self.waiting_for_traj = False
        self.waiting_for_movement = False
        self.start_time = 0
        self.traj_time = 0
        self.test_active = False

        self.init_robot()

    def init_robot(self):
        duration_msg = Float32()
        duration_msg.data = 3.0
        self.duration_pub.publish(duration_msg)

        pose_msg = Twist()
        pose_msg.linear.x = -1000.0
        pose_msg.linear.y = -360.0
        pose_msg.linear.z = 400.0
        pose_msg.angular.x = 180.0
        pose_msg.angular.y = 0.0
        pose_msg.angular.z = 90.0
        self.pose_pub.publish(pose_msg)

        time.sleep(3)

        duration_msg.data = 0.05
        self.duration_pub.publish(duration_msg)

        self.test_active = True
        self.timer = self.create_timer(1.5, self.measurement_cycle)

    def measurement_cycle(self):
        if self.step_count >= 100:
            self.print_results()
            self.timer.cancel()
            self.test_active = False
            self.destroy_node()
            rclpy.shutdown()
            return

        self.current_x += 1.0

        pose_msg = Twist()
        pose_msg.linear.x = self.current_x
        pose_msg.linear.y = -360.0
        pose_msg.linear.z = 400.0
        pose_msg.angular.x = 180.0
        pose_msg.angular.y = 0.0
        pose_msg.angular.z = 90.0
        self.start_time = time.perf_counter()
        self.pose_pub.publish(pose_msg)

        self.waiting_for_traj = True
        self.waiting_for_movement = True
        self.step_count += 1

    def ik_complete_callback(self, msg):
        if self.waiting_for_traj and self.test_active:
            traj_time = time.perf_counter()
            traj_latency = (traj_time - self.start_time) * 1000
            self.traj_latencies.append(traj_latency)
            self.waiting_for_traj = False

    def joint_state_callback(self, msg):
        if self.waiting_for_movement and self.test_active and msg.velocity:
            if any(abs(velocity) > 0.00005 for velocity in msg.velocity):
                movement_time = time.perf_counter()
                movement_latency = (movement_time - self.start_time) * 1000
                self.movement_latencies.append(movement_latency)
                self.waiting_for_movement = False

    def print_results(self):
        print(f"\n{'='*60}")
        print(f"LATENCY TEST RESULTS - {self.step_count} MEASUREMENTS")
        print(f"{'='*60}")

        if self.traj_latencies:
            traj_array = np.array(self.traj_latencies)
            print(f"TRAJECTORY LATENCY (IK completion):")
            print(f"  Average: {np.mean(traj_array):.4f} ms")
            print(f"  Min:     {np.min(traj_array):.4f} ms")
            print(f"  Max:     {np.max(traj_array):.4f} ms")
            print(f"  Std Dev: {np.std(traj_array):.4f} ms")
            print(f"  Count:   {len(self.traj_latencies)}/{self.step_count}")

        if self.movement_latencies:
            move_array = np.array(self.movement_latencies)
            print(f"\nMOVEMENT LATENCY (robot starts moving):")
            print(f"  Average: {np.mean(move_array):.4f} ms")
            print(f"  Min:     {np.min(move_array):.4f} ms")
            print(f"  Max:     {np.max(move_array):.4f} ms")
            print(f"  Std Dev: {np.std(move_array):.4f} ms")
            print(f"  Count:   {len(self.movement_latencies)}/{self.step_count}")

        print(f"{'='*60}")


def main():
    rclpy.init()
    node = RobotLatencyTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()