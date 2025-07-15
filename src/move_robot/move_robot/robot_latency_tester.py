#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState
import time


class RobotLatencyTester(Node):
    def __init__(self):
        super().__init__("robot_latency_tester")

        self.duration_pub = self.create_publisher(
            Float32, "/ur10e/movement_duration", 10
        )
        self.pose_pub = self.create_publisher(Twist, "/ur10e/point_pose", 10)
        self.is_moving_sub = self.create_subscription(
            Bool, "/ur10e/is_moving", self.is_moving_callback, 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.ping_time = 0
        self.robot_clock_latency = 3e6
        self.ur_control_latency = 1e6
        self.traj_latencies = []
        self.movement_latencies = []
        self.cycle_count = 0
        self.current_x = -1000.0
        self.waiting_for_trajectory = False
        self.waiting_for_movement = False
        self.test_active = False
        self.test_round = 1
        self.total_rounds = 3

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

        duration_msg.data = 0.2
        self.duration_pub.publish(duration_msg)

        self.test_active = True
        self.timer = self.create_timer(1.0, self.measurement_cycle)
        print(f"Starting test round {self.test_round}/{self.total_rounds}")

    def measurement_cycle(self):
        if self.cycle_count >= 60:
            self.print_round_results()

            if self.test_round < self.total_rounds:
                self.reset_for_next_round()
            else:
                self.print_final_results()
                self.timer.cancel()
                self.test_active = False
                self.destroy_node()
                rclpy.shutdown()
            return

        self.current_x += 2.0
        self.ping_time = time.time_ns()

        pose_msg = Twist()
        pose_msg.linear.x = self.current_x
        pose_msg.linear.y = -360.0
        pose_msg.linear.z = 400.0
        pose_msg.angular.x = 180.0
        pose_msg.angular.y = 0.0
        pose_msg.angular.z = 90.0
        self.pose_pub.publish(pose_msg)

        self.waiting_for_trajectory = True
        self.waiting_for_movement = True
        self.cycle_count += 1

    def is_moving_callback(self, msg):
        if msg.data and self.waiting_for_trajectory and self.test_active:
            traj_time = time.time_ns()
            traj_latency = traj_time - self.ping_time - self.ur_control_latency

            self.traj_latencies.append(traj_latency)
            self.waiting_for_trajectory = False

    def joint_state_callback(self, msg):
        # Check if robot is moving by looking at joint velocities
        is_moving = False
        if msg.velocity:
            for velocity in msg.velocity:
                if abs(velocity) > 0.0:  # Any non-zero velocity means robot is moving
                    is_moving = True
                    break

        if is_moving and self.waiting_for_movement and self.test_active:
            movement_time = time.time_ns()
            movement_latency = movement_time - self.ping_time - self.robot_clock_latency

            self.movement_latencies.append(movement_latency)
            self.waiting_for_movement = False

    def print_round_results(self):
        print(f"\n=== Round {self.test_round} Results ===")

        if self.traj_latencies:
            avg_traj_latency_ms = (
                sum(self.traj_latencies) / len(self.traj_latencies) / 1e6
            )
            print(f"Average ping-to-trajectory latency: {avg_traj_latency_ms:.3f} ms")
            print(f"Trajectory measurements: {len(self.traj_latencies)}/60")

        if self.movement_latencies:
            avg_movement_latency_ms = (
                sum(self.movement_latencies) / len(self.movement_latencies) / 1e6
            )
            print(f"Average ping-to-movement latency: {avg_movement_latency_ms:.3f} ms")
            print(f"Movement measurements: {len(self.movement_latencies)}/60")

        if self.traj_latencies and self.movement_latencies:
            avg_robot_latency = (
                avg_movement_latency_ms - avg_traj_latency_ms
            )
            print(f"Total robot latency: {avg_robot_latency:.3f} ms")

    def reset_for_next_round(self):
        self.test_round += 1
        self.cycle_count = 0
        self.current_x = -1000.0
        self.traj_latencies = []
        self.movement_latencies = []
        self.waiting_for_trajectory = False
        self.waiting_for_movement = False

        print(f"\nStarting test round {self.test_round}/{self.total_rounds}")

    def print_final_results(self):
        print(f"\n{'='*50}")
        print(f"ALL {self.total_rounds} ROUNDS COMPLETED")
        print(f"{'='*50}")


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
