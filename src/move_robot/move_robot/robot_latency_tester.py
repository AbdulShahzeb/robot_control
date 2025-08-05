#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import time
import numpy as np
import math


class RobotLatencyTester(Node):
    def __init__(self):
        super().__init__("robot_latency_tester")

        self.duration_pub = self.create_publisher(
            Float32, "/ur10e/movement_duration", 10
        )
        self.joint_pose_pub = self.create_publisher(
            Float32MultiArray, "/ur10e/target_joint_pose", 10
        )

        # Joint states subscription for movement latency
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Controller state subscription for ROS2 communication latency
        self.controller_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            "/scaled_joint_trajectory_controller/state",
            self.controller_state_callback,
            10,
        )

        self.movement_latencies = []
        self.movement_ros2_delays = []
        self.waiting_for_movement = False

        self.communication_latencies = []
        self.communication_ros2_delays = []
        self.controller_message_timestamps = []
        self.waiting_for_communication = False

        self.step_count = 0
        self.current_base = 0.0
        self.direction = 1
        self.start_time = 0
        self.test_active = False

        self.init_robot()

    def init_robot(self):
        duration_msg = Float32()
        duration_msg.data = 3.0
        timer_interval = 2.0

        self.duration_pub.publish(duration_msg)

        joint_msg = Float32MultiArray()
        joint_msg.data = [0.0, -81.0, 112.0, -122.0, -90.0, 0.0]
        self.joint_pose_pub.publish(joint_msg)

        time.sleep(3)

        duration_msg.data = 0.1
        self.duration_pub.publish(duration_msg)

        self.test_active = True
        self.timer = self.create_timer(timer_interval, self.measurement_cycle)

    def measurement_cycle(self):
        if self.step_count >= 500:
            self.print_results()
            self.timer.cancel()
            self.test_active = False
            self.destroy_node()
            rclpy.shutdown()
            return

        if self.current_base >= 50.0:
            self.direction = -1
        elif self.current_base <= 0.0:
            self.direction = 1

        self.current_base += self.direction * 1.0

        joint_msg = Float32MultiArray()
        joint_msg.data = [self.current_base, -81.0, 112.0, -122.0, -90.0, 0.0]
        self.start_time = time.perf_counter()
        self.joint_pose_pub.publish(joint_msg)

        self.waiting_for_movement = True
        self.waiting_for_communication = True
        self.step_count += 1

    def joint_state_callback(self, msg):
        if self.waiting_for_movement and self.test_active and msg.velocity:
            velocity = abs(math.degrees(msg.velocity[5]))
            if velocity >= 0.00005:
                movement_time = time.perf_counter()
                movement_latency = (movement_time - self.start_time) * 1000
                self.movement_latencies.append(movement_latency)

                current_ros_time = self.get_clock().now()
                msg_ros_time = rclpy.time.Time.from_msg(msg.header.stamp)
                ros2_delay = (current_ros_time - msg_ros_time).nanoseconds / 1_000_000.0
                self.movement_ros2_delays.append(ros2_delay)

                self.waiting_for_movement = False

    def controller_state_callback(self, msg):
        current_time = time.perf_counter()
        self.controller_message_timestamps.append(current_time)

        if self.waiting_for_communication and self.test_active and msg.error.positions:
            current_base_error_rad = msg.error.positions[0]
            current_base_error_deg = abs(math.degrees(current_base_error_rad))

            if current_base_error_deg >= 0.002:
                communication_time = time.perf_counter()
                communication_latency = (communication_time - self.start_time) * 1000
                self.communication_latencies.append(communication_latency)

                current_ros_time = self.get_clock().now()
                msg_ros_time = rclpy.time.Time.from_msg(msg.header.stamp)
                ros2_delay = (current_ros_time - msg_ros_time).nanoseconds / 1_000_000.0
                self.communication_ros2_delays.append(ros2_delay)

                self.waiting_for_communication = False

    def calculate_controller_frequency(self):
        if len(self.controller_message_timestamps) < 2:
            return 87.5  # Default fallback

        intervals = []
        for i in range(1, len(self.controller_message_timestamps)):
            interval = (
                self.controller_message_timestamps[i]
                - self.controller_message_timestamps[i - 1]
            )
            intervals.append(interval)

        avg_interval = np.mean(intervals)
        frequency = 1.0 / avg_interval
        return frequency

    def print_results(self):
        print(f"\n{'='*70}")
        print(f"DUAL LATENCY TEST RESULTS - {self.step_count} MEASUREMENTS")
        print(f"{'='*70}")

        # Movement latency results (joint states method)
        if self.movement_latencies:
            move_array = np.array(self.movement_latencies)
            print(f"ACTUAL MOVEMENT LATENCY (Joint States Method):")
            print(f"  Average: {np.mean(move_array):.4f} ms")
            print(f"  Min:     {np.min(move_array):.4f} ms")
            print(f"  Max:     {np.max(move_array):.4f} ms")
            print(f"  Std Dev: {np.std(move_array):.4f} ms")
            print(f"  Count:   {len(self.movement_latencies)}/{self.step_count}")

            if self.movement_ros2_delays:
                comm_array = np.array(self.movement_ros2_delays)
                print(f"\n  ROS2 Subscriber Delay (Joint States):")
                print(f"    Average: {np.mean(comm_array):.4f} ms")
                print(f"    Min:     {np.min(comm_array):.4f} ms")
                print(f"    Max:     {np.max(comm_array):.4f} ms")
                print(f"    Std Dev: {np.std(comm_array):.4f} ms")

                # Compensated movement latency
                avg_movement = np.mean(move_array)
                avg_joint_delay = np.mean(comm_array)
                joint_publisher_latency = 2.0 / 2.0  # 500Hz, half period
                compensated_movement = (
                    avg_movement - avg_joint_delay - joint_publisher_latency
                )

                print(f"\n  Joint States Frequency: 500 Hz (fixed)")
                print(
                    f"  Publisher Latency Compensation: {joint_publisher_latency:.1f} ms"
                )
                print(f"  Compensated Movement Latency:")
                print(
                    f"    {avg_movement:.4f} - {avg_joint_delay:.4f} - {joint_publisher_latency:.1f} = {compensated_movement:.4f} ms"
                )

        print(f"\n{'-'*70}")

        # ROS2 communication latency results (controller error method)
        if self.communication_latencies:
            try:
                with open('ros2_latency.txt', 'w') as f:
                    for latency in self.communication_latencies:
                        f.write(f"{latency:.4f}\n")
                print(f"\nCommunication latency data written to 'ros2_latency.txt'")
            except Exception as e:
                print(f"\nError writing to file: {e}")

            comm_array = np.array(self.communication_latencies)
            print(f"ROS2 COMMUNICATION LATENCY (Controller Error Method):")
            print(f"  Average: {np.mean(comm_array):.4f} ms")
            print(f"  Min:     {np.min(comm_array):.4f} ms")
            print(f"  Max:     {np.max(comm_array):.4f} ms")
            print(f"  Std Dev: {np.std(comm_array):.4f} ms")
            print(f"  Count:   {len(self.communication_latencies)}/{self.step_count}")

            if self.communication_ros2_delays:
                ros2_array = np.array(self.communication_ros2_delays)
                print(f"\n  ROS2 Subscriber Delay (Controller States):")
                print(f"    Average: {np.mean(ros2_array):.4f} ms")
                print(f"    Min:     {np.min(ros2_array):.4f} ms")
                print(f"    Max:     {np.max(ros2_array):.4f} ms")
                print(f"    Std Dev: {np.std(ros2_array):.4f} ms")

                # Compensated communication latency
                avg_communication = np.mean(comm_array)
                avg_controller_delay = np.mean(ros2_array)
                controller_frequency = self.calculate_controller_frequency()
                controller_publisher_latency = (1000.0 / controller_frequency) / 2.0
                compensated_communication = (
                    avg_communication
                    - avg_controller_delay
                    - controller_publisher_latency
                )

                print(
                    f"\n  Controller State Frequency: {controller_frequency:.1f} Hz (measured)"
                )
                print(
                    f"  Publisher Latency Compensation: {controller_publisher_latency:.1f} ms"
                )
                print(f"  Compensated Communication Latency:")
                print(
                    f"    {avg_communication:.4f} - {avg_controller_delay:.4f} - {controller_publisher_latency:.1f} = {compensated_communication:.4f} ms"
                )

        print(f"{'='*70}")

def main():
    print("Starting Dual Latency Test")
    print("- Joint States Method: Measures ping-to-movement latency")
    print("- Controller Error Method: Measures ROS2 communication latency")

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
