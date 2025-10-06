#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import numpy as np
import math

MAX_UINT32 = 2**32 - 1
MAX_DURATION_SEC = MAX_UINT32 / 1e9

class RobotLatencyTester(Node):
    def __init__(self):
        super().__init__("robot_latency_tester")

        self.traj_pub = self.create_publisher(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
        )

        # Joint states subscription for movement latency
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.movement_latencies = []
        self.movement_ros2_delays = []
        self.waiting_for_movement = False

        self.step_count = 0
        self.current_base = 0.0
        self.direction = 1
        self.start_time = 0
        self.test_active = False
        self.duration = 0.1

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.init_robot()

    def init_robot(self):
        # Move to home position with longer duration
        home_q_deg = [0.0, -81.0, 112.0, -122.0, -90.0, 0.0]
        home_q_rad = np.deg2rad(home_q_deg).tolist()
        self.publish_trajectory(home_q_rad, 3.0)
        time.sleep(4)  # Wait longer to ensure settling

        self.duration = 0.1
        self.test_active = True
        timer_interval = 2.0  # Space out commands to allow full stop
        self.timer = self.create_timer(timer_interval, self.measurement_cycle)

    def measurement_cycle(self):
        if self.step_count >= 100:
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

        q_deg = [self.current_base, -81.0, 112.0, -122.0, -90.0, 0.0]
        q_rad = np.deg2rad(q_deg).tolist()

        self.start_time = time.perf_counter()
        self.publish_trajectory(q_rad, self.duration)

        self.waiting_for_movement = True
        self.step_count += 1

    def publish_trajectory(self, positions, duration_sec):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions

        if duration_sec <= MAX_DURATION_SEC:
            point.time_from_start.nanosec = int(duration_sec * 1e9)
        else:
            point.time_from_start.sec = int(duration_sec)

        traj_msg.points = [point]
        self.traj_pub.publish(traj_msg)

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

    def print_results(self):
        print(f"\n{'='*70}")
        print(f"LATENCY TEST RESULTS - {self.step_count} MEASUREMENTS")
        print(f"{'='*70}")

        # Movement latency results (joint states method)
        if self.movement_latencies:
            try:
                with open('movement_latency.txt', 'w') as f:
                    for latency in self.movement_latencies:
                        f.write(f"{latency:.4f}\n")
                print(f"\nMovement latency data written to 'movement_latency.txt'")
            except Exception as e:
                print(f"\nError writing to file: {e}")

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

        print(f"{'='*70}")

def main():
    print("Starting Latency Test")
    print("- Joint States Method: Measures ping-to-movement latency")

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