#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import time

class RobotLatencyTester(Node):
    def __init__(self):
        super().__init__('robot_latency_tester')
        
        self.duration_pub = self.create_publisher(Float32, '/ur10e/movement_duration', 10)
        self.pose_pub = self.create_publisher(Twist, '/ur10e/point_pose', 10)
        self.is_moving_sub = self.create_subscription(Bool, '/ur10e/is_moving', self.is_moving_callback, 10)
        
        self.ping_time = 0
        self.latencies = []
        self.cycle_count = 0
        self.current_x = -1000.0
        self.waiting_for_movement = False
        self.test_active = False
        
        self.robot_latency_ns = 2000000
        
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
        
        duration_msg.data = 1.0
        self.duration_pub.publish(duration_msg)
        
        self.test_active = True
        self.timer = self.create_timer(1.0, self.measurement_cycle)
    
    def measurement_cycle(self):
        if self.cycle_count >= 60:
            if self.latencies:
                avg_latency_ms = sum(self.latencies) / len(self.latencies) / 1e6
                print(f"Average one-way latency: {avg_latency_ms:.3f} ms")
                print(f"Tests completed: {len(self.latencies)}/60")
            self.timer.cancel()
            self.test_active = False
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
        
        self.waiting_for_movement = True
        self.cycle_count += 1
    
    def is_moving_callback(self, msg):
        if msg.data and self.waiting_for_movement and self.test_active:
            pong_time = time.time_ns()
            raw_latency = pong_time - self.ping_time - self.robot_latency_ns
            one_way_latency = raw_latency / 2
            
            self.latencies.append(one_way_latency)
            self.waiting_for_movement = False

def main():
    rclpy.init()
    node = RobotLatencyTester()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()