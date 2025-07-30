#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState
import time
import numpy as np

class VelocityErrorTester(Node):
    def __init__(self):
        super().__init__("velocity_error_tester")

        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        if msg.velocity:
            for velocity in msg.velocity:
                if abs(velocity) > 0:
                    print(f"{velocity:.12f}")

def main(args=None):
    rclpy.init(args=args)
    node = VelocityErrorTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()