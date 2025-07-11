#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class ArduinoLatencyTester(Node):
    def __init__(self):
        super().__init__('arduino_latency_tester')
        self.publisher = self.create_publisher(Float32, 'stepper/speed', 10)
        self.subscription = self.create_subscription(Float32, 'stepper/pong', self.pong_callback, 10)
        self.ping_time = 0
        self.latencies = []
        self.test_count = 0
        
    def pong_callback(self, msg):
        pong_time = time.time_ns()
        latency = (pong_time - self.ping_time) / 2
        self.latencies.append(latency)

    def ping(self):
        self.ping_time = time.time_ns()
        msg = Float32()
        msg.data = float(self.test_count)
        self.publisher.publish(msg)
        self.test_count += 1

def main():
    rclpy.init()
    tester = ArduinoLatencyTester()
    
    for i in range(60):
        tester.ping()
        
        timeout = time.time() + 0.95
        while len(tester.latencies) <= i and time.time() < timeout:
            rclpy.spin_once(tester, timeout_sec=0.001)
        
        if len(tester.latencies) <= i:
            print(f"Test {i+1} timed out")
            break
            
        time.sleep(1.0)
    
    rclpy.shutdown()
    
    if tester.latencies:
        avg_latency_ms = sum(tester.latencies) / len(tester.latencies) / 1e6
        print(f"Average one-way latency: {avg_latency_ms:.3f} ms")
        print(f"Tests completed: {len(tester.latencies)}/60")

if __name__ == '__main__':
    main()