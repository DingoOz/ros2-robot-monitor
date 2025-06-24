#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import math
import time

class TestIMUPublisher(Node):
    def __init__(self):
        super().__init__('test_imu_publisher')
        self.publisher = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(0.05, self.publish_imu)  # 20Hz
        self.start_time = time.time()
        
    def publish_imu(self):
        current_time = time.time() - self.start_time
        
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Generate varying sinusoidal data for interesting charts
        msg.linear_acceleration = Vector3()
        msg.linear_acceleration.x = 2.0 * math.sin(current_time * 0.5) + 1.0
        msg.linear_acceleration.y = 1.5 * math.cos(current_time * 0.8) + 2.0  
        msg.linear_acceleration.z = 9.8 + 0.5 * math.sin(current_time * 1.2)
        
        msg.angular_velocity = Vector3()
        msg.angular_velocity.x = 0.8 * math.sin(current_time * 0.3)
        msg.angular_velocity.y = 1.2 * math.cos(current_time * 0.7)
        msg.angular_velocity.z = 0.6 * math.sin(current_time * 1.5)
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestIMUPublisher()
    
    print("Publishing dynamic IMU data to /imu topic...")
    print("Press Ctrl+C to stop")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()