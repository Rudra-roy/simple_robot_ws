#!/usr/bin/env python3
"""
Yaw Calibration Helper
Run this while robot is pointing in a known direction to get the offset.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class YawCalibrator(Node):
    def __init__(self):
        super().__init__('yaw_calibrator')
        self.sub = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            10
        )
        self.readings = []
        self.get_logger().info('Yaw Calibrator Started')
        self.get_logger().info('Point robot FORWARD and press Ctrl+C when ready')
        
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw)
        
        self.readings.append(yaw_deg)
        if len(self.readings) > 50:
            self.readings.pop(0)
        
        avg_yaw = sum(self.readings) / len(self.readings)
        
        print(f'\rCurrent Yaw: {yaw_deg:7.2f}° | Average: {avg_yaw:7.2f}° | Offset to use: {-avg_yaw:7.2f}°', end='', flush=True)

def main():
    print()
    print('=' * 60)
    print('           YAW OFFSET CALIBRATION TOOL')
    print('=' * 60)
    print()
    print('Instructions:')
    print('  1. Position robot pointing in your desired "forward" direction')
    print('  2. Keep robot stationary')
    print('  3. Read the "Offset to use" value')
    print('  4. Press Ctrl+C when done')
    print()
    print('The offset value should be added to your navigation config.')
    print()
    
    rclpy.init()
    node = YawCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        avg = sum(node.readings) / len(node.readings) if node.readings else 0
        print()
        print()
        print('=' * 60)
        print(f'  FINAL OFFSET: {-avg:.2f}°')
        print('=' * 60)
        print()
        print(f'Add this to your navigation stack config:')
        print(f'  yaw_offset: {-avg:.4f}  # degrees')
        print(f'  yaw_offset_rad: {math.radians(-avg):.6f}  # radians')
        print()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
