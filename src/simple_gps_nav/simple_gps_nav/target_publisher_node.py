#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TargetPublisherNode(Node):
    def __init__(self, target_lat, target_lon):
        super().__init__('target_publisher_node')
        
        self.target_lat = target_lat
        self.target_lon = target_lon
        
        self.target_pub = self.create_publisher(
            PoseStamped,
            '/gps_target',
            10
        )
        
        self.timer = self.create_timer(1.0, self.publish_target)
        
        self.get_logger().info(f'Target Publisher Node started')
        self.get_logger().info(f'Publishing target: Lat={self.target_lat:.6f}, Lon={self.target_lon:.6f}')
    
    def publish_target(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.target_lat
        msg.pose.position.y = self.target_lon
        msg.pose.position.z = 0.0
        
        self.target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*50)
    print("GPS Target Publisher Node")
    print("="*50)
    
    try:
        target_lat = float(input("Enter target latitude: "))
        target_lon = float(input("Enter target longitude: "))
    except ValueError:
        print("Invalid input. Using default values (0.0, 0.0)")
        target_lat = 0.0
        target_lon = 0.0
    except KeyboardInterrupt:
        print("\nExiting...")
        return
    
    node = TargetPublisherNode(target_lat, target_lon)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
