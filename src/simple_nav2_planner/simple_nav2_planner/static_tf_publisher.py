#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class MinimalTFPublisher(Node):
    """
    Minimal TF publisher that only adds missing static transforms.
    Does NOT publish odom->base_link (already done by simulator/EKF).
    Only publishes base_link->camera_link and camera_link->camera_depth if needed.
    """
    def __init__(self):
        super().__init__('minimal_tf_publisher')
        
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish missing static transforms once
        self.publish_static_transforms()
        
        self.get_logger().info('Minimal TF Publisher initialized')
        self.get_logger().info('Publishing only missing static transforms (base_link->camera_link, camera_link->camera_depth)')
    
    def publish_static_transforms(self):
        """Publish only missing static transforms."""
        transforms = []
        
        # base_link -> camera_link (static offset, adjust based on your robot)
        t_base_camera = TransformStamped()
        t_base_camera.header.stamp = self.get_clock().now().to_msg()
        t_base_camera.header.frame_id = 'base_link'
        t_base_camera.child_frame_id = 'camera_link'
        # Adjust these values based on where your camera is mounted
        t_base_camera.transform.translation.x = 0.1  # Forward
        t_base_camera.transform.translation.y = 0.0  # Center
        t_base_camera.transform.translation.z = 0.2  # Up
        t_base_camera.transform.rotation.x = 0.0
        t_base_camera.transform.rotation.y = 0.0
        t_base_camera.transform.rotation.z = 0.0
        t_base_camera.transform.rotation.w = 1.0
        transforms.append(t_base_camera)
        
        # camera_link -> camera_depth (identity or small offset)
        t_camera_depth = TransformStamped()
        t_camera_depth.header.stamp = self.get_clock().now().to_msg()
        t_camera_depth.header.frame_id = 'camera_link'
        t_camera_depth.child_frame_id = 'camera_depth'
        t_camera_depth.transform.translation.x = 0.0
        t_camera_depth.transform.translation.y = 0.0
        t_camera_depth.transform.translation.z = 0.0
        t_camera_depth.transform.rotation.x = 0.0
        t_camera_depth.transform.rotation.y = 0.0
        t_camera_depth.transform.rotation.z = 0.0
        t_camera_depth.transform.rotation.w = 1.0
        transforms.append(t_camera_depth)
        
        # Publish all static transforms at once
        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info('Published static transforms')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
