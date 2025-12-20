#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np


class PointCloudTransformNode(Node):
    """
    Node that subscribes to Unity point cloud (YZX format) and republishes in standard XYZ format.
    """

    def __init__(self):
        super().__init__('pointcloud_transform_node')
        
        # Parameters
        self.declare_parameter('input_topic', '/camera/points_unity')
        self.declare_parameter('output_topic', '/camera/points')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # QoS profile for input (match Unity's settings - likely BEST_EFFORT)
        input_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # QoS profile for output (RELIABLE for RViz2 compatibility)
        output_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            input_qos
        )
        
        # Publisher
        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            output_qos
        )
        
        self.msg_count = 0
        
        self.get_logger().info(f'PointCloud Transform Node started')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing to: {output_topic}')

    def pointcloud_callback(self, msg):
        """
        Callback to transform point cloud from YZX to XYZ format.
        """
        self.msg_count += 1
        
        # Log first message details
        if self.msg_count == 1:
            self.get_logger().info(f'Received first point cloud:')
            self.get_logger().info(f'  Frame: {msg.header.frame_id}')
            self.get_logger().info(f'  Size: {msg.width}x{msg.height}')
            self.get_logger().info(f'  Point step: {msg.point_step}')
            self.get_logger().info(f'  Fields: {[f.name for f in msg.fields]}')
        
        # Create new PointCloud2 message
        output_msg = PointCloud2()
        
        # Copy header and metadata
        output_msg.header = msg.header
        output_msg.height = msg.height
        output_msg.width = msg.width
        output_msg.is_bigendian = msg.is_bigendian
        output_msg.point_step = msg.point_step
        output_msg.row_step = msg.row_step
        output_msg.is_dense = msg.is_dense
        
        # Create new fields in XYZ order
        output_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1)
        ]
        
        # Fast transformation using NumPy
        # Convert bytes to numpy array
        num_points = msg.width * msg.height
        
        # Create structured array for input (YZX format)
        input_dtype = np.dtype([
            ('y', np.float32),
            ('z', np.float32),
            ('x', np.float32),
            ('rgba', np.uint32)
        ])
        
        # Create structured array for output (XYZ format)
        output_dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgba', np.uint32)
        ])
        
        # Parse input data
        input_points = np.frombuffer(msg.data, dtype=input_dtype)
        
        # Create output array and rearrange fields
        output_points = np.zeros(num_points, dtype=output_dtype)
        output_points['x'] = input_points['x']
        output_points['y'] = input_points['y']
        output_points['z'] = input_points['z']
        output_points['rgba'] = input_points['rgba']
        
        # Convert back to bytes
        output_msg.data = output_points.tobytes()
        
        # Publish transformed point cloud
        self.publisher.publish(output_msg)
        
        if self.msg_count % 30 == 0:
            self.get_logger().info(f'Processed {self.msg_count} point clouds')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
