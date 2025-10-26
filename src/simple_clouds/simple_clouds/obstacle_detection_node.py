#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs
import struct
import math
from typing import List, Tuple


class ObstacleDetectionNode(Node):
    """
    Python implementation of obstacle detection node for point cloud processing.
    Converts C++ PCL-based processing to pure Python with numpy.
    """
    
    def __init__(self):
        super().__init__('obstacle_detection_node')
        
        # Declare parameters
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('robot_frame', 'camera_link')
        self.declare_parameter('normal_estimation_radius', 0.2)
        self.declare_parameter('max_ground_angle', 0.5)
        self.declare_parameter('outlier_removal_radius', 0.2)
        self.declare_parameter('outlier_removal_min_neighbors', 10)
        self.declare_parameter('min_obstacle_height', -0.5)
        self.declare_parameter('max_obstacle_height', 2.0)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('min_range', 0.3)
        
        # Get parameters
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.normal_radius = self.get_parameter('normal_estimation_radius').get_parameter_value().double_value
        self.max_ground_angle = self.get_parameter('max_ground_angle').get_parameter_value().double_value
        self.outlier_radius = self.get_parameter('outlier_removal_radius').get_parameter_value().double_value
        self.outlier_min_neighbors = self.get_parameter('outlier_removal_min_neighbors').get_parameter_value().integer_value
        self.min_height = self.get_parameter('min_obstacle_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_obstacle_height').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        
        # Initialize TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            'input',
            self.point_cloud_callback,
            self.queue_size
        )
        
        self.publisher = self.create_publisher(
            PointCloud2,
            'output',
            self.queue_size
        )
        
        self.get_logger().info('Obstacle detection node initialized (Python version)')
    
    def point_cloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud and publish filtered obstacles."""
        try:
            # Get the latest available transform instead of exact timestamp
            # This prevents "extrapolation into the future" errors
            # try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                msg.header.frame_id,
                rclpy.time.Time(),  # Use latest available transform
                rclpy.duration.Duration(seconds=1.0)
            )
            # except TransformException:
            #     # Fallback: try with message timestamp but longer timeout
            #     transform = self.tf_buffer.lookup_transform(
            #         self.world_frame,
            #         msg.header.frame_id,
            #         msg.header.stamp,
            #         rclpy.duration.Duration(seconds=1.0)
            #     )
            
            # Convert ROS PointCloud2 to numpy array
            points = self.pointcloud2_to_numpy(msg)
            if len(points) == 0:
                return
            
            # Transform points to world frame
            transformed_points = self.transform_points(points, transform)
            
            # Apply processing pipeline
            range_filtered = self.filter_by_range(transformed_points)
            if len(range_filtered) == 0:
                return
                
            voxel_filtered = self.apply_voxel_filter(range_filtered)
            if len(voxel_filtered) == 0:
                return
                
            outlier_filtered = self.remove_outliers(voxel_filtered)
            if len(outlier_filtered) == 0:
                return
                
            ground_removed = self.remove_ground_plane(outlier_filtered)
            if len(ground_removed) == 0:
                return
                
            height_filtered = self.filter_by_height(ground_removed)
            
            # Convert back to ROS PointCloud2 and publish
            output_msg = self.numpy_to_pointcloud2(height_filtered, msg.header.stamp, self.world_frame)
            self.publisher.publish(output_msg)
            
        except TransformException as ex:
            self.get_logger().warn(f'Transform failed: {ex}')
        except Exception as ex:
            self.get_logger().error(f'Processing failed: {ex}')
    
    def pointcloud2_to_numpy(self, msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 message to numpy array of [x, y, z] points."""
        points = []
        
        # Parse PointCloud2 format
        point_step = msg.point_step
        
        # Find x, y, z field offsets
        x_offset = y_offset = z_offset = None
        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if None in [x_offset, y_offset, z_offset]:
            self.get_logger().warn('Point cloud missing x, y, or z fields')
            return np.array([])
        
        # Extract points
        for i in range(0, len(msg.data), point_step):
            try:
                x = struct.unpack('f', msg.data[i + x_offset:i + x_offset + 4])[0]
                y = struct.unpack('f', msg.data[i + y_offset:i + y_offset + 4])[0]
                z = struct.unpack('f', msg.data[i + z_offset:i + z_offset + 4])[0]
                
                # Filter out invalid points
                if not (math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isinf(x) or math.isinf(y) or math.isinf(z)):
                    points.append([x, y, z])
            except struct.error:
                continue
        
        return np.array(points) if points else np.array([])
    
    def numpy_to_pointcloud2(self, points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
        """Convert numpy array to PointCloud2 message."""
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        
        if len(points) == 0:
            msg.height = 1
            msg.width = 0
            msg.point_step = 12  # 3 floats * 4 bytes
            msg.row_step = 0
            msg.data = []
            msg.is_dense = True
            return msg
        
        msg.height = 1
        msg.width = len(points)
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        
        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack data
        data = []
        for point in points:
            data.extend(struct.pack('fff', point[0], point[1], point[2]))
        
        msg.data = data
        msg.is_dense = True
        
        return msg
    
    def transform_points(self, points: np.ndarray, transform: TransformStamped) -> np.ndarray:
        """Transform points using the given transform."""
        if len(points) == 0:
            return points
        
        # Extract translation and rotation
        trans = transform.transform.translation
        rot = transform.transform.rotation
        
        # Convert quaternion to rotation matrix
        x, y, z, w = rot.x, rot.y, rot.z, rot.w
        rotation_matrix = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
        
        translation = np.array([trans.x, trans.y, trans.z])
        
        # Apply transformation
        transformed = np.dot(points, rotation_matrix.T) + translation
        return transformed
    
    def filter_by_range(self, points: np.ndarray) -> np.ndarray:
        """Filter points by distance from origin."""
        if len(points) == 0:
            return points
        
        ranges = np.linalg.norm(points, axis=1)
        mask = (ranges >= self.min_range) & (ranges <= self.max_range)
        return points[mask]
    
    def apply_voxel_filter(self, points: np.ndarray, voxel_size: float = 0.05) -> np.ndarray:
        """Apply voxel grid filter to downsample point cloud."""
        if len(points) == 0:
            return points
        
        # Simple voxel grid implementation
        voxel_coords = np.floor(points / voxel_size).astype(int)
        
        # Find unique voxels
        unique_voxels, unique_indices = np.unique(voxel_coords, axis=0, return_inverse=True)
        
        # Average points in each voxel
        filtered_points = []
        for i in range(len(unique_voxels)):
            voxel_mask = unique_indices == i
            voxel_points = points[voxel_mask]
            filtered_points.append(np.mean(voxel_points, axis=0))
        
        return np.array(filtered_points)
    
    def remove_outliers(self, points: np.ndarray) -> np.ndarray:
        """Remove outlier points based on neighbor count."""
        if len(points) == 0:
            return points
        
        # Simple radius-based outlier removal
        filtered_points = []
        
        for i, point in enumerate(points):
            # Count neighbors within radius
            distances = np.linalg.norm(points - point, axis=1)
            neighbor_count = np.sum(distances < self.outlier_radius) - 1  # Exclude self
            
            if neighbor_count >= self.outlier_min_neighbors:
                filtered_points.append(point)
        
        return np.array(filtered_points) if filtered_points else np.array([])
    
    def remove_ground_plane(self, points: np.ndarray) -> np.ndarray:
        """Remove ground plane using RANSAC-like approach."""
        if len(points) == 0:
            return points
        
        # Simple ground plane removal - assume lowest z values are ground
        # Sort by z coordinate
        z_sorted_indices = np.argsort(points[:, 2])
        
        # Take bottom 20% of points as potential ground
        ground_sample_size = max(int(len(points) * 0.2), 10)
        if ground_sample_size >= len(points):
            return points
        
        ground_candidates = points[z_sorted_indices[:ground_sample_size]]
        
        # Estimate ground plane height as median of candidates
        ground_height = np.median(ground_candidates[:, 2])
        
        # Remove points close to ground plane
        threshold = 0.1  # 10cm threshold
        mask = points[:, 2] > (ground_height + threshold)
        
        return points[mask]
    
    def filter_by_height(self, points: np.ndarray) -> np.ndarray:
        """Filter points by height relative to robot."""
        if len(points) == 0:
            return points
        
        try:
            # Get robot position in world frame using latest available transform
            robot_transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.robot_frame,
                rclpy.time.Time(),  # Use latest available transform
                rclpy.duration.Duration(seconds=1.0)
            )
            
            robot_z = robot_transform.transform.translation.z
            
            # Filter by relative height
            relative_heights = points[:, 2] - robot_z
            mask = (relative_heights >= self.min_height) & (relative_heights <= self.max_height)
            
            return points[mask]
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not get robot transform for height filtering: {ex}')
            return points  # Return all points if transform fails


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
