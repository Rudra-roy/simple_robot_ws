#!/usr/bin/env python3
"""
Simple 2D Mapper Node
Creates a 2D occupancy grid map from point cloud data and publishes map->odom transform.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
import struct


class MapperNode(Node):
    def __init__(self):
        super().__init__('mapper_node')
        
        # Parameters
        self.declare_parameter('cloud_topic', '/rosbot/camera_depth/point_cloud')
        self.declare_parameter('map_resolution', 0.05)  # meters per cell
        self.declare_parameter('map_width', 200)  # cells
        self.declare_parameter('map_height', 200)  # cells
        self.declare_parameter('min_height', -0.5)  # minimum z to consider (meters)
        self.declare_parameter('max_height', 2.0)  # maximum z to consider (meters)
        self.declare_parameter('obstacle_threshold', 0.3)  # height above ground to be obstacle
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('publish_map', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        # Get parameters
        cloud_topic = self.get_parameter('cloud_topic').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        update_rate = self.get_parameter('update_rate').value
        self.publish_map_flag = self.get_parameter('publish_map').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # Initialize map (persistent - grows over time)
        self.map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)  # -1 = unknown
        self.hit_count = np.zeros((self.map_height, self.map_width), dtype=np.int32)  # Track observations
        self.miss_count = np.zeros((self.map_height, self.map_width), dtype=np.int32)  # Track free space
        self.map_origin_x = -(self.map_width * self.map_resolution) / 2.0
        self.map_origin_y = -(self.map_height * self.map_resolution) / 2.0
        
        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Map->odom transform (initially identity)
        # The map builds in odom frame coordinates, and we publish map->odom as identity
        # This means the map origin aligns with the odom origin at startup
        self.map_to_odom = TransformStamped()
        self.map_to_odom.header.frame_id = self.map_frame
        self.map_to_odom.child_frame_id = self.odom_frame
        self.map_to_odom.transform.rotation.w = 1.0
        
        # Subscribers
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            cloud_topic,
            self.cloud_callback,
            10
        )
        
        # Publishers
        if self.publish_map_flag:
            self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Timer for publishing map and transform
        self.timer = self.create_timer(1.0 / update_rate, self.timer_callback)
        
        self.latest_cloud = None
        self.get_logger().info(f'Mapper node started. Subscribing to {cloud_topic}')
        
    def cloud_callback(self, msg):
        """Store the latest point cloud for processing"""
        self.latest_cloud = msg
        
    def timer_callback(self):
        """Process point cloud and publish map + transform"""
        if self.latest_cloud is None:
            return
            
        # Process the point cloud to update the map
        self.process_cloud(self.latest_cloud)
        
        # Publish the map
        if self.publish_map_flag:
            self.publish_map()
        
        # Publish map->odom transform
        self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.map_to_odom)
        
    def process_cloud(self, cloud_msg):
        """Convert point cloud to 2D occupancy grid (accumulative/persistent)"""
        # Parse point cloud
        points = self.parse_pointcloud2(cloud_msg)
        if points is None or len(points) == 0:
            return
            
        # Get robot position in odom frame (since we publish map->odom)
        try:
            robot_transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            robot_x = robot_transform.transform.translation.x
            robot_y = robot_transform.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'Could not get robot position: {e}', throttle_duration_sec=5.0)
            return
            
        # Get transform from cloud frame to odom frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                cloud_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'Could not transform point cloud: {e}', throttle_duration_sec=5.0)
            return
        
        # Transform points to odom frame (map is aligned with odom at origin)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        
        # Simple translation (assuming no significant rotation)
        points[:, 0] += tx
        points[:, 1] += ty
        points[:, 2] += tz
        
        # Filter by height
        height_mask = (points[:, 2] > self.min_height) & (points[:, 2] < self.max_height)
        filtered_points = points[height_mask]
        
        if len(filtered_points) == 0:
            return
        
        # Separate into ground and obstacles
        obstacle_mask = filtered_points[:, 2] > self.obstacle_threshold
        
        # Ray tracing: mark free space between robot and obstacles
        robot_grid_x = int((robot_x - self.map_origin_x) / self.map_resolution)
        robot_grid_y = int((robot_y - self.map_origin_y) / self.map_resolution)
        
        # Update map with observations
        for i, point in enumerate(filtered_points):
            x, y, z = point[:3]
            
            # Convert to grid coordinates
            grid_x = int((x - self.map_origin_x) / self.map_resolution)
            grid_y = int((y - self.map_origin_y) / self.map_resolution)
            
            # Check bounds
            if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                if obstacle_mask[i]:
                    # Mark as obstacle (hit)
                    self.hit_count[grid_y, grid_x] += 1
                else:
                    # Mark as free space
                    self.miss_count[grid_y, grid_x] += 1
                
                # Simple ray tracing: mark cells between robot and point as free
                self._mark_free_cells(robot_grid_x, robot_grid_y, grid_x, grid_y)
        
        # Update occupancy based on hit/miss counts (simple Bayesian update)
        self._update_occupancy()
    
    def _mark_free_cells(self, x0, y0, x1, y1):
        """Mark cells along a line as free space (Bresenham's line algorithm)"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            # Don't mark the endpoint (that's the obstacle)
            if x == x1 and y == y1:
                break
                
            # Mark as free if in bounds
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                self.miss_count[y, x] += 1
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def _update_occupancy(self):
        """Update occupancy probabilities based on hit/miss counts"""
        total_count = self.hit_count + self.miss_count
        
        # Calculate occupancy probability
        # If total observations > 0, calculate probability
        mask = total_count > 0
        
        # Occupancy = hits / (hits + misses) * 100
        occupancy = np.zeros_like(self.map_data, dtype=np.float32)
        occupancy[mask] = (self.hit_count[mask] / total_count[mask]) * 100
        
        # Convert to occupancy grid values
        # Free: 0-40%, Occupied: 60-100%, Unknown: 40-60% or no data
        self.map_data[occupancy < 40] = 0    # Free
        self.map_data[occupancy > 60] = 100  # Occupied
        self.map_data[(occupancy >= 40) & (occupancy <= 60)] = 50  # Uncertain
        self.map_data[~mask] = -1  # Unknown (no observations)
        
    def parse_pointcloud2(self, cloud_msg):
        """Parse PointCloud2 message to numpy array"""
        # Get point step and row step
        point_step = cloud_msg.point_step
        row_step = cloud_msg.row_step
        
        # Find x, y, z fields
        x_offset = None
        y_offset = None
        z_offset = None
        
        for field in cloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if x_offset is None or y_offset is None or z_offset is None:
            self.get_logger().warn('Point cloud missing x, y, or z fields')
            return None
        
        # Parse points
        points = []
        data = cloud_msg.data
        
        for i in range(0, len(data), point_step):
            if i + point_step > len(data):
                break
                
            x = struct.unpack('f', data[i + x_offset:i + x_offset + 4])[0]
            y = struct.unpack('f', data[i + y_offset:i + y_offset + 4])[0]
            z = struct.unpack('f', data[i + z_offset:i + z_offset + 4])[0]
            
            # Filter out invalid points
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or 
                    np.isinf(x) or np.isinf(y) or np.isinf(z)):
                points.append([x, y, z])
        
        return np.array(points) if points else None
        
    def publish_map(self):
        """Publish the occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = self.map_frame
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Flatten map data (row-major order)
        map_msg.data = self.map_data.flatten().tolist()
        
        self.map_pub.publish(map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
