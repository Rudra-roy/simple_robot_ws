#!/usr/bin/env python3
"""
Global Planner Node
Receives goal poses and plans a direct path to the goal.
Checks for obstacles and triggers local planner when needed.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer
import math
import numpy as np
import struct


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.3)  # meters
        self.declare_parameter('safety_margin', 0.2)  # meters
        self.declare_parameter('linear_velocity', 0.5)  # m/s
        self.declare_parameter('angular_velocity', 0.5)  # rad/s
        self.declare_parameter('goal_tolerance', 0.1)  # meters
        self.declare_parameter('angular_tolerance', 0.1)  # radians
        self.declare_parameter('obstacle_check_distance', 2.0)  # meters
        self.declare_parameter('costmap_obstacle_threshold', 50)  # 0-100
        
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.angular_tolerance = self.get_parameter('angular_tolerance').value
        self.obstacle_check_distance = self.get_parameter('obstacle_check_distance').value
        self.costmap_threshold = self.get_parameter('costmap_obstacle_threshold').value
        
        # State variables
        self.current_pose = None
        self.goal_pose = None
        self.costmap = None
        self.is_aligned = False
        self.obstacle_detected = False
        self.robot_stopped = False
        
        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos_reliable
        )
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/costmap',
            self.costmap_callback,
            qos_reliable
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            qos_best_effort
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.robot_boundary_pub = self.create_publisher(
            PointCloud2,
            '/robot_boundary_cloud',
            10
        )
        
        self.path_marker_pub = self.create_publisher(
            Marker,
            '/planned_path_marker',
            10
        )
        
        self.goal_marker_pub = self.create_publisher(
            Marker,
            '/goal_marker',
            10
        )
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for control loop
        self.create_timer(0.1, self.control_loop)
        
        # Timer for visualization
        self.create_timer(0.2, self.publish_visualizations)
        
        self.get_logger().info('Global Planner Node initialized')
        self.get_logger().info(f'Robot radius: {self.robot_radius}m, Safety margin: {self.safety_margin}m')
    
    def goal_callback(self, msg: PoseStamped):
        """Receive new goal pose"""
        self.goal_pose = msg
        self.is_aligned = False
        self.obstacle_detected = False
        self.robot_stopped = False
        
        self.get_logger().info(f'🎯 Received new goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        self.get_logger().info('📊 Generating path visualization...')
        
        # Immediately publish path visualization
        self.publish_path_to_goal()
        self.publish_goal_marker()
    
    def odom_callback(self, msg: Odometry):
        """Update current robot pose from odometry"""
        if self.current_pose is None:
            self.get_logger().info(f'✓ Odometry received: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
        self.current_pose = msg.pose.pose
    
    def costmap_callback(self, msg: OccupancyGrid):
        """Receive costmap updates"""
        self.costmap = msg
    
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def get_distance_to_goal(self):
        """Calculate distance to goal"""
        if self.current_pose is None or self.goal_pose is None:
            return float('inf')
        
        dx = self.goal_pose.pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def get_angle_to_goal(self):
        """Calculate angle to goal relative to robot's current orientation"""
        if self.current_pose is None or self.goal_pose is None:
            return 0.0
        
        dx = self.goal_pose.pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.position.y
        
        goal_angle = math.atan2(dy, dx)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        angle_diff = self.normalize_angle(goal_angle - current_yaw)
        return angle_diff
    
    def check_obstacles_in_path(self):
        """Check if there are obstacles in the direct path to goal"""
        if self.costmap is None or self.current_pose is None or self.goal_pose is None:
            return False
        
        # Get costmap info
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        height = self.costmap.info.height
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        
        # Current position
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Goal direction
        dx = self.goal_pose.pose.position.x - robot_x
        dy = self.goal_pose.pose.position.y - robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.01:
            return False
        
        # Normalize direction
        dx /= distance
        dy /= distance
        
        # Check distance (limit to obstacle_check_distance)
        check_distance = min(distance, self.obstacle_check_distance)
        
        # Check points along the path
        num_checks = int(check_distance / resolution) + 1
        robot_width = self.robot_radius + self.safety_margin
        
        for i in range(1, num_checks):
            t = (i * resolution)
            check_x = robot_x + dx * t
            check_y = robot_y + dy * t
            
            # Check area around the path (robot width)
            for offset in np.linspace(-robot_width, robot_width, 5):
                # Perpendicular offset
                offset_x = check_x - dy * offset
                offset_y = check_y + dx * offset
                
                # Convert to grid coordinates
                grid_x = int((offset_x - origin_x) / resolution)
                grid_y = int((offset_y - origin_y) / resolution)
                
                # Check bounds
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    index = grid_y * width + grid_x
                    if index < len(self.costmap.data):
                        cost = self.costmap.data[index]
                        if cost > self.costmap_threshold:
                            return True
        
        return False
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def control_loop(self):
        """Main control loop"""
        if self.goal_pose is None:
            return
        
        if self.current_pose is None:
            self.get_logger().warn('⚠️ Waiting for odometry data...', throttle_duration_sec=2.0)
            return
        
        distance = self.get_distance_to_goal()
        self.get_logger().info(f'🚀 Control loop running: distance={distance:.2f}m', throttle_duration_sec=1.0)
        
        # Check if goal reached
        if distance < self.goal_tolerance:
            self.stop_robot()
            self.get_logger().info('Goal reached!', throttle_duration_sec=2.0)
            self.goal_pose = None
            return
        
        # Check for obstacles in path
        obstacle_in_path = self.check_obstacles_in_path()
        
        if obstacle_in_path:
            if not self.obstacle_detected:
                self.get_logger().warn('Obstacle found in the path')
                self.get_logger().info('Preparing local planner for obstacle avoidance')
                self.obstacle_detected = True
            
            self.stop_robot()
            self.robot_stopped = True
            return
        else:
            if self.obstacle_detected:
                self.get_logger().info('Path is clear, resuming navigation')
                self.obstacle_detected = False
                self.robot_stopped = False
        
        # Calculate angle to goal
        angle_to_goal = self.get_angle_to_goal()
        
        cmd = Twist()
        
        # Align to goal direction
        if abs(angle_to_goal) > self.angular_tolerance:
            # Rotate in place
            cmd.angular.z = self.angular_vel if angle_to_goal > 0 else -self.angular_vel
            cmd.linear.x = 0.0
            self.is_aligned = False
            self.get_logger().info(
                f'Aligning to goal: angle_error={math.degrees(angle_to_goal):.1f}°',
                throttle_duration_sec=1.0
            )
        else:
            # Move forward
            self.is_aligned = True
            cmd.linear.x = self.linear_vel
            cmd.angular.z = 0.0
            self.get_logger().info(
                f'Moving forward: distance={distance:.2f}m',
                throttle_duration_sec=1.0
            )
        
        self.cmd_vel_pub.publish(cmd)
    
    def publish_visualizations(self):
        """Publish all visualizations"""
        if self.current_pose is None:
            self.get_logger().warn('⚠️ No odometry for visualization', throttle_duration_sec=5.0)
        self.publish_robot_boundary()
        if self.goal_pose is not None:
            self.publish_path_to_goal()
            self.publish_goal_marker()
    
    def publish_robot_boundary(self):
        """Publish point cloud circle around robot boundary"""
        if self.current_pose is None:
            return
        
        try:
            # Generate circle of points at 0.3m radius
            num_points = 360  # Dense circle (every 1 degree)
            radius = 0.3  # 0.3m radius
            
            # Get robot's current yaw
            robot_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
        
        # Transform circle points to odom frame
            points = []
            for i in range(num_points):
                angle = 2.0 * math.pi * i / num_points
                # Local circle point
                local_x = radius * math.cos(angle)
                local_y = radius * math.sin(angle)
                
                # Rotate by robot yaw and translate to robot position
                x = robot_x + local_x * math.cos(robot_yaw) - local_y * math.sin(robot_yaw)
                y = robot_y + local_x * math.sin(robot_yaw) + local_y * math.cos(robot_yaw)
                z = 0.0
                points.append([x, y, z])
        
            # Create PointCloud2 message
            cloud_msg = PointCloud2()
            cloud_msg.header.frame_id = "odom"  # Publish in odom frame
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Define point cloud fields (x, y, z)
            cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            cloud_msg.is_bigendian = False
            cloud_msg.point_step = 12  # 3 floats * 4 bytes
            cloud_msg.row_step = cloud_msg.point_step * len(points)
            cloud_msg.is_dense = True
            cloud_msg.height = 1
            cloud_msg.width = len(points)
            
            # Pack point data
            buffer = []
            for point in points:
                buffer.append(struct.pack('fff', point[0], point[1], point[2]))
            
            cloud_msg.data = b''.join(buffer)
            
            self.robot_boundary_pub.publish(cloud_msg)
            self.get_logger().info(f'📍 Published boundary cloud with {len(points)} points', throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f'❌ Failed to publish boundary cloud: {e}')
    
    def publish_path_to_goal(self):
        """Publish line strip showing path from robot to goal"""
        if self.current_pose is None or self.goal_pose is None:
            return
        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "global_path"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Start point (robot position)
        start_point = Point()
        start_point.x = self.current_pose.position.x
        start_point.y = self.current_pose.position.y
        start_point.z = 0.1  # Slightly above ground
        marker.points.append(start_point)
        
        # End point (goal position)
        end_point = Point()
        end_point.x = self.goal_pose.pose.position.x
        end_point.y = self.goal_pose.pose.position.y
        end_point.z = 0.1
        marker.points.append(end_point)
        
        # Line properties
        marker.scale.x = 0.05  # Line width
        
        # Color (bright green if clear, yellow if obstacle)
        if self.obstacle_detected:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        self.path_marker_pub.publish(marker)
    
    def publish_goal_marker(self):
        """Publish goal position marker (arrow)"""
        if self.goal_pose is None:
            return
        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position and orientation from goal pose
        marker.pose = self.goal_pose.pose
        marker.pose.position.z = 0.2  # Raise above ground for visibility
        
        # Arrow size
        marker.scale.x = 0.5  # Length
        marker.scale.y = 0.1  # Width
        marker.scale.z = 0.1  # Height
        
        # Color (bright red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        self.goal_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
