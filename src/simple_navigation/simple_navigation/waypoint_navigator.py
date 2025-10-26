#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math


def euler_from_quaternion(quaternion):
    """
    Convert quaternion (w, x, y, z) to euler angles (roll, pitch, yaw).
    
    Args:
        quaternion: list or array [x, y, z, w]
    
    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    x, y, z, w = quaternion
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('angle_tolerance', 0.1)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Navigation state
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.is_navigating = False
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Control loop timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Waypoint Navigator initialized. Waiting for goal pose on /goal_pose')
        self.get_logger().info('Send goal using: ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped ...')
    
    def odom_callback(self, msg: Odometry):
        """Update robot's current position from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
    
    def goal_callback(self, msg: PoseStamped):
        """Receive new goal pose and start navigation."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        # Extract goal yaw from quaternion
        orientation_q = msg.pose.orientation
        _, _, self.goal_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        
        self.is_navigating = True
        
        self.get_logger().info(f'New goal received: x={self.goal_x:.2f}, y={self.goal_y:.2f}, yaw={math.degrees(self.goal_yaw):.1f}°')
        
        # Publish straight-line path to goal for visualization
        self.publish_path()
    
    def publish_path(self):
        """Publish a straight-line path from current position to goal for RViz visualization."""
        if self.goal_x is None or self.goal_y is None:
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        # Create waypoints along the straight line
        num_points = 20
        for i in range(num_points + 1):
            t = i / num_points
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = self.current_x + t * (self.goal_x - self.current_x)
            pose.pose.position.y = self.current_y + t * (self.goal_y - self.current_y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info('Published planned path to RViz')
    
    def control_loop(self):
        """Main control loop for navigation."""
        if not self.is_navigating or self.goal_x is None:
            return
        
        # Calculate distance and angle to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        
        # Calculate angle error
        angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)
        
        cmd = Twist()
        
        # Check if reached goal position
        if distance < self.goal_tolerance:
            # Check if need to adjust final orientation
            if self.goal_yaw is not None:
                final_angle_error = self.normalize_angle(self.goal_yaw - self.current_yaw)
                
                if abs(final_angle_error) > self.angle_tolerance:
                    # Rotate to goal orientation
                    cmd.angular.z = self.angular_speed if final_angle_error > 0 else -self.angular_speed
                    self.get_logger().info(f'Adjusting final orientation: error={math.degrees(final_angle_error):.1f}°')
                else:
                    # Goal reached!
                    self.stop_robot()
                    self.is_navigating = False
                    self.get_logger().info('Goal reached!')
                    return
            else:
                # Goal reached!
                self.stop_robot()
                self.is_navigating = False
                self.get_logger().info('Goal reached!')
                return
        else:
            # If angle error is large, rotate first
            if abs(angle_error) > 0.3:  # ~17 degrees
                cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
                self.get_logger().info(f'Rotating to goal: error={math.degrees(angle_error):.1f}°, dist={distance:.2f}m')
            else:
                # Move forward and adjust angle
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.5 * angle_error  # Proportional control for smooth turning
                self.get_logger().info(f'Moving to goal: dist={distance:.2f}m, angle_error={math.degrees(angle_error):.1f}°')
        
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
