#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
import math
import numpy as np
from collections import deque
import tf2_ros
import tf2_geometry_msgs


def euler_from_quaternion(quaternion):
    """Convert quaternion (x, y, z, w) to euler angles (roll, pitch, yaw)."""
    x, y, z, w = quaternion
    
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


class EnhancedLocalPlanner(Node):
    def __init__(self):
        super().__init__('enhanced_local_planner')
        
        # Enhanced parameters
        self.declare_parameter('safety_margin', 0.5)
        self.declare_parameter('obstacle_prediction_time', 2.0)
        self.declare_parameter('max_obstacle_speed', 1.0)  # m/s
        self.declare_parameter('temporal_weight', 0.7)
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('goal_tolerance', 0.15)
        
        self.safety_margin = self.get_parameter('safety_margin').value
        self.obstacle_prediction_time = self.get_parameter('obstacle_prediction_time').value
        self.max_obstacle_speed = self.get_parameter('max_obstacle_speed').value
        self.temporal_weight = self.get_parameter('temporal_weight').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # Enhanced obstacle tracking
        self.obstacle_history = {}  # Track obstacles over time
        self.obstacle_velocities = {}  # Track obstacle movements
        self.last_obstacle_update = self.get_clock().now()
        
        # Temporal fusion
        self.temporal_costmap = None
        self.costmap_history = deque(maxlen=5)  # Keep last 5 costmaps
        
        # Robot state
        self.current_pose = None
        self.current_velocity = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Navigation state
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.is_navigating = False
        
        # Enhanced publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enhanced_path_pub = self.create_publisher(Path, '/enhanced_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/costmap', self.costmap_callback, 10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # TF buffer for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.enhanced_control_loop)
        
        # Obstacle processing timer
        self.obstacle_timer = self.create_timer(0.2, self.process_obstacles)
        
        self.get_logger().info('Enhanced Local Planner initialized')

    def odom_callback(self, msg: Odometry):
        """Update robot state with velocity information."""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
        
        # Extract position and orientation
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

    def goal_callback(self, msg: PoseStamped):
        """Receive new goal pose and start navigation."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        orientation_q = msg.pose.orientation
        _, _, self.goal_yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        
        self.is_navigating = True
        self.get_logger().info(f'New goal received: x={self.goal_x:.2f}, y={self.goal_y:.2f}')

    def costmap_callback(self, msg: OccupancyGrid):
        """Enhanced costmap processing with temporal fusion."""
        self.costmap_msg = msg
        current_costmap = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
        # Store in history
        self.costmap_history.append({
            'costmap': current_costmap,
            'timestamp': self.get_clock().now(),
            'msg': msg
        })
        
        # Apply temporal fusion
        self.apply_temporal_fusion()
        
        # Extract obstacles with confidence
        self.extract_obstacles_with_confidence()

    def apply_temporal_fusion(self):
        """Fuse multiple costmap frames for more complete obstacle information."""
        if len(self.costmap_history) < 2:
            self.temporal_costmap = self.costmap_history[-1]['costmap'] if self.costmap_history else None
            return
        
        # Weight recent observations more heavily
        total_weight = 0
        fused_costmap = np.zeros_like(self.costmap_history[-1]['costmap'], dtype=float)
        
        current_time = self.get_clock().now()
        for i, costmap_data in enumerate(reversed(self.costmap_history)):
            age = (current_time - costmap_data['timestamp']).nanoseconds / 1e9
            weight = math.exp(-age / self.temporal_weight)  # Exponential decay
            
            fused_costmap += costmap_data['costmap'] * weight
            total_weight += weight
        
        if total_weight > 0:
            fused_costmap = (fused_costmap / total_weight).astype(np.uint8)
        
        self.temporal_costmap = fused_costmap

    def extract_obstacles_with_confidence(self):
        """Extract obstacles with confidence scores based on temporal persistence."""
        if self.temporal_costmap is None or self.costmap_msg is None:
            return
        
        current_time = self.get_clock().now()
        resolution = self.costmap_msg.info.resolution
        origin_x = self.costmap_msg.info.origin.position.x
        origin_y = self.costmap_msg.info.origin.position.y
        
        # Find high-cost regions
        obstacle_cells = np.where(self.temporal_costmap >= 50)
        
        # Group connected components
        try:
            from scipy import ndimage
            labeled_array, num_features = ndimage.label(self.temporal_costmap >= 50)
        except ImportError:
            self.get_logger().warn('scipy not available, using simple obstacle detection')
            self.simple_obstacle_detection()
            return
        
        new_obstacles = {}
        
        for i in range(1, num_features + 1):
            obstacle_mask = labeled_array == i
            y_coords, x_coords = np.where(obstacle_mask)
            
            if len(x_coords) < 3:  # Minimum size
                continue
                
            # Calculate obstacle center and bounds
            center_x = np.mean(x_coords)
            center_y = np.mean(y_coords)
            
            world_x = center_x * resolution + origin_x
            world_y = center_y * resolution + origin_y
            
            # Calculate confidence based on size and temporal persistence
            size = len(x_coords)
            confidence = min(1.0, size / 100.0)  # Normalize by size
            
            obstacle_id = f"obstacle_{i}"
            new_obstacles[obstacle_id] = {
                'position': (world_x, world_y),
                'size': size,
                'confidence': confidence,
                'timestamp': current_time,
                'pixel_coords': (x_coords, y_coords)
            }
        
        # Update obstacle history and calculate velocities
        self.update_obstacle_tracking(new_obstacles, current_time)

    def simple_obstacle_detection(self):
        """Simple obstacle detection without scipy dependency."""
        if self.temporal_costmap is None or self.costmap_msg is None:
            return
        
        current_time = self.get_clock().now()
        resolution = self.costmap_msg.info.resolution
        origin_x = self.costmap_msg.info.origin.position.x
        origin_y = self.costmap_msg.info.origin.position.y
        
        height, width = self.temporal_costmap.shape
        new_obstacles = {}
        obstacle_id = 0
        
        # Simple grid-based obstacle detection
        for y in range(height):
            for x in range(width):
                if self.temporal_costmap[y, x] >= 50:
                    # Check if this cell is near an existing obstacle
                    world_x = x * resolution + origin_x
                    world_y = y * resolution + origin_y
                    
                    found_existing = False
                    for obs_id, obs_data in new_obstacles.items():
                        obs_x, obs_y = obs_data['position']
                        dist = math.sqrt((world_x - obs_x)**2 + (world_y - obs_y)**2)
                        if dist < 0.5:  # Group within 0.5m
                            # Update existing obstacle position (average)
                            old_x, old_y = obs_data['position']
                            count = obs_data['size']
                            new_x = (old_x * count + world_x) / (count + 1)
                            new_y = (old_y * count + world_y) / (count + 1)
                            obs_data['position'] = (new_x, new_y)
                            obs_data['size'] += 1
                            obs_data['confidence'] = min(1.0, obs_data['size'] / 100.0)
                            found_existing = True
                            break
                    
                    if not found_existing:
                        obstacle_id += 1
                        new_obstacles[f"obstacle_{obstacle_id}"] = {
                            'position': (world_x, world_y),
                            'size': 1,
                            'confidence': 0.1,
                            'timestamp': current_time
                        }
        
        self.update_obstacle_tracking(new_obstacles, current_time)

    def update_obstacle_tracking(self, new_obstacles, current_time):
        """Track obstacles over time and estimate velocities."""
        # Match new obstacles with previous ones
        for obstacle_id, obstacle_data in new_obstacles.items():
            best_match = None
            min_distance = float('inf')
            
            for old_id, old_data in self.obstacle_history.items():
                if (current_time - old_data['timestamp']).nanoseconds / 1e9 > 5.0:
                    continue
                    
                dist = math.sqrt(
                    (obstacle_data['position'][0] - old_data['position'][0])**2 +
                    (obstacle_data['position'][1] - old_data['position'][1])**2
                )
                
                if dist < min_distance and dist < 2.0:  # Maximum matching distance
                    min_distance = dist
                    best_match = old_id
            
            if best_match:
                # Calculate velocity
                dt = (current_time - self.obstacle_history[best_match]['timestamp']).nanoseconds / 1e9
                if dt > 0:
                    dx = obstacle_data['position'][0] - self.obstacle_history[best_match]['position'][0]
                    dy = obstacle_data['position'][1] - self.obstacle_history[best_match]['position'][1]
                    
                    velocity = math.sqrt(dx**2 + dy**2) / dt
                    direction = math.atan2(dy, dx)
                    
                    self.obstacle_velocities[obstacle_id] = {
                        'speed': min(velocity, self.max_obstacle_speed),
                        'direction': direction,
                        'last_update': current_time
                    }
                
                # Update obstacle data
                obstacle_data['id'] = best_match
                if best_match in self.obstacle_history:
                    del self.obstacle_history[best_match]
        
        # Update history
        self.obstacle_history = new_obstacles
        self.last_obstacle_update = current_time

    def predict_obstacle_positions(self, prediction_time):
        """Predict future obstacle positions."""
        predicted_obstacles = {}
        current_time = self.get_clock().now()
        
        for obstacle_id, obstacle_data in self.obstacle_history.items():
            if obstacle_id not in self.obstacle_velocities:
                predicted_obstacles[obstacle_id] = obstacle_data
                continue
            
            velocity_data = self.obstacle_velocities[obstacle_id]
            
            # Predict future position
            current_x, current_y = obstacle_data['position']
            speed = velocity_data['speed']
            direction = velocity_data['direction']
            
            future_x = current_x + speed * prediction_time * math.cos(direction)
            future_y = current_y + speed * prediction_time * math.sin(direction)
            
            predicted_obstacles[obstacle_id] = {
                **obstacle_data,
                'position': (future_x, future_y),
                'predicted': True
            }
        
        return predicted_obstacles

    def enhanced_control_loop(self):
        """Enhanced control loop with obstacle prediction and avoidance."""
        if self.current_pose is None or not self.is_navigating:
            return
        
        # Check if goal reached
        if self.goal_x is not None and self.goal_y is not None:
            dist_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
            if dist_to_goal < self.goal_tolerance:
                self.stop_robot()
                self.is_navigating = False
                self.get_logger().info('🎉 Goal reached!')
                return
        
        # Get predicted obstacles
        predicted_obstacles = self.predict_obstacle_positions(self.obstacle_prediction_time)
        
        # Calculate safe velocity command
        cmd_vel = self.calculate_safe_velocity(predicted_obstacles)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Visualize obstacles and predictions
        self.visualize_obstacles(predicted_obstacles)

    def calculate_safe_velocity(self, obstacles):
        """Calculate safe velocity considering predicted obstacles."""
        cmd_vel = Twist()
        
        if not self.is_navigating or self.goal_x is None:
            return cmd_vel
        
        # Calculate basic goal direction
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        goal_distance = math.sqrt(dx**2 + dy**2)
        goal_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(goal_angle - self.current_yaw)
        
        # Check for obstacles in the path
        safe_directions = self.find_safe_directions(obstacles, goal_angle)
        
        if safe_directions['forward']:
            # Safe to move forward
            cmd_vel.linear.x = min(self.linear_speed, goal_distance * 0.5)
            cmd_vel.angular.z = 2.0 * angle_error
        elif safe_directions['left'] and safe_directions['right']:
            # Choose direction with smaller angle deviation
            if abs(safe_directions['left_angle']) < abs(safe_directions['right_angle']):
                cmd_vel.angular.z = self.angular_speed
            else:
                cmd_vel.angular.z = -self.angular_speed
        elif safe_directions['left']:
            cmd_vel.angular.z = self.angular_speed
        elif safe_directions['right']:
            cmd_vel.angular.z = -self.angular_speed
        else:
            # Completely blocked, rotate in place
            cmd_vel.angular.z = self.angular_speed
        
        return cmd_vel

    def find_safe_directions(self, obstacles, goal_angle):
        """Find safe directions considering obstacles."""
        safe_distance = 1.0  # meters
        
        # Check forward, left, and right sectors
        sectors = {
            'forward': (-0.4, 0.4),    # -23° to +23°
            'left': (0.4, 1.57),        # +23° to +90°
            'right': (-1.57, -0.4)      # -90° to -23°
        }
        
        safe_directions = {
            'forward': True,
            'left': True,
            'right': True,
            'left_angle': 0.0,
            'right_angle': 0.0
        }
        
        for obstacle_id, obstacle_data in obstacles.items():
            if obstacle_data['confidence'] < self.confidence_threshold:
                continue
                
            obs_x, obs_y = obstacle_data['position']
            dist = math.sqrt((obs_x - self.current_x)**2 + (obs_y - self.current_y)**2)
            
            if dist > safe_distance + self.safety_margin:
                continue
                
            # Calculate angle to obstacle relative to robot heading
            dx_obs = obs_x - self.current_x
            dy_obs = obs_y - self.current_y
            angle_to_obs = math.atan2(dy_obs, dx_obs)
            relative_angle = self.normalize_angle(angle_to_obs - self.current_yaw)
            
            # Check which sectors are blocked
            for sector_name, (min_angle, max_angle) in sectors.items():
                if min_angle <= relative_angle <= max_angle:
                    safe_directions[sector_name] = False
            
            # Calculate alternative directions
            if relative_angle > 0:  # Obstacle on left
                safe_directions['left_angle'] = relative_angle - 0.8  # Bias to go right
            else:  # Obstacle on right
                safe_directions['right_angle'] = relative_angle + 0.8  # Bias to go left
        
        return safe_directions

    def process_obstacles(self):
        """Periodic obstacle processing and cleanup."""
        current_time = self.get_clock().now()
        
        # Remove old obstacles
        expired_obstacles = []
        for obstacle_id, obstacle_data in self.obstacle_history.items():
            if (current_time - obstacle_data['timestamp']).nanoseconds / 1e9 > 5.0:
                expired_obstacles.append(obstacle_id)
        
        for obstacle_id in expired_obstacles:
            if obstacle_id in self.obstacle_history:
                del self.obstacle_history[obstacle_id]
            if obstacle_id in self.obstacle_velocities:
                del self.obstacle_velocities[obstacle_id]

    def visualize_obstacles(self, obstacles):
        """Visualize obstacles and predictions in RViz."""
        marker_array = MarkerArray()
        
        # Clear all markers first
        clear_marker = Marker()
        clear_marker.header.frame_id = "odom"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = "obstacles"
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Current obstacles
        for i, (obstacle_id, obstacle_data) in enumerate(obstacles.items()):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i + 1  # Start from 1 after clear marker
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = obstacle_data['position'][0]
            marker.pose.position.y = obstacle_data['position'][1]
            marker.pose.position.z = 0.0
            
            # Scale based on obstacle size/confidence
            base_size = 0.3
            size_factor = 0.5 + obstacle_data['confidence'] * 0.5
            marker.scale.x = base_size * size_factor
            marker.scale.y = base_size * size_factor
            marker.scale.z = 0.5
            
            # Color based on confidence and prediction
            if obstacle_data.get('predicted', False):
                marker.color.a = 0.5  # Transparent for predictions
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            else:
                marker.color.a = 0.8
                # Red for high confidence, yellow for medium, green for low
                confidence = obstacle_data['confidence']
                if confidence > 0.7:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif confidence > 0.3:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

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
    node = EnhancedLocalPlanner()
    
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