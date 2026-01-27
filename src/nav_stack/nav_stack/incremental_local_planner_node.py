#!/usr/bin/env python3
"""
Incremental Local Planner Node
Uses 10-degree rotation increments with costmap-based obstacle checking.
Analyzes -120° to +120° range for safe direction with goal bias.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker
import math
import numpy as np
from enum import Enum


class PlannerState(Enum):
    IDLE = 0
    ANALYZING = 1
    ROTATING = 2
    MOVING_FORWARD = 3
    RETURNING_TO_START = 4


class IncrementalLocalPlannerNode(Node):
    def __init__(self):
        super().__init__('incremental_local_planner_node')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.6)
        self.declare_parameter('linear_velocity', 1.2)
        self.declare_parameter('angular_velocity', 3.0)  # Fast rotation for 20° increments
        self.declare_parameter('costmap_obstacle_threshold', 70)
        self.declare_parameter('forward_duration', 3.0)
        self.declare_parameter('obstacle_check_distance', 5.0)
        self.declare_parameter('rotation_increment', 20.0)  # degrees
        self.declare_parameter('max_rotation_angle', 120.0)  # degrees
        self.declare_parameter('direction_scan_range', 60.0)  # -60 to +60 degrees
        
        self.robot_radius = self.get_parameter('robot_radius').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.costmap_threshold = self.get_parameter('costmap_obstacle_threshold').value
        self.forward_duration = self.get_parameter('forward_duration').value
        self.obstacle_check_distance = self.get_parameter('obstacle_check_distance').value
        self.rotation_increment = math.radians(self.get_parameter('rotation_increment').value)
        self.max_rotation_angle = math.radians(self.get_parameter('max_rotation_angle').value)
        self.scan_range = math.radians(self.get_parameter('direction_scan_range').value)
        
        # State
        self.state = PlannerState.IDLE
        self.current_pose = None
        self.goal_pose = None
        self.costmap = None
        
        # Direction tracking
        self.chosen_direction = None  # "left" or "right"
        self.starting_yaw = None  # Starting orientation when planner triggered
        self.current_rotation_count = 0.0  # Accumulated rotation in radians
        self.target_yaw = None  # Target yaw for current rotation step
        self.has_tried_opposite = False  # Track if we've already tried opposite direction
        self.rotation_start_time = None  # Start time of current rotation
        self.rotation_start_yaw = None  # Yaw when rotation started
        
        # Forward movement tracking
        self.forward_timer_start = None
        
        # Starting position for visualization
        self.starting_position = None
        
        # Scan ray data for visualization
        self.scan_ray_data = []  # List of (angle, free_distance) tuples
        
        # Instruction printing
        self.last_instruction_time = None
        
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
        self.create_subscription(
            Bool,
            '/costmap_local_planner_trigger',
            self.trigger_callback,
            10
        )
        
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos_reliable
        )
        
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_best_effort
        )
        
        self.create_subscription(
            OccupancyGrid,
            '/costmap',
            self.costmap_callback,
            qos_reliable
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.planner_state_pub = self.create_publisher(String, '/costmap_planner_state', 10)
        
        # Visualization publishers
        self.scan_rays_pub = self.create_publisher(
            Marker,
            '/incremental_local/scan_directions',
            10
        )
        
        self.chosen_direction_pub = self.create_publisher(
            Marker,
            '/incremental_local/chosen_direction',
            10
        )
        
        self.rotation_progress_pub = self.create_publisher(
            Marker,
            '/incremental_local/rotation_progress',
            10
        )
        
        self.forward_path_pub = self.create_publisher(
            Marker,
            '/incremental_local/forward_path_preview',
            10
        )
        
        self.starting_position_pub = self.create_publisher(
            Marker,
            '/incremental_local/starting_position',
            10
        )
        
        self.goal_bias_pub = self.create_publisher(
            Marker,
            '/incremental_local/goal_bias_arrow',
            10
        )
        
        # Timer for state machine
        self.create_timer(0.1, self.state_machine_loop)
        self.create_timer(0.5, self.print_movement_instruction)
        self.create_timer(0.2, self.publish_visualizations)
        
        self.get_logger().info('Incremental Local Planner initialized')
        self.get_logger().info('20° rotation increments, 3s forward, 120° limit')
    
    def trigger_callback(self, msg: Bool):
        """Triggered by global planner when obstacle detected"""
        if msg.data and self.state == PlannerState.IDLE:
            self.get_logger().info('Local planner activated - analyzing direction')
            self.starting_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            self.starting_position = (self.current_pose.position.x, self.current_pose.position.y)
            self.current_rotation_count = 0.0
            self.has_tried_opposite = False
            self.state = PlannerState.ANALYZING
            self.last_instruction_time = None
    
    def goal_callback(self, msg: PoseStamped):
        """Update goal pose"""
        self.goal_pose = msg
    
    def odom_callback(self, msg: Odometry):
        """Update current pose"""
        self.current_pose = msg.pose.pose
    
    def costmap_callback(self, msg: OccupancyGrid):
        """Update local costmap"""
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
    
    def print_movement_instruction(self):
        """Print movement instructions every 0.5 seconds"""
        if self.current_pose is None or self.goal_pose is None:
            return
        
        if self.state == PlannerState.IDLE:
            return
        
        current_time = self.get_clock().now()
        if self.last_instruction_time is not None:
            elapsed = (current_time - self.last_instruction_time).nanoseconds / 1e9
            if elapsed < 0.4:
                return
        
        if self.state == PlannerState.ANALYZING:
            self.get_logger().info('Analyzing direction (-120° to +120°)...')
            self.last_instruction_time = current_time
        
        elif self.state == PlannerState.ROTATING:
            direction = "LEFT" if self.chosen_direction == "left" else "RIGHT"
            rotation_deg = math.degrees(self.current_rotation_count)
            self.get_logger().info(f'Rotating {direction} - {rotation_deg:.0f}° of 120° limit')
            self.last_instruction_time = current_time
        
        elif self.state == PlannerState.MOVING_FORWARD:
            elapsed = (self.get_clock().now() - self.forward_timer_start).nanoseconds / 1e9 if self.forward_timer_start else 0
            self.get_logger().info(f'Moving forward ({elapsed:.1f}s/3.0s)')
            self.last_instruction_time = current_time
        
        elif self.state == PlannerState.RETURNING_TO_START:
            self.get_logger().info('Returning to starting position...')
            self.last_instruction_time = current_time
    
    def state_machine_loop(self):
        """Main state machine"""
        if self.current_pose is None or self.goal_pose is None:
            return
        
        if self.state == PlannerState.IDLE:
            pass
        
        elif self.state == PlannerState.ANALYZING:
            self.execute_analysis()
        
        elif self.state == PlannerState.ROTATING:
            self.execute_rotation()
        
        elif self.state == PlannerState.MOVING_FORWARD:
            self.execute_forward_movement()
        
        elif self.state == PlannerState.RETURNING_TO_START:
            self.execute_return_to_start()
    
    def execute_analysis(self):
        """
        Analyze -60° to +60° range from current heading.
        Choose direction with goal bias and free space.
        """
        if self.costmap is None:
            self.get_logger().warn('No costmap available')
            self.state = PlannerState.IDLE
            return
        
        # Stop robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info('STOPPED - Scanning -60° to +60° range')
        
        # Find best direction
        turn_direction = self.find_best_direction()
        
        if turn_direction is None:
            self.get_logger().error('No free space found in scan range')
            self.signal_global_planner()
            return
        
        self.chosen_direction = turn_direction
        self.get_logger().info(f'Chosen direction: {turn_direction.upper()}')
        
        # Set first rotation target
        self.set_next_rotation_target()
        self.state = PlannerState.ROTATING
    
    def find_best_direction(self):
        """
        Scan from -60° to +60° relative to current heading.
        Score each direction based on free space and goal alignment.
        Return "left" or "right" based on which side has better score.
        """
        if self.costmap is None:
            return None
        
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        height = self.costmap.info.height
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # Goal direction
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        goal_dx = goal_x - robot_x
        goal_dy = goal_y - robot_y
        goal_angle = math.atan2(goal_dy, goal_dx)
        
        # Scan from -60° to +60°
        angle_samples = np.linspace(-self.scan_range, self.scan_range, 120)
        
        left_score = 0.0
        right_score = 0.0
        
        # Clear previous scan data
        self.scan_ray_data = []
        
        for relative_angle in angle_samples:
            scan_angle = current_yaw + relative_angle
            
            # Measure free distance in this direction
            free_distance = self.measure_free_distance(scan_angle, robot_x, robot_y, 
                                                      origin_x, origin_y, resolution, width, height)
            
            # Store for visualization
            self.scan_ray_data.append((scan_angle, free_distance))
            
            # Calculate goal alignment score
            angle_to_goal = abs(self.normalize_angle(scan_angle - goal_angle))
            goal_bias = 1.0 - (angle_to_goal / math.pi)  # Higher score when closer to goal
            
            # Combined score
            score = free_distance * (0.6 + 0.4 * goal_bias)
            
            # Accumulate to left or right
            if relative_angle > 0:  # Left side
                left_score += score
            else:  # Right side
                right_score += score
        
        self.get_logger().info(f'Direction scores - Left: {left_score:.2f}, Right: {right_score:.2f}')
        
        # Choose better side
        if left_score > right_score:
            return "left"
        elif right_score > left_score:
            return "right"
        else:
            # If equal, choose side closer to goal
            left_to_goal = abs(self.normalize_angle((current_yaw + self.scan_range/2) - goal_angle))
            right_to_goal = abs(self.normalize_angle((current_yaw - self.scan_range/2) - goal_angle))
            return "left" if left_to_goal < right_to_goal else "right"
    
    def measure_free_distance(self, angle, robot_x, robot_y, origin_x, origin_y, resolution, width, height):
        """Measure free distance in given direction"""
        max_distance = self.obstacle_check_distance
        
        for dist in np.arange(0.5, max_distance, resolution):
            ray_x = robot_x + dist * math.cos(angle)
            ray_y = robot_y + dist * math.sin(angle)
            
            grid_x = int((ray_x - origin_x) / resolution)
            grid_y = int((ray_y - origin_y) / resolution)
            
            if 0 <= grid_x < width and 0 <= grid_y < height:
                index = grid_y * width + grid_x
                if index < len(self.costmap.data):
                    cost = self.costmap.data[index]
                    if cost > self.costmap_threshold:
                        return dist
            else:
                return dist
        
        return max_distance
    
    def set_next_rotation_target(self):
        """Set target yaw for next 20° rotation increment"""
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # Record starting position for this rotation step
        self.rotation_start_yaw = current_yaw
        self.rotation_start_time = self.get_clock().now()
        
        if self.chosen_direction == "left":
            self.target_yaw = current_yaw + self.rotation_increment
        else:  # right
            self.target_yaw = current_yaw - self.rotation_increment
        
        self.target_yaw = self.normalize_angle(self.target_yaw)
    
    def execute_rotation(self):
        """
        Rotate to target_yaw in 20° increments.
        Uses time-based estimation to stop and check angle.
        When target reached, check if path is clear.
        If clear: move forward. If not: rotate another 20°.
        If 120° limit reached: switch sides.
        """
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # Calculate how much we've rotated from start
        rotated_angle = abs(self.normalize_angle(current_yaw - self.rotation_start_yaw))
        
        # Fixed timeout for real robot with acceleration limits
        # Give robot 3 seconds to complete 20° rotation (plenty of time for accel/decel)
        elapsed_time = (self.get_clock().now() - self.rotation_start_time).nanoseconds / 1e9
        
        # Primary check: angle achieved (16° out of 20° = 80%)
        # Secondary check: fixed 3-second timeout for robot dynamics
        angle_achieved = rotated_angle >= self.rotation_increment * 0.80  # 80% of target (16° out of 20°)
        timeout = elapsed_time > 3.0  # Fixed 3-second timeout
        
        # Only stop if angle is achieved OR we've exceeded safety timeout
        if angle_achieved or timeout:
            # Stop and check angle
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            
            if timeout and not angle_achieved:
                self.get_logger().warn(f'Rotation timeout! Only rotated {math.degrees(rotated_angle):.1f}° in {elapsed_time:.2f}s')
            else:
                self.get_logger().info(f'Rotated {math.degrees(rotated_angle):.1f}° - checking path...')
            
            # Check if path ahead is clear
            if self.check_path_clear():
                self.get_logger().info('Path clear - moving forward for 3s')
                self.forward_timer_start = self.get_clock().now()
                self.state = PlannerState.MOVING_FORWARD
                return
            
            # Path not clear - check rotation limit
            self.current_rotation_count += self.rotation_increment
            
            if self.current_rotation_count >= self.max_rotation_angle:
                self.get_logger().warn(f'120° limit reached on {self.chosen_direction} side')
                
                if not self.has_tried_opposite:
                    self.get_logger().info('Returning to start and trying opposite direction')
                    self.state = PlannerState.RETURNING_TO_START
                else:
                    self.get_logger().error('Both directions exhausted - no path found')
                    self.signal_global_planner()
                return
            
            # Continue rotating - set next target
            self.get_logger().info('Obstacle found - rotating another 20°')
            self.set_next_rotation_target()
            return
        
        # Continue rotating at constant speed
        cmd = Twist()
        cmd.linear.x = 0.0
        
        if self.chosen_direction == "left":
            cmd.angular.z = self.angular_vel
        else:
            cmd.angular.z = -self.angular_vel
        
        self.cmd_vel_pub.publish(cmd)
    
    def check_path_clear(self):
        """
        Check if path ahead is clear with rover boundary width.
        Check up to obstacle_check_distance meters.
        """
        if self.costmap is None or self.current_pose is None:
            return False
        
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        height = self.costmap.info.height
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # Direction vector
        dx_norm = math.cos(current_yaw)
        dy_norm = math.sin(current_yaw)
        
        # Check path
        num_samples = int(self.obstacle_check_distance / resolution) + 1
        
        for i in range(num_samples):
            t = i * resolution
            if t > self.obstacle_check_distance:
                break
            
            check_x = robot_x + dx_norm * t
            check_y = robot_y + dy_norm * t
            
            # Check circular footprint (robot_radius)
            for angle in np.linspace(0, 2*math.pi, 12):
                px = check_x + self.robot_radius * math.cos(angle)
                py = check_y + self.robot_radius * math.sin(angle)
                
                grid_x = int((px - origin_x) / resolution)
                grid_y = int((py - origin_y) / resolution)
                
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    index = grid_y * width + grid_x
                    if index < len(self.costmap.data):
                        cost = self.costmap.data[index]
                        if cost > self.costmap_threshold:
                            return False
                else:
                    return False
        
        return True
    
    def execute_forward_movement(self):
        """
        Move forward for 3 seconds.
        Check for obstacles during movement.
        When complete, return to global planner.
        """
        # Check for obstacles
        if not self.check_path_clear():
            self.get_logger().warn('Obstacle detected during forward movement')
            # Reset rotation count and try again from current position
            self.starting_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            self.current_rotation_count = 0.0
            self.state = PlannerState.ANALYZING
            return
        
        # Check if 3 seconds elapsed
        elapsed = (self.get_clock().now() - self.forward_timer_start).nanoseconds / 1e9
        
        if elapsed >= self.forward_duration:
            self.get_logger().info('3 seconds completed - returning to global planner')
            self.signal_global_planner()
            return
        
        # Move forward
        cmd = Twist()
        cmd.linear.x = self.linear_vel
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def execute_return_to_start(self):
        """
        Rotate back to starting orientation.
        Then switch to opposite direction.
        """
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        angle_diff = self.normalize_angle(self.starting_yaw - current_yaw)
        
        # Check if back at starting position (within 1 degree)
        if abs(angle_diff) < math.radians(1.0):
            # Stop
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            
            self.get_logger().info('Returned to starting orientation')
            
            # Switch direction
            self.chosen_direction = "right" if self.chosen_direction == "left" else "left"
            self.current_rotation_count = 0.0
            self.has_tried_opposite = True
            
            self.get_logger().info(f'Switching to {self.chosen_direction.upper()} direction')
            
            # Set first rotation target in new direction
            self.set_next_rotation_target()
            self.state = PlannerState.ROTATING
            return
        
        # Continue rotating back with proportional control
        cmd = Twist()
        cmd.linear.x = 0.0
        
        # Proportional control for smooth return
        if abs(angle_diff) > math.radians(5):
            angular_speed = self.angular_vel
        else:
            angular_speed = self.angular_vel * (abs(angle_diff) / math.radians(5))
            angular_speed = max(angular_speed, 0.1)
        
        if angle_diff > 0:
            cmd.angular.z = angular_speed
        else:
            cmd.angular.z = -angular_speed
        
        self.cmd_vel_pub.publish(cmd)
    
    def signal_global_planner(self):
        """Return control to global planner"""
        self.get_logger().info('Signaling global planner to resume')
        
        # Stop robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # Reset state
        self.state = PlannerState.IDLE
        self.chosen_direction = None
        self.starting_yaw = None
        self.starting_position = None
        self.current_rotation_count = 0.0
        self.target_yaw = None
        self.forward_timer_start = None
        self.has_tried_opposite = False
        self.rotation_start_time = None
        self.rotation_start_yaw = None
        self.scan_ray_data = []
        
        # Publish state
        state_msg = String()
        state_msg.data = "global_active"
        self.planner_state_pub.publish(state_msg)
    
    def publish_visualizations(self):
        """Publish all visualization markers"""
        if self.current_pose is None:
            return
        
        # Always publish if we have data
        if len(self.scan_ray_data) > 0:
            self.publish_scan_rays()
        
        if self.chosen_direction is not None:
            self.publish_chosen_direction()
            self.publish_rotation_progress()
        
        if self.state == PlannerState.MOVING_FORWARD or self.state == PlannerState.ROTATING:
            self.publish_forward_path()
        
        if self.starting_position is not None:
            self.publish_starting_position()
        
        if self.goal_pose is not None:
            self.publish_goal_bias()
    
    def publish_scan_rays(self):
        """Publish scan direction rays as LINE_LIST"""
        if self.current_pose is None or len(self.scan_ray_data) == 0:
            return
        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scan_directions"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Create line segments for each ray
        for angle, free_distance in self.scan_ray_data:
            # Start point (robot position)
            start = Point()
            start.x = robot_x
            start.y = robot_y
            start.z = 0.05
            marker.points.append(start)
            
            # End point (free distance along ray)
            end = Point()
            end.x = robot_x + free_distance * math.cos(angle)
            end.y = robot_y + free_distance * math.sin(angle)
            end.z = 0.05
            marker.points.append(end)
            
            # Color based on free distance (gradient from red to green)
            color_value = min(free_distance / self.obstacle_check_distance, 1.0)
            marker.colors.append(self._create_color(1.0 - color_value, color_value, 0.0, 0.6))
            marker.colors.append(self._create_color(1.0 - color_value, color_value, 0.0, 0.6))
        
        marker.scale.x = 0.02  # Line width
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        self.scan_rays_pub.publish(marker)
    
    def publish_chosen_direction(self):
        """Publish large arrow showing chosen turn direction"""
        if self.current_pose is None or self.chosen_direction is None:
            return
        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "chosen_direction"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # Arrow pointing in chosen direction (90 degrees left or right)
        if self.chosen_direction == "left":
            direction_angle = current_yaw + math.pi / 2
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0  # Blue
        else:
            direction_angle = current_yaw - math.pi / 2
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0  # Orange
        
        marker.pose.position.x = self.current_pose.position.x
        marker.pose.position.y = self.current_pose.position.y
        marker.pose.position.z = 0.3  # Raised for visibility
        
        marker.pose.orientation.z = math.sin(direction_angle / 2.0)
        marker.pose.orientation.w = math.cos(direction_angle / 2.0)
        
        marker.scale.x = 1.5  # Length
        marker.scale.y = 0.15  # Width
        marker.scale.z = 0.15  # Height
        marker.color.a = 0.9
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        self.chosen_direction_pub.publish(marker)
    
    def publish_rotation_progress(self):
        """Publish arc showing rotation progress toward 120 degree limit"""
        if self.current_pose is None or self.starting_yaw is None:
            return
        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rotation_progress"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        arc_radius = 1.0
        
        # Draw arc from starting yaw to current rotation
        num_segments = 20
        rotation_fraction = self.current_rotation_count / self.max_rotation_angle
        
        if self.chosen_direction == "left":
            angle_start = self.starting_yaw
            angle_end = self.starting_yaw + self.current_rotation_count
        else:
            angle_start = self.starting_yaw
            angle_end = self.starting_yaw - self.current_rotation_count
        
        for i in range(num_segments + 1):
            t = i / num_segments
            angle = angle_start + t * (angle_end - angle_start)
            
            point = Point()
            point.x = robot_x + arc_radius * math.cos(angle)
            point.y = robot_y + arc_radius * math.sin(angle)
            point.z = 0.2
            marker.points.append(point)
        
        marker.scale.x = 0.05  # Line width
        
        # Color gradient: green -> yellow -> red as approaching limit
        if rotation_fraction < 0.5:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif rotation_fraction < 0.8:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        self.rotation_progress_pub.publish(marker)
    
    def publish_forward_path(self):
        """Publish line showing predicted forward trajectory"""
        if self.current_pose is None:
            return
        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "forward_path"
        marker.id = 3
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Predict forward movement for 2 seconds
        prediction_distance = self.linear_vel * self.forward_duration
        num_points = 10
        
        for i in range(num_points + 1):
            t = i / num_points
            dist = t * prediction_distance
            
            point = Point()
            point.x = robot_x + dist * math.cos(current_yaw)
            point.y = robot_y + dist * math.sin(current_yaw)
            point.z = 0.15
            marker.points.append(point)
        
        marker.scale.x = 0.04  # Line width
        
        # Color: cyan during rotation, green during forward movement
        if self.state == PlannerState.MOVING_FORWARD:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        marker.color.a = 0.7
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        self.forward_path_pub.publish(marker)
    
    def publish_starting_position(self):
        """Publish marker at starting position when planner was triggered"""
        if self.starting_position is None:
            return
        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "starting_position"
        marker.id = 4
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.starting_position[0]
        marker.pose.position.y = self.starting_position[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.6  # Diameter
        marker.scale.y = 0.6
        marker.scale.z = 0.05  # Height (thin disk)
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.4
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        self.starting_position_pub.publish(marker)
    
    def publish_goal_bias(self):
        """Publish arrow from robot toward goal showing goal bias"""
        if self.current_pose is None or self.goal_pose is None:
            return
        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_bias"
        marker.id = 5
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.01:
            return
        
        goal_angle = math.atan2(dy, dx)
        
        marker.pose.position.x = robot_x
        marker.pose.position.y = robot_y
        marker.pose.position.z = 0.25
        
        marker.pose.orientation.z = math.sin(goal_angle / 2.0)
        marker.pose.orientation.w = math.cos(goal_angle / 2.0)
        
        # Arrow length proportional to distance (max 2m)
        arrow_length = min(distance * 0.5, 2.0)
        marker.scale.x = arrow_length
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # Magenta
        marker.color.a = 0.5
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        self.goal_bias_pub.publish(marker)
    
    def _create_color(self, r, g, b, a):
        """Helper to create ColorRGBA for marker colors"""
        from std_msgs.msg import ColorRGBA
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a
        return color


def main(args=None):
    rclpy.init(args=args)
    node = IncrementalLocalPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
