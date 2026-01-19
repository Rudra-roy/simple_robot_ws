#!/usr/bin/env python3
"""
Incremental Local Planner Node
Uses 10-degree rotation increments with costmap-based obstacle checking.
Analyzes -120° to +120° range for safe direction with goal bias.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
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
        self.declare_parameter('angular_velocity', 1.0)  # Slower for precise rotation steps
        self.declare_parameter('costmap_obstacle_threshold', 70)
        self.declare_parameter('forward_duration', 2.0)
        self.declare_parameter('obstacle_check_distance', 5.0)
        self.declare_parameter('rotation_increment', 20.0)  # degrees
        self.declare_parameter('max_rotation_angle', 120.0)  # degrees
        self.declare_parameter('direction_scan_range', 120.0)  # -120 to +120 degrees
        
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
        
        # Timer for state machine
        self.create_timer(0.1, self.state_machine_loop)
        self.create_timer(0.5, self.print_movement_instruction)
        
        self.get_logger().info('Incremental Local Planner initialized')
        self.get_logger().info('20° rotation increments, 2s forward, 120° limit')
    
    def trigger_callback(self, msg: Bool):
        """Triggered by global planner when obstacle detected"""
        if msg.data and self.state == PlannerState.IDLE:
            self.get_logger().info('Local planner activated - analyzing direction')
            self.starting_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
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
            self.get_logger().info(f'Moving forward ({elapsed:.1f}s/2.0s)')
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
        Analyze -120° to +120° range from current heading.
        Choose direction with goal bias and free space.
        """
        if self.costmap is None:
            self.get_logger().warn('No costmap available')
            self.state = PlannerState.IDLE
            return
        
        # Stop robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info('STOPPED - Scanning -120° to +120° range')
        
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
        Scan from -120° to +120° relative to current heading.
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
        
        # Scan from -120° to +120°
        angle_samples = np.linspace(-self.scan_range, self.scan_range, 25)
        
        left_score = 0.0
        right_score = 0.0
        
        for relative_angle in angle_samples:
            scan_angle = current_yaw + relative_angle
            
            # Measure free distance in this direction
            free_distance = self.measure_free_distance(scan_angle, robot_x, robot_y, 
                                                      origin_x, origin_y, resolution, width, height)
            
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
        
        # Calculate expected time for 20° rotation with large safety margin
        # At 3.0 rad/s, 20° = 0.349 rad takes 0.116s, but need buffer for acceleration
        expected_time = (self.rotation_increment / self.angular_vel) * 3.0  # 3x safety margin
        elapsed_time = (self.get_clock().now() - self.rotation_start_time).nanoseconds / 1e9
        
        # Primary check: angle achieved (with some tolerance for overshoot)
        # Secondary check: timeout protection (3x expected time)
        angle_achieved = rotated_angle >= self.rotation_increment * 0.85  # 85% of target
        timeout = elapsed_time > expected_time
        
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
                self.get_logger().info('✓ Path clear - moving forward for 2s')
                self.forward_timer_start = self.get_clock().now()
                self.state = PlannerState.MOVING_FORWARD
                return
            
            # Path not clear - check rotation limit
            self.current_rotation_count += self.rotation_increment
            
            if self.current_rotation_count >= self.max_rotation_angle:
                self.get_logger().warn(f'⚠ 120° limit reached on {self.chosen_direction} side')
                
                if not self.has_tried_opposite:
                    self.get_logger().info('Returning to start and trying opposite direction')
                    self.state = PlannerState.RETURNING_TO_START
                else:
                    self.get_logger().error('Both directions exhausted - no path found')
                    self.signal_global_planner()
                return
            
            # Continue rotating - set next target
            self.get_logger().info('⚠ Obstacle found - rotating another 20°')
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
        Move forward for 2 seconds.
        Check for obstacles during movement.
        When complete, return to global planner.
        """
        # Check for obstacles
        if not self.check_path_clear():
            self.get_logger().warn('⚠ Obstacle detected during forward movement')
            # Reset rotation count and try again from current position
            self.starting_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            self.current_rotation_count = 0.0
            self.state = PlannerState.ANALYZING
            return
        
        # Check if 2 seconds elapsed
        elapsed = (self.get_clock().now() - self.forward_timer_start).nanoseconds / 1e9
        
        if elapsed >= self.forward_duration:
            self.get_logger().info('✓ 2 seconds completed - returning to global planner')
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
            
            self.get_logger().info('✓ Returned to starting orientation')
            
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
        self.current_rotation_count = 0.0
        self.target_yaw = None
        self.forward_timer_start = None
        self.has_tried_opposite = False
        self.rotation_start_time = None
        self.rotation_start_yaw = None
        
        # Publish state
        state_msg = String()
        state_msg.data = "global_active"
        self.planner_state_pub.publish(state_msg)


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
