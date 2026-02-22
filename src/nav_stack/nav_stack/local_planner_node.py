#!/usr/bin/env python3
"""
Local Planner Node - Tangential Obstacle Avoidance
Activates when global planner encounters obstacles.
Uses tangential approach to navigate around obstacles.
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
    ROTATING_360 = 1
    ANALYZING = 2
    MOVING_TANGENT = 3
    STRAIGHT_CLEAR = 4
    CHECKING_GOAL = 5


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner_node')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('linear_velocity', 0.3)  # Slower for obstacle avoidance
        self.declare_parameter('angular_velocity', 0.3)
        self.declare_parameter('rotation_speed', 0.2)  # For 360 scan
        self.declare_parameter('costmap_obstacle_threshold', 50)
        self.declare_parameter('ray_cast_resolution', 10.0)  # degrees
        self.declare_parameter('min_free_distance', 1.0)  # minimum free space to consider
        self.declare_parameter('straight_clear_duration', 5.0)  # seconds
        self.declare_parameter('obstacle_check_distance', 2.0)
        
        self.robot_radius = self.get_parameter('robot_radius').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.costmap_threshold = self.get_parameter('costmap_obstacle_threshold').value
        self.ray_resolution = self.get_parameter('ray_cast_resolution').value
        self.min_free_distance = self.get_parameter('min_free_distance').value
        self.clear_duration = self.get_parameter('straight_clear_duration').value
        self.obstacle_check_distance = self.get_parameter('obstacle_check_distance').value
        
        # State
        self.state = PlannerState.IDLE
        self.current_pose = None
        self.goal_pose = None
        self.costmap = None
        self.global_map = None
        self.safe_point = None
        
        # 360 rotation tracking
        self.rotation_start_yaw = None
        self.rotation_total = 0.0
        self.rotation_last_yaw = 0.0
        self.rotation_complete = False
        self.scan_completed_this_waypoint = False  # Only one 360° scan per waypoint
        
        # Tangent movement tracking
        self.chosen_direction = None
        self.clear_timer_start = None
        self.last_turn_direction = None  # Remember last goal turn direction
        
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
            '/local_planner_trigger',
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
            '/odometry/filtered',
            self.odom_callback,
            qos_best_effort
        )
        
        self.create_subscription(
            OccupancyGrid,
            '/costmap',
            self.costmap_callback,
            qos_reliable
        )
        
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.planner_state_pub = self.create_publisher(String, '/planner_state', 10)
        
        # Timer for state machine
        self.create_timer(0.1, self.state_machine_loop)
        
        self.get_logger().info('Local Planner Node initialized - waiting for trigger')
    
    def trigger_callback(self, msg: Bool):
        """Triggered by global planner when obstacle detected"""
        if msg.data and self.state == PlannerState.IDLE:
            self.get_logger().info('🔧 Local planner activated')
            self.safe_point = (self.current_pose.position.x, self.current_pose.position.y)
            
            # Check if we already did 360 scan for this waypoint
            if self.scan_completed_this_waypoint and self.last_turn_direction is not None:
                # Use opposite direction of last turn
                self.get_logger().info(f'♻️ Reusing scan - using opposite of last turn ({self.last_turn_direction})')
                if self.last_turn_direction == "left":
                    self.chosen_direction = "right"
                else:
                    self.chosen_direction = "left"
                self.state = PlannerState.MOVING_TANGENT
            else:
                # First obstacle - do 360 scan
                self.get_logger().info('🔄 First obstacle - performing 360° scan')
                self.state = PlannerState.ROTATING_360
                self.rotation_start_yaw = None
                self.rotation_total = 0.0
                self.rotation_last_yaw = 0.0
                self.rotation_complete = False
    
    def goal_callback(self, msg: PoseStamped):
        """Update goal pose - reset scan flag for new waypoint"""
        self.goal_pose = msg
        self.scan_completed_this_waypoint = False  # New waypoint, allow 360 scan again
        self.last_turn_direction = None  # Clear turn memory for new waypoint
    
    def odom_callback(self, msg: Odometry):
        """Update current pose"""
        self.current_pose = msg.pose.pose
    
    def costmap_callback(self, msg: OccupancyGrid):
        """Update local costmap"""
        self.costmap = msg
    
    def map_callback(self, msg: OccupancyGrid):
        """Update global map"""
        self.global_map = msg
    
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
    
    def state_machine_loop(self):
        """Main state machine"""
        if self.current_pose is None or self.goal_pose is None:
            return
        
        if self.state == PlannerState.IDLE:
            pass
        
        elif self.state == PlannerState.ROTATING_360:
            self.execute_360_rotation()
        
        elif self.state == PlannerState.ANALYZING:
            self.execute_analysis()
        
        elif self.state == PlannerState.MOVING_TANGENT:
            self.execute_tangent_movement()
        
        elif self.state == PlannerState.STRAIGHT_CLEAR:
            self.execute_straight_clear()
        
        elif self.state == PlannerState.CHECKING_GOAL:
            self.execute_goal_check()
    
    def execute_360_rotation(self):
        """Perform 360 degree rotation for map gathering"""
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        if self.rotation_start_yaw is None:
            self.rotation_start_yaw = current_yaw
            self.rotation_total = 0.0
            self.rotation_last_yaw = current_yaw
            self.get_logger().info('🔄 Starting 360° rotation for mapping')
            
        # Track cumulative rotation to handle -π/π wrapping
        yaw_diff = self.normalize_angle(current_yaw - self.rotation_last_yaw)
        self.rotation_total += abs(yaw_diff)
        self.rotation_last_yaw = current_yaw
        
        # Check if completed full rotation (2π radians = 360°)
        if self.rotation_total >= 2.0 * math.pi - 0.1:
            self.stop_robot()
            self.get_logger().info('✅ 360° rotation complete')
            self.rotation_start_yaw = None  # Reset for next time
            self.state = PlannerState.ANALYZING
            return
        
        # Continue rotating
        cmd = Twist()
        cmd.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'🔄 Rotating: {math.degrees(self.rotation_total):.1f}°/360°', throttle_duration_sec=1.0)
    
    def execute_analysis(self):
        """Analyze map and find best free direction"""
        if self.global_map is None:
            self.get_logger().warn('No global map available')
            self.state = PlannerState.IDLE
            return
        
        self.get_logger().info('🔍 Analyzing left vs right free space...')
        
        # Simple: just determine if left or right has more free space
        turn_direction = self.find_better_side()  # Returns "left" or "right"
        
        if turn_direction is None:
            self.get_logger().error('❌ No free space found - returning to idle')
            self.signal_global_planner()
            return
        
        self.chosen_direction = turn_direction  # Store "left" or "right"
        self.scan_completed_this_waypoint = True  # Mark scan as done for this waypoint
        self.get_logger().info(f'✅ Turning {turn_direction} (scan complete for this waypoint)')
        self.state = PlannerState.MOVING_TANGENT
    
    def find_better_side(self):
        """Simple: determine if left or right has more free space"""
        if self.global_map is None:
            return None
        
        resolution = self.global_map.info.resolution
        width = self.global_map.info.width
        height = self.global_map.info.height
        origin_x = self.global_map.info.origin.position.x
        origin_y = self.global_map.info.origin.position.y
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # Cast rays to the left (90 degrees) and right (-90 degrees)
        left_angle = current_yaw + math.pi / 2  # 90 degrees left
        right_angle = current_yaw - math.pi / 2  # 90 degrees right
        
        left_distance = 0.0
        right_distance = 0.0
        max_distance = 5.0
        
        # Measure left side
        for dist in np.arange(0.5, max_distance, resolution):
            ray_x = robot_x + dist * math.cos(left_angle)
            ray_y = robot_y + dist * math.sin(left_angle)
            
            grid_x = int((ray_x - origin_x) / resolution)
            grid_y = int((ray_y - origin_y) / resolution)
            
            if 0 <= grid_x < width and 0 <= grid_y < height:
                index = grid_y * width + grid_x
                if index < len(self.global_map.data):
                    cost = self.global_map.data[index]
                    if cost > 10:  # Hit obstacle
                        break
                    left_distance = dist
            else:
                break
        
        # Measure right side
        for dist in np.arange(0.5, max_distance, resolution):
            ray_x = robot_x + dist * math.cos(right_angle)
            ray_y = robot_y + dist * math.sin(right_angle)
            
            grid_x = int((ray_x - origin_x) / resolution)
            grid_y = int((ray_y - origin_y) / resolution)
            
            if 0 <= grid_x < width and 0 <= grid_y < height:
                index = grid_y * width + grid_x
                if index < len(self.global_map.data):
                    cost = self.global_map.data[index]
                    if cost > 50:  # Hit obstacle
                        break
                    right_distance = dist
            else:
                break
        
        self.get_logger().info(f'Left: {left_distance:.2f}m, Right: {right_distance:.2f}m')
        
        # Calculate which side is closer to goal direction
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        goal_dx = goal_x - robot_x
        goal_dy = goal_y - robot_y
        goal_angle = math.atan2(goal_dy, goal_dx)
        
        # Calculate angle difference for left and right
        left_to_goal = abs(self.normalize_angle(left_angle - goal_angle))
        right_to_goal = abs(self.normalize_angle(right_angle - goal_angle))
        
        # Score = free_distance * goal_bias
        # Give 50% bonus to the side that's closer to goal direction
        left_score = left_distance * (1.0 if left_to_goal < right_to_goal else 0.7)
        right_score = right_distance * (1.0 if right_to_goal < left_to_goal else 0.7)
        
        self.get_logger().info(f'Scores - Left: {left_score:.2f}, Right: {right_score:.2f}')
        
        # Return the side with better score (space + goal bias)
        if left_score > right_score:
            self.get_logger().info('→ Choosing LEFT')
            return "left"
        elif right_score > left_score:
            self.get_logger().info('→ Choosing RIGHT')
            return "right"
        else:
            # Equal scores - prefer the one closer to goal
            if left_to_goal < right_to_goal:
                self.get_logger().info('→ Equal, choosing LEFT (closer to goal)')
                return "left"
            else:
                self.get_logger().info('→ Equal, choosing RIGHT (closer to goal)')
                return "right"
    
    def execute_tangent_movement(self):
        """Rotate left/right and watch costmap for clear path"""
        if self.chosen_direction is None:
            self.state = PlannerState.ANALYZING
            return
        
        # Check local costmap - is path ahead clear?
        if self.check_obstacle_ahead():
            # Obstacle ahead - keep rotating in chosen direction
            cmd = Twist()
            if self.chosen_direction == "left":
                cmd.angular.z = self.angular_vel
            else:  # "right"
                cmd.angular.z = -self.angular_vel
            
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(f'🔄 Rotating {self.chosen_direction} - obstacle ahead', throttle_duration_sec=1.0)
        else:
            # Path is clear! Stop rotating and start 5s forward test
            self.stop_robot()
            self.get_logger().info('✅ Path clear - starting 5s straight test')
            self.state = PlannerState.STRAIGHT_CLEAR
            self.clear_timer_start = None  # Reset timer for fresh start
    
    def execute_straight_clear(self):
        """Move straight for 5 seconds to confirm path is clear"""
        # Initialize timer on first call
        if self.clear_timer_start is None:
            self.clear_timer_start = self.get_clock().now()
            # Don't move yet - wait one cycle to verify path is still clear
            self.get_logger().info('⏸️ Verifying path before moving...')
            return
        
        elapsed = (self.get_clock().now() - self.clear_timer_start).nanoseconds / 1e9
        
        # Check for obstacles while moving
        if self.check_obstacle_ahead():
            self.get_logger().warn('⚠️ Obstacle detected during clear test - aborting')
            self.stop_robot()
            self.state = PlannerState.IDLE
            self.signal_global_planner()
            self.clear_timer_start = None
            return
        
        if elapsed >= self.clear_duration:
            self.get_logger().info('✅ 5s clear - checking goal path')
            self.chosen_direction = None  # Clear the direction after 5s forward
            self.state = PlannerState.CHECKING_GOAL
            self.clear_timer_start = None
            return
        
        # Continue moving straight
        cmd = Twist()
        cmd.linear.x = self.linear_vel
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'➡️ Clear test: {elapsed:.1f}s / {self.clear_duration:.1f}s', throttle_duration_sec=1.0)
    
    def execute_goal_check(self):
        """Check if direct path to goal is now clear"""
        # First, turn to face the goal and remember which direction we turned
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        
        goal_dx = goal_x - robot_x
        goal_dy = goal_y - robot_y
        goal_angle = math.atan2(goal_dy, goal_dx)
        
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        angle_diff = self.normalize_angle(goal_angle - current_yaw)
        
        # Remember which direction we're turning to face goal
        if angle_diff > 0:
            self.last_turn_direction = "left"
        else:
            self.last_turn_direction = "right"
        
        # Check if path to goal is clear
        if self.check_direct_path_to_goal_clear():
            self.get_logger().info(f'✅ Direct path to goal clear (turned {self.last_turn_direction} to face goal)')
            self.stop_robot()
            self.signal_global_planner()
        else:
            self.get_logger().info(f'⚠️ Goal blocked (turned {self.last_turn_direction}) - will use opposite next time')
            # Don't trigger immediately - let global planner handle it
            self.stop_robot()
            self.signal_global_planner()
    
    def check_obstacle_ahead(self):
        """Check if obstacle in robot boundary ahead in costmap (swept circle check)"""
        if self.costmap is None:
            return False
        
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        height = self.costmap.info.height
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # Check robot boundary (swept circle) forward up to 4m
        check_distance = 4.0
        num_samples = int(check_distance / resolution) + 1
        
        for i in range(num_samples):
            dist = i * resolution
            line_x = robot_x + dist * math.cos(current_yaw)
            line_y = robot_y + dist * math.sin(current_yaw)
            
            # Check circle around this point (robot boundary)
            num_circle = 12  # Points around robot boundary
            for angle_idx in range(num_circle):
                angle = 2.0 * math.pi * angle_idx / num_circle
                check_x = line_x + self.robot_radius * math.cos(angle)
                check_y = line_y + self.robot_radius * math.sin(angle)
                
                grid_x = int((check_x - origin_x) / resolution)
                grid_y = int((check_y - origin_y) / resolution)
                
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    index = grid_y * width + grid_x
                    if index < len(self.costmap.data):
                        if self.costmap.data[index] > self.costmap_threshold:
                            return True
        
        return False
    
    def check_obstacle_in_direction(self, direction):
        """Check if obstacle in specific direction"""
        if self.costmap is None:
            return False
        
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        height = self.costmap.info.height
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Check along direction
        for dist in np.arange(0.3, 1.5, resolution):
            check_x = robot_x + dist * math.cos(direction)
            check_y = robot_y + dist * math.sin(direction)
            
            grid_x = int((check_x - origin_x) / resolution)
            grid_y = int((check_y - origin_y) / resolution)
            
            if 0 <= grid_x < width and 0 <= grid_y < height:
                index = grid_y * width + grid_x
                if index < len(self.costmap.data):
                    if self.costmap.data[index] > self.costmap_threshold:
                        return True
        
        return False
    
    def check_direct_path_to_goal_clear(self):
        """Check if bee-line to goal is clear (same as global planner)"""
        if self.costmap is None or self.goal_pose is None:
            return False
        
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        height = self.costmap.info.height
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.01:
            return True
        
        dx_norm = dx / distance
        dy_norm = dy / distance
        
        check_distance = min(distance, self.obstacle_check_distance)
        num_samples = int(check_distance / resolution) + 1
        
        for i in range(num_samples):
            t = i * resolution
            line_x = robot_x + dx_norm * t
            line_y = robot_y + dy_norm * t
            
            # Check circle around line point
            num_circle = 16
            for angle_idx in range(num_circle):
                angle = 2.0 * math.pi * angle_idx / num_circle
                check_x = line_x + self.robot_radius * math.cos(angle)
                check_y = line_y + self.robot_radius * math.sin(angle)
                
                grid_x = int((check_x - origin_x) / resolution)
                grid_y = int((check_y - origin_y) / resolution)
                
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    index = grid_y * width + grid_x
                    if index < len(self.costmap.data):
                        if self.costmap.data[index] > self.costmap_threshold:
                            return False
        
        return True
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def signal_global_planner(self):
        """Signal global planner to resume"""
        self.state = PlannerState.IDLE
        msg = String()
        msg.data = "global_active"
        self.planner_state_pub.publish(msg)
        self.get_logger().info('✅ Signaling global planner to resume')


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
