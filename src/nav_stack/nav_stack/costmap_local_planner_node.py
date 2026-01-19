#!/usr/bin/env python3
"""
Costmap-Only Local Planner Node
Uses only /costmap for obstacle avoidance - no /map dependency.
Analyzes left/right free space and performs tangential movement.
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
    MOVING_TANGENT = 2
    STRAIGHT_CLEAR = 3
    CHECKING_GOAL = 4


class CostmapLocalPlannerNode(Node):
    def __init__(self):
        super().__init__('costmap_local_planner_node')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.6)
        self.declare_parameter('linear_velocity', 1.2)
        self.declare_parameter('angular_velocity', 4.0)
        self.declare_parameter('costmap_obstacle_threshold', 70)
        self.declare_parameter('min_free_distance', 1.0)
        self.declare_parameter('straight_clear_duration', 5.0)
        self.declare_parameter('obstacle_check_distance', 5.0)
        self.declare_parameter('tangent_scan_distance', 5.0)
        
        self.robot_radius = self.get_parameter('robot_radius').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.costmap_threshold = self.get_parameter('costmap_obstacle_threshold').value
        self.min_free_distance = self.get_parameter('min_free_distance').value
        self.clear_duration = self.get_parameter('straight_clear_duration').value
        self.obstacle_check_distance = self.get_parameter('obstacle_check_distance').value
        self.tangent_scan_distance = self.get_parameter('tangent_scan_distance').value
        
        # State
        self.state = PlannerState.IDLE
        self.current_pose = None
        self.goal_pose = None
        self.costmap = None
        self.safe_point = None
        
        # Tangent movement tracking
        self.chosen_direction = None  # "left" or "right"
        self.forward_timer_start = None  # Timer for 5-second forward movement
        
        # Instruction printing
        self.last_instruction_time = None
        self.instruction_interval = 0.5  # seconds
        
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
        
        self.get_logger().info('Costmap-Only Local Planner initialized')
        self.get_logger().info('No /map dependency - using /costmap only')
    
    def trigger_callback(self, msg: Bool):
        """Triggered by global planner when obstacle detected"""
        if msg.data and self.state == PlannerState.IDLE:
            self.get_logger().info('Local planner activated - analyzing costmap')
            self.safe_point = (self.current_pose.position.x, self.current_pose.position.y)
            self.state = PlannerState.ANALYZING
            self.last_instruction_time = None  # Reset to print immediately
    
    def goal_callback(self, msg: PoseStamped):
        """Update goal pose"""
        self.goal_pose = msg
    
    def odom_callback(self, msg: Odometry):
        """Update current pose"""
        self.current_pose = msg.pose.pose
    
    def costmap_callback(self, msg: OccupancyGrid):
        """Update local costmap - ONLY source of obstacle info"""
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
        
        # Check if enough time has passed since last instruction
        current_time = self.get_clock().now()
        if self.last_instruction_time is not None:
            elapsed = (current_time - self.last_instruction_time).nanoseconds / 1e9
            if elapsed < 0.4:
                return
        
        distance = self.get_distance_to_goal()
        
        if self.state == PlannerState.ANALYZING:
            self.get_logger().info(f'Analyzing costmap for best direction...')
            self.last_instruction_time = current_time
        
        elif self.state == PlannerState.MOVING_TANGENT:
            direction = "LEFT" if self.chosen_direction == "left" else "RIGHT"
            self.get_logger().info(f'Rotating {direction} (checking width & depth...)')
            self.last_instruction_time = current_time
        
        elif self.state == PlannerState.STRAIGHT_CLEAR:
            elapsed = (self.get_clock().now() - self.forward_timer_start).nanoseconds / 1e9 if self.forward_timer_start else 0
            self.get_logger().info(f'Moving forward ({elapsed:.1f}s/5.0s)')
            self.last_instruction_time = current_time
        
        elif self.state == PlannerState.CHECKING_GOAL:
            self.get_logger().info(f'Approaching goal - {distance:.2f}m remaining')
            self.last_instruction_time = current_time
    
    def state_machine_loop(self):
        """Main state machine"""
        if self.current_pose is None or self.goal_pose is None:
            return
        
        if self.state == PlannerState.IDLE:
            pass
        
        elif self.state == PlannerState.ANALYZING:
            self.execute_analysis()
        
        elif self.state == PlannerState.MOVING_TANGENT:
            self.execute_tangent_movement()
        
        elif self.state == PlannerState.STRAIGHT_CLEAR:
            self.execute_straight_clear()
        
        elif self.state == PlannerState.CHECKING_GOAL:
            self.execute_goal_check()
    
    def execute_analysis(self):
        """Stop robot and analyze costmap to find best free direction (left or right)"""
        if self.costmap is None:
            self.get_logger().warn('No costmap available')
            self.state = PlannerState.IDLE
            return
        
        # STOP robot completely
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info('STOPPED - Analyzing left vs right in costmap...')
        
        # Determine which side has more free space
        turn_direction = self.find_better_side_in_costmap()
        
        if turn_direction is None:
            self.get_logger().error('No free space found - returning to idle')
            self.signal_global_planner()
            return
        
        self.chosen_direction = turn_direction
        self.get_logger().info(f'Choosing {turn_direction} direction')
        self.state = PlannerState.MOVING_TANGENT
    
    def find_better_side_in_costmap(self):
        """
        Analyze left and right sides using ONLY costmap data.
        Simple ray-casting perpendicular to robot orientation.
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
        
        # Cast rays perpendicular to current heading
        left_angle = current_yaw + math.pi / 2  # 90 degrees left
        right_angle = current_yaw - math.pi / 2  # 90 degrees right
        
        left_distance = 0.0
        right_distance = 0.0
        max_distance = self.tangent_scan_distance
        
        # Measure left side
        for dist in np.arange(0.5, max_distance, resolution):
            ray_x = robot_x + dist * math.cos(left_angle)
            ray_y = robot_y + dist * math.sin(left_angle)
            
            grid_x = int((ray_x - origin_x) / resolution)
            grid_y = int((ray_y - origin_y) / resolution)
            
            if 0 <= grid_x < width and 0 <= grid_y < height:
                index = grid_y * width + grid_x
                if index < len(self.costmap.data):
                    cost = self.costmap.data[index]
                    if cost > self.costmap_threshold:
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
                if index < len(self.costmap.data):
                    cost = self.costmap.data[index]
                    if cost > self.costmap_threshold:
                        break
                    right_distance = dist
            else:
                break
        
        self.get_logger().info(f'Costmap analysis - Left: {left_distance:.2f}m, Right: {right_distance:.2f}m')
        
        # Calculate which side is closer to goal direction
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        goal_dx = goal_x - robot_x
        goal_dy = goal_y - robot_y
        goal_angle = math.atan2(goal_dy, goal_dx)
        
        # Angle difference for left and right
        left_to_goal = abs(self.normalize_angle(left_angle - goal_angle))
        right_to_goal = abs(self.normalize_angle(right_angle - goal_angle))
        
        # Score = free_distance * goal_bias
        left_score = left_distance * (1.0 if left_to_goal < right_to_goal else 0.7)
        right_score = right_distance * (1.0 if right_to_goal < left_to_goal else 0.7)
        
        self.get_logger().info(f'Scores - Left: {left_score:.2f}, Right: {right_score:.2f}')
        
        # Choose side with better score
        if left_score > right_score and left_distance > self.min_free_distance:
            self.get_logger().info('→ Choosing LEFT')
            return "left"
        elif right_score > left_score and right_distance > self.min_free_distance:
            self.get_logger().info('→ Choosing RIGHT')
            return "right"
        elif left_score > 0.1 or right_score > 0.1:
            # If scores are close, prefer side closer to goal
            if left_to_goal < right_to_goal:
                return "left"
            else:
                return "right"
        else:
            return None
    
    def execute_tangent_movement(self):
        """
        Rotate in chosen direction while checking:
        - Width: robot_radius wide (circular footprint)
        - Depth: obstacle_check_distance meters ahead
        When both satisfied, move forward.
        """
        if self.costmap is None:
            return
        
        # Check if path ahead satisfies BOTH conditions:
        # 1. Wide enough (robot_radius)
        # 2. Deep enough (obstacle_check_distance)
        is_clear = self.check_path_width_and_depth()
        
        if is_clear:
            # Conditions satisfied - start moving forward
            self.get_logger().info('✓ Path clear (wide & deep enough) - moving forward for 5s')
            self.forward_timer_start = self.get_clock().now()
            self.state = PlannerState.STRAIGHT_CLEAR
            return
        
        # Continue rotating (NO forward movement)
        cmd = Twist()
        cmd.linear.x = 0.0
        
        if self.chosen_direction == "left":
            cmd.angular.z = self.angular_vel
        else:  # right
            cmd.angular.z = -self.angular_vel
        
        self.cmd_vel_pub.publish(cmd)
    
    def check_path_width_and_depth(self):
        """
        Check if path ahead satisfies BOTH:
        1. Width: robot_radius wide (circular footprint clear)
        2. Depth: obstacle_check_distance meters ahead
        Returns True only if BOTH conditions met.
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
        
        # Direction vector (current heading)
        dx_norm = math.cos(current_yaw)
        dy_norm = math.sin(current_yaw)
        
        # Check path up to obstacle_check_distance
        check_distance = self.obstacle_check_distance
        num_samples = int(check_distance / resolution) + 1
        
        for i in range(num_samples):
            t = i * resolution
            if t > check_distance:
                break
            
            check_x = robot_x + dx_norm * t
            check_y = robot_y + dy_norm * t
            
            # Check circular footprint (robot_radius) around each point
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
                            # Obstacle found - not clear
                            return False
                else:
                    # Outside costmap bounds - not clear
                    return False
        
        # Full check_distance is clear with robot_radius width
        return True
    
    def check_path_clear_in_costmap(self):
        """Check if direct path to goal is clear using ONLY costmap"""
        if self.costmap is None or self.current_pose is None or self.goal_pose is None:
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
        
        # Check up to obstacle_check_distance or goal distance
        check_distance = min(distance, self.obstacle_check_distance)
        num_samples = int(check_distance / resolution) + 1
        
        for i in range(num_samples):
            t = i * resolution
            check_x = robot_x + dx_norm * t
            check_y = robot_y + dy_norm * t
            
            # Check robot radius around point
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
                    # Outside costmap bounds - consider as obstacle
                    return False
        
        return True
    
    def execute_straight_clear(self):
        """
        Move forward for 5 seconds while continuously checking for obstacles.
        If obstacle found, recalculate safe direction.
        If 5 seconds complete, return to global planner.
        """
        # Check for obstacles in path
        if not self.check_path_width_and_depth():
            self.get_logger().warn('⚠ Obstacle detected during forward movement - recalculating direction')
            self.state = PlannerState.ANALYZING
            self.forward_timer_start = None
            return
        
        # Check if 5 seconds elapsed
        elapsed = (self.get_clock().now() - self.forward_timer_start).nanoseconds / 1e9
        
        if elapsed >= 5.0:
            self.get_logger().info('✓ 5 seconds completed - returning to global planner')
            self.signal_global_planner()
            return
        
        # Move forward
        cmd = Twist()
        cmd.linear.x = self.linear_vel
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def execute_goal_check(self):
        """Final approach to goal"""
        distance = self.get_distance_to_goal()
        
        if distance < 0.5:
            self.get_logger().info('Near goal - returning to global planner')
            self.signal_global_planner()
            return
        
        # Continue moving toward goal
        angle_to_goal = self.get_angle_to_goal()
        
        cmd = Twist()
        cmd.linear.x = self.linear_vel
        cmd.angular.z = 0.5 * angle_to_goal
        self.cmd_vel_pub.publish(cmd)
    
    def get_distance_to_goal(self):
        """Calculate distance to goal"""
        if self.current_pose is None or self.goal_pose is None:
            return float('inf')
        
        dx = self.goal_pose.pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def get_angle_to_goal(self):
        """Calculate angle to goal"""
        if self.current_pose is None or self.goal_pose is None:
            return 0.0
        
        dx = self.goal_pose.pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.position.y
        
        goal_angle = math.atan2(dy, dx)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        return self.normalize_angle(goal_angle - current_yaw)
    
    def signal_global_planner(self):
        """Return control to global planner"""
        self.get_logger().info('Signaling global planner to resume')
        
        # Stop robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # Reset state
        self.state = PlannerState.IDLE
        self.chosen_direction = None
        self.forward_timer_start = None
        
        # Publish state
        state_msg = String()
        state_msg.data = "global_active"
        self.planner_state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapLocalPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
