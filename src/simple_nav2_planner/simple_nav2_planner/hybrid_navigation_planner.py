#!/usr/bin/env python3
"""
Hybrid Navigation Planner
Combines global straight-line planning with local A* obstacle avoidance
Uses 360° survey to gather obstacle information before planning
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_msgs.msg import String
import math
import numpy as np
from enum import Enum
from collections import deque
import heapq


class PlannerState(Enum):
    """State machine states for the hybrid planner"""
    IDLE = 0
    GLOBAL_PLANNER = 1      # Straight-line to goal
    SURVEYING = 2           # 360° rotation to gather info
    LOCAL_AVOIDANCE = 3     # A* path following


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


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class HybridNavigationPlanner(Node):
    """
    Main hybrid navigation planner node
    Switches between global straight-line and local A* avoidance
    """
    
    def __init__(self):
        super().__init__('hybrid_navigation_planner')
        
        # ============ Parameters ============
        self.declare_parameter('obstacle_detection_radius', 4.0)  # meters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('survey_angular_speed', 0.3)  # rad/s for 360° rotation
        self.declare_parameter('clearance_margin', 2.0)  # meters beyond obstacle edge
        self.declare_parameter('obstacle_inflation_radius', 0.8)  # meters for A* safety (increased from 0.5)
        
        self.obstacle_radius = self.get_parameter('obstacle_detection_radius').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.survey_angular_speed = self.get_parameter('survey_angular_speed').value
        self.clearance_margin = self.get_parameter('clearance_margin').value
        self.obstacle_inflation_radius = self.get_parameter('obstacle_inflation_radius').value
        
        # ============ State Management ============
        self.state = PlannerState.IDLE
        self.previous_state = PlannerState.IDLE
        
        # ============ Robot State ============
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_pose = None
        
        # ============ Goal State ============
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        
        # ============ Costmap & Map Data ============
        self.costmap = None
        self.costmap_info = None
        self.global_map = None
        self.global_map_info = None
        
        # ============ Survey Data ============
        self.survey_start_yaw = None
        self.survey_total_rotation = 0.0  # Accumulated rotation during survey
        self.survey_last_yaw = None  # Last yaw reading for incremental tracking
        
        # ============ Path Planning Data ============
        self.current_path = []  # List of waypoints from A*
        self.current_waypoint_index = 0
        self.clearance_waypoint = None
        self.replan_count = 0  # Track replanning attempts
        self.max_replans = 3  # Maximum replans before giving up
        
        # ============ Publishers ============
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/planner_state', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # ============ Subscribers ============
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/costmap', self.costmap_callback, 10)
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # ============ Control Timer ============
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🚀 Hybrid Navigation Planner initialized')
        self.get_logger().info(f'   Obstacle detection radius: {self.obstacle_radius}m')
        self.get_logger().info(f'   Obstacle inflation radius: {self.obstacle_inflation_radius}m')
        self.get_logger().info(f'   Linear speed: {self.linear_speed}m/s')
        self.get_logger().info(f'   Survey angular speed: {self.survey_angular_speed}rad/s')

    # ============================================================
    # CALLBACK FUNCTIONS
    # ============================================================
    
    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry."""
        self.current_pose = msg.pose.pose
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
    
    def costmap_callback(self, msg: OccupancyGrid):
        """Receive local costmap for obstacle detection."""
        self.costmap = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.costmap_info = msg.info
    
    def map_callback(self, msg: OccupancyGrid):
        """Receive global map from rtabmap for A* planning."""
        self.global_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.global_map_info = msg.info
    
    def goal_callback(self, msg: PoseStamped):
        """Receive new goal and start navigation."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        orientation_q = msg.pose.orientation
        _, _, self.goal_yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        
        # Reset state and start navigation
        self.state = PlannerState.GLOBAL_PLANNER
        self.current_path = []
        self.current_waypoint_index = 0
        self.clearance_waypoint = None
        self.replan_count = 0  # Reset replan counter for new goal
        
        self.get_logger().info(f'🎯 New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})')
        self.get_logger().info(f'   Switching to GLOBAL_PLANNER mode')
    
    # ============================================================
    # MAIN CONTROL LOOP
    # ============================================================
    
    def control_loop(self):
        """Main control loop - executes current state behavior."""
        # Publish current state
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)
        
        # State machine execution
        if self.state == PlannerState.IDLE:
            self.execute_idle()
        
        elif self.state == PlannerState.GLOBAL_PLANNER:
            self.execute_global_planner()
        
        elif self.state == PlannerState.SURVEYING:
            self.execute_surveying()
        
        elif self.state == PlannerState.LOCAL_AVOIDANCE:
            self.execute_local_avoidance()
    
    # ============================================================
    # STATE EXECUTION FUNCTIONS (Placeholders for now)
    # ============================================================
    
    def execute_idle(self):
        """IDLE state - do nothing, wait for goal."""
        pass
    
    def execute_global_planner(self):
        """GLOBAL_PLANNER state - go straight to goal."""
        if self.current_pose is None or self.goal_x is None:
            return
        
        # CRITICAL SAFETY CHECK: Verify robot is not in collision
        if not self.is_robot_position_safe():
            self.get_logger().error('❌ COLLISION DETECTED! Robot position is occupied!')
            self.get_logger().error('   Emergency stop - possible odometry drift or actual collision')
            self.stop_robot()
            self.state = PlannerState.IDLE
            return
        
        # Safety check: ensure robot has clearance
        if not self.check_robot_clearance():
            self.get_logger().error('❌ Robot surrounded! Emergency stop.')
            self.stop_robot()
            self.state = PlannerState.IDLE
            return
        
        # Check if goal reached
        dist_to_goal = self.distance_to_goal()
        if dist_to_goal < self.goal_tolerance:
            # SAFETY CHECK: Verify robot is not in collision before declaring success
            if not self.is_robot_position_safe():
                self.get_logger().error('⚠️  Goal reached but ROBOT IS IN COLLISION!')
                self.get_logger().error('   Stopping immediately for safety')
                self.stop_robot()
                self.state = PlannerState.IDLE
                return
            
            # SAFETY CHECK: Verify robot has clearance
            if not self.check_robot_clearance():
                self.get_logger().error('⚠️  Goal reached but robot SURROUNDED by obstacles!')
                self.get_logger().error('   Stopping immediately for safety')
                self.stop_robot()
                self.state = PlannerState.IDLE
                return
            
            self.stop_robot()
            self.state = PlannerState.IDLE
            self.get_logger().info('🎉 Goal reached! Robot is safe and clear.')
            return
        
        # Check for obstacles in the DIRECT PATH to goal (not just radius)
        if self.obstacle_in_direct_path():
            # Check if this is a NEW obstacle (not in global map)
            if self.is_new_obstacle():
                self.get_logger().warn(f'⚠️  NEW obstacle blocking direct path!')
                self.get_logger().info('   Switching to SURVEYING mode')
            else:
                self.get_logger().warn(f'⚠️  Known obstacle blocking direct path!')
                self.get_logger().info('   Switching to SURVEYING mode')
            
            # Either way, do survey first before A* planning
            self.transition_to_surveying()
            return
        
        # Navigate straight to goal
        cmd_vel = self.calculate_global_velocity()
        self.cmd_vel_pub.publish(cmd_vel)
    
    def calculate_global_velocity(self):
        """Calculate velocity command for straight-line navigation to goal."""
        cmd_vel = Twist()
        
        # Calculate angle and distance to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        goal_distance = math.sqrt(dx**2 + dy**2)
        goal_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(goal_angle - self.current_yaw)
        
        # Check for obstacles in front of robot (safety brake)
        obstacle_ahead = self.detect_obstacle_ahead(distance=1.0)
        
        # If angle error is large, rotate in place first
        if abs(angle_error) > 0.3:  # ~17 degrees
            cmd_vel.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            cmd_vel.linear.x = 0.0
        else:
            # Move forward with proportional control
            base_speed = min(self.linear_speed, goal_distance * 0.5)
            
            # SAFETY: Reduce speed if obstacle detected ahead
            if obstacle_ahead:
                base_speed = min(base_speed, 0.1)  # Max 0.1 m/s when obstacle nearby
                self.get_logger().warn('⚠️  Obstacle ahead - reducing speed')
            
            cmd_vel.linear.x = base_speed
            cmd_vel.angular.z = 2.0 * angle_error  # Proportional turning
        
        return cmd_vel
    
    def detect_obstacle_in_radius(self, radius):
        """
        Detect if there are obstacles within specified radius in costmap.
        Returns True if obstacle found.
        """
        if self.costmap is None or self.costmap_info is None:
            return False
        
        resolution = self.costmap_info.resolution
        origin_x = self.costmap_info.origin.position.x
        origin_y = self.costmap_info.origin.position.y
        height, width = self.costmap.shape
        
        # Convert robot position to costmap grid coordinates
        robot_grid_x = int((self.current_x - origin_x) / resolution)
        robot_grid_y = int((self.current_y - origin_y) / resolution)
        
        # Check cells within radius
        cells_radius = int(radius / resolution)
        
        for dy in range(-cells_radius, cells_radius + 1):
            for dx in range(-cells_radius, cells_radius + 1):
                grid_x = robot_grid_x + dx
                grid_y = robot_grid_y + dy
                
                # Check bounds
                if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
                    continue
                
                # Check if within radius (circular check)
                dist_cells = math.sqrt(dx**2 + dy**2)
                if dist_cells > cells_radius:
                    continue
                
                # Check if occupied (threshold: 50)
                if self.costmap[grid_y, grid_x] > 50:
                    return True
        
        return False
    
    def detect_obstacle_ahead(self, distance=1.0):
        """
        Detect if there's an obstacle directly in front of the robot.
        Checks a cone in the robot's forward direction.
        
        Args:
            distance: How far ahead to check (meters)
            
        Returns:
            True if obstacle detected ahead, False otherwise
        """
        if self.costmap is None or self.costmap_info is None:
            return False
        
        resolution = self.costmap_info.resolution
        origin_x = self.costmap_info.origin.position.x
        origin_y = self.costmap_info.origin.position.y
        height, width = self.costmap.shape
        
        # Sample points along forward direction
        num_samples = int(distance / 0.1)  # Check every 10cm
        num_samples = max(5, min(num_samples, 20))
        
        for i in range(1, num_samples + 1):
            # Point along forward direction
            check_dist = (i / num_samples) * distance
            check_x = self.current_x + check_dist * math.cos(self.current_yaw)
            check_y = self.current_y + check_dist * math.sin(self.current_yaw)
            
            # Convert to grid
            grid_x = int((check_x - origin_x) / resolution)
            grid_y = int((check_y - origin_y) / resolution)
            
            # Check bounds
            if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
                continue
            
            # Check if occupied
            if self.costmap[grid_y, grid_x] > 50:
                return True
            
            # Also check cells on either side (cone shape)
            for offset in [-1, 1]:
                side_x = grid_x + int(offset * math.sin(self.current_yaw))
                side_y = grid_y - int(offset * math.cos(self.current_yaw))
                
                if 0 <= side_x < width and 0 <= side_y < height:
                    if self.costmap[side_y, side_x] > 50:
                        return True
        
        return False
    
    def obstacle_in_direct_path(self):
        """
        Check if there's an obstacle in the direct path to goal.
        Only checks along the line from robot to goal, not full radius.
        Returns True if path is blocked.
        """
        if self.costmap is None or self.costmap_info is None or self.goal_x is None:
            return False
        
        # Sample points along direct line to goal
        num_samples = int(self.distance_to_goal() / 0.2)  # Sample every 0.2m
        num_samples = max(5, min(num_samples, 50))  # Clamp between 5-50
        
        for i in range(1, num_samples + 1):
            t = i / num_samples
            sample_x = self.current_x + t * (self.goal_x - self.current_x)
            sample_y = self.current_y + t * (self.goal_y - self.current_y)
            
            # Check if this point is occupied in costmap
            if self.is_point_occupied(sample_x, sample_y, self.costmap, self.costmap_info, threshold=50):
                return True
        
        return False
    
    def is_new_obstacle(self):
        """
        Check if detected obstacle is NEW (not in global map).
        Compares costmap obstacles with global map.
        Returns True if obstacle is not in global map.
        """
        if self.global_map is None or self.global_map_info is None:
            # No global map, assume obstacle is new
            return True
        
        if self.costmap is None or self.costmap_info is None:
            return False
        
        # Find obstacle cells in costmap
        obstacle_cells = np.where(self.costmap > 50)
        
        if len(obstacle_cells[0]) == 0:
            return False
        
        # Sample some obstacle cells (not all, for efficiency)
        sample_indices = np.random.choice(len(obstacle_cells[0]), 
                                         size=min(20, len(obstacle_cells[0])), 
                                         replace=False)
        
        new_obstacle_count = 0
        total_samples = len(sample_indices)
        
        for idx in sample_indices:
            grid_y = obstacle_cells[0][idx]
            grid_x = obstacle_cells[1][idx]
            
            # Convert to world coordinates
            world_x = grid_x * self.costmap_info.resolution + self.costmap_info.origin.position.x
            world_y = grid_y * self.costmap_info.resolution + self.costmap_info.origin.position.y
            
            # Check if this obstacle exists in global map
            if not self.is_point_occupied(world_x, world_y, self.global_map, self.global_map_info, threshold=50):
                new_obstacle_count += 1
        
        # If more than 50% of sampled obstacles are new, consider it a new obstacle
        return (new_obstacle_count / total_samples) > 0.5
    
    def is_point_occupied(self, x, y, occupancy_map, map_info, threshold=50):
        """
        Check if a world coordinate point is occupied in given map.
        Returns True if occupied.
        """
        grid_pos = self.world_to_grid(x, y, map_info)
        if grid_pos is None:
            return False
        
        grid_x, grid_y = grid_pos
        height, width = occupancy_map.shape
        
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return False
        
        return occupancy_map[grid_y, grid_x] > threshold
    
    def is_robot_position_safe(self):
        """
        Check if robot's current position is actually in free space (not in collision).
        Checks both costmap and global map to detect if robot is inside an obstacle.
        Returns True if safe, False if in collision.
        """
        # Check in costmap first (real-time obstacle data)
        if self.costmap is not None and self.costmap_info is not None:
            robot_grid = self.world_to_grid(self.current_x, self.current_y, self.costmap_info)
            if robot_grid is not None:
                grid_x, grid_y = robot_grid
                height, width = self.costmap.shape
                
                # Check if robot position is occupied
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    robot_cell_value = self.costmap[grid_y, grid_x]
                    if robot_cell_value > 50:
                        self.get_logger().error(f'❌ COLLISION: Robot position occupied in costmap (value: {robot_cell_value})!')
                        return False
                    
                    # Check immediate surrounding cells (robot footprint)
                    robot_radius_cells = 2  # ~0.2m radius assuming 0.05m resolution
                    for dy in range(-robot_radius_cells, robot_radius_cells + 1):
                        for dx in range(-robot_radius_cells, robot_radius_cells + 1):
                            check_x = grid_x + dx
                            check_y = grid_y + dy
                            
                            if 0 <= check_x < width and 0 <= check_y < height:
                                if self.costmap[check_y, check_x] > 80:  # High threshold for footprint
                                    dist = math.sqrt(dx**2 + dy**2) * self.costmap_info.resolution
                                    self.get_logger().warn(f'⚠️  Obstacle detected {dist:.2fm} from robot center')
                                    if dist < 0.15:  # Very close - likely collision
                                        return False
        
        # Also check global map
        if self.global_map is not None and self.global_map_info is not None:
            robot_grid = self.world_to_grid(self.current_x, self.current_y, self.global_map_info)
            if robot_grid is not None:
                grid_x, grid_y = robot_grid
                height, width = self.global_map.shape
                
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    robot_cell_value = self.global_map[grid_y, grid_x]
                    if robot_cell_value > 50:
                        self.get_logger().error(f'❌ COLLISION: Robot position occupied in global map (value: {robot_cell_value})!')
                        return False
        
        return True
    
    def check_robot_clearance(self):
        """
        Check if robot has at least 0.5m clearance in at least one direction.
        Returns True if robot has escape route, False if surrounded.
        """
        if self.costmap is None or self.costmap_info is None:
            return True  # No map data, assume safe
        
        clearance_distance = 0.5  # meters
        resolution = self.costmap_info.resolution
        clearance_cells = int(clearance_distance / resolution)
        
        # Check 8 directions: N, NE, E, SE, S, SW, W, NW
        directions = [
            (0, 1),    # North
            (1, 1),    # Northeast
            (1, 0),    # East
            (1, -1),   # Southeast
            (0, -1),   # South
            (-1, -1),  # Southwest
            (-1, 0),   # West
            (-1, 1),   # Northwest
        ]
        
        height, width = self.costmap.shape
        origin_x = self.costmap_info.origin.position.x
        origin_y = self.costmap_info.origin.position.y
        
        # Robot position in grid
        robot_grid_x = int((self.current_x - origin_x) / resolution)
        robot_grid_y = int((self.current_y - origin_y) / resolution)
        
        free_directions = 0
        
        for dx, dy in directions:
            # Check if this direction is clear for clearance_distance
            is_clear = True
            
            for step in range(1, clearance_cells + 1):
                check_x = robot_grid_x + dx * step
                check_y = robot_grid_y + dy * step
                
                # Check bounds
                if check_x < 0 or check_x >= width or check_y < 0 or check_y >= height:
                    is_clear = False
                    break
                
                # Check if occupied
                if self.costmap[check_y, check_x] > 50:
                    is_clear = False
                    break
            
            if is_clear:
                free_directions += 1
        
        # Need at least 1 free direction
        if free_directions == 0:
            self.get_logger().warn(f'⚠️  Robot has NO clearance in any direction!')
            return False
        
        return True
    
    def execute_surveying(self):
        """
        SURVEYING state - rotate 360° to help rtabmap update the global /map.
        No map aggregation needed - we only use the global map from /map topic.
        """
        if self.survey_start_yaw is None:
            # Just started survey, record starting angle and total rotation
            self.survey_start_yaw = self.current_yaw
            self.survey_total_rotation = 0.0
            self.survey_last_yaw = self.current_yaw
            self.get_logger().info(f'📡 Starting 360° survey to update global map...')
        
        # Calculate incremental rotation since last update
        delta_yaw = normalize_angle(self.current_yaw - self.survey_last_yaw)
        self.survey_total_rotation += abs(delta_yaw)
        self.survey_last_yaw = self.current_yaw
        
        # Check if completed full rotation (360° = 2*pi)
        if self.survey_total_rotation >= 2 * math.pi - 0.1:  # Allow small tolerance
            self.stop_robot()
            self.get_logger().info(f'✅ Survey complete! Rotated {math.degrees(self.survey_total_rotation):.1f}°')
            self.get_logger().info(f'   Global /map should now be updated by rtabmap')
            
            # Transition to local avoidance with path planning
            self.transition_to_local_avoidance()
            return
        
        # Continue rotating
        cmd_vel = Twist()
        cmd_vel.angular.z = self.survey_angular_speed
        cmd_vel.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
    
    def transition_to_surveying(self):
        """Transition to SURVEYING state and reset survey data."""
        self.state = PlannerState.SURVEYING
        self.survey_start_yaw = None
        self.survey_total_rotation = 0.0
        self.survey_last_yaw = None
    
    def execute_local_avoidance(self):
        """LOCAL_AVOIDANCE state - follow A* path to clearance waypoint."""
        if self.current_pose is None:
            return
        
        # CRITICAL SAFETY CHECK: Verify robot is not in collision
        if not self.is_robot_position_safe():
            self.get_logger().error('❌ COLLISION DETECTED! Robot position is occupied!')
            self.get_logger().error('   Emergency stop - possible odometry drift or actual collision')
            self.stop_robot()
            self.state = PlannerState.IDLE
            return
        
        # Safety check: ensure robot has clearance in at least one direction
        if not self.check_robot_clearance():
            self.get_logger().error('❌ Robot surrounded! Emergency stop.')
            self.stop_robot()
            self.state = PlannerState.IDLE
            return
        
        # Check if path exists
        if len(self.current_path) == 0:
            if self.replan_count >= self.max_replans:
                self.get_logger().error('❌ Max replans reached! Switching to GLOBAL_PLANNER')
                self.state = PlannerState.GLOBAL_PLANNER
                self.replan_count = 0
            else:
                self.get_logger().warn('⚠️  No path to follow! Re-surveying...')
                self.replan_count += 1
                self.transition_to_surveying()
            return
        
        # Check if reached clearance waypoint (end of path)
        if self.current_waypoint_index >= len(self.current_path):
            self.get_logger().info('✅ Reached clearance waypoint!')
            self.replan_count = 0  # Reset counter on success
            
            # Check if we have clear line-of-sight to goal
            if self.check_clear_path_to_goal():
                self.get_logger().info('🎯 Clear path to goal! Switching to GLOBAL_PLANNER')
                self.state = PlannerState.GLOBAL_PLANNER
            else:
                # Check if close enough to goal to try direct approach
                dist_to_goal = self.distance_to_goal()
                if dist_to_goal < 2.0:  # Within 2m of goal
                    self.get_logger().info('📍 Close to goal, switching to GLOBAL_PLANNER')
                    self.state = PlannerState.GLOBAL_PLANNER
                else:
                    self.get_logger().warn('⚠️  Path still blocked. Re-surveying...')
                    self.transition_to_surveying()
            return
        
        # Get current target waypoint
        target_waypoint = self.current_path[self.current_waypoint_index]
        
        # Check if reached current waypoint
        dist_to_waypoint = math.sqrt(
            (target_waypoint[0] - self.current_x)**2 +
            (target_waypoint[1] - self.current_y)**2
        )
        
        if dist_to_waypoint < 0.3:  # Waypoint reached
            self.current_waypoint_index += 1
            self.get_logger().info(f'📍 Waypoint {self.current_waypoint_index}/{len(self.current_path)} reached')
            return
        
        # DYNAMIC REPLANNING: Check /costmap for NEW obstacles in upcoming path
        if self.check_dynamic_obstacles_in_path():
            self.get_logger().warn('🚨 Dynamic obstacle detected in path!')
            
            # Try to replan from current position to clearance waypoint
            if self.clearance_waypoint is not None:
                self.get_logger().info('   Replanning A* from current position...')
                self.plan_astar_path_global(self.clearance_waypoint)
                
                # Reset waypoint index to start of new path
                self.current_waypoint_index = 0
                
                if len(self.current_path) == 0:
                    # Replanning failed
                    if self.replan_count >= self.max_replans:
                        self.get_logger().error('❌ Max replans reached! Emergency stop.')
                        self.stop_robot()
                        self.state = PlannerState.IDLE
                    else:
                        self.get_logger().warn('   Replan failed, re-surveying...')
                        self.replan_count += 1
                        self.transition_to_surveying()
                return
        
        # Navigate to current waypoint
        cmd_vel = self.calculate_waypoint_velocity(target_waypoint)
        self.cmd_vel_pub.publish(cmd_vel)
    
    def calculate_waypoint_velocity(self, waypoint):
        """Calculate velocity command to reach waypoint."""
        cmd_vel = Twist()
        
        # Calculate angle and distance to waypoint
        dx = waypoint[0] - self.current_x
        dy = waypoint[1] - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_angle - self.current_yaw)
        
        # Check for obstacles in front of robot (safety brake)
        obstacle_ahead = self.detect_obstacle_ahead(distance=0.8)
        
        # If angle error is large, rotate first
        if abs(angle_error) > 0.4:  # ~23 degrees
            cmd_vel.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            cmd_vel.linear.x = 0.0
        else:
            # Move forward with steering
            base_speed = min(self.linear_speed, distance * 0.5)
            
            # SAFETY: Reduce speed if obstacle detected ahead
            if obstacle_ahead:
                base_speed = min(base_speed, 0.1)  # Max 0.1 m/s when obstacle nearby
                self.get_logger().warn('⚠️  Obstacle ahead during waypoint follow - reducing speed')
            
            cmd_vel.linear.x = base_speed
            cmd_vel.angular.z = 2.5 * angle_error  # Proportional control
        
        return cmd_vel
    
    def check_clear_path_to_goal(self):
        """
        Check if there's a clear line-of-sight path to goal in global map.
        Returns True if path is clear.
        """
        if self.global_map is None or self.global_map_info is None:
            return False
        
        # Sample points along line from robot to goal
        num_samples = 20
        for i in range(num_samples + 1):
            t = i / num_samples
            sample_x = self.current_x + t * (self.goal_x - self.current_x)
            sample_y = self.current_y + t * (self.goal_y - self.current_y)
            
            # Check if this point is free
            grid_pos = self.world_to_grid(sample_x, sample_y, self.global_map_info)
            if grid_pos is None:
                return False
            
            grid_x, grid_y = grid_pos
            if self.global_map[grid_y, grid_x] > 50:
                return False  # Obstacle in path
        
        return True  # Path is clear
    
    def obstacle_blocks_path(self):
        """
        Check if detected obstacle is actually blocking our planned path.
        Returns True if path is blocked.
        """
        if len(self.current_path) == 0 or self.costmap is None:
            return False
        
        # Check upcoming waypoints
        for i in range(self.current_waypoint_index, min(self.current_waypoint_index + 3, len(self.current_path))):
            waypoint = self.current_path[i]
            
            # Check if this waypoint area has obstacles in costmap
            grid_pos = self.world_to_grid(waypoint[0], waypoint[1], self.costmap_info)
            if grid_pos is None:
                continue
            
            # Check small area around waypoint
            grid_x, grid_y = grid_pos
            height, width = self.costmap.shape
            
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    check_x = grid_x + dx
                    check_y = grid_y + dy
                    
                    if check_x < 0 or check_x >= width or check_y < 0 or check_y >= height:
                        continue
                    
                    if self.costmap[check_y, check_x] > 50:
                        return True  # Waypoint is blocked
        
        return False
    
    def check_dynamic_obstacles_in_path(self):
        """
        Check /costmap for dynamic obstacles blocking upcoming waypoints.
        Uses current costmap (not survey map) to detect NEW obstacles.
        Returns True if obstacle detected in next 3 waypoints.
        """
        if self.costmap is None or self.costmap_info is None:
            return False
        
        if len(self.current_path) == 0:
            return False
        
        # Check next 3 waypoints (or remaining waypoints if fewer)
        waypoints_to_check = min(3, len(self.current_path) - self.current_waypoint_index)
        
        for i in range(waypoints_to_check):
            waypoint_idx = self.current_waypoint_index + i
            if waypoint_idx >= len(self.current_path):
                break
            
            waypoint = self.current_path[waypoint_idx]
            
            # Convert waypoint to costmap grid coordinates
            grid_pos = self.world_to_grid(waypoint[0], waypoint[1], self.costmap_info)
            if grid_pos is None:
                continue
            
            grid_x, grid_y = grid_pos
            height, width = self.costmap.shape
            
            # Check area around waypoint (not just single cell)
            check_radius = 3  # cells
            for dy in range(-check_radius, check_radius + 1):
                for dx in range(-check_radius, check_radius + 1):
                    check_x = grid_x + dx
                    check_y = grid_y + dy
                    
                    if check_x < 0 or check_x >= width or check_y < 0 or check_y >= height:
                        continue
                    
                    # Check if occupied in costmap
                    if self.costmap[check_y, check_x] > 50:
                        # This is a dynamic obstacle - verify it's NOT in global map
                        world_x = check_x * self.costmap_info.resolution + self.costmap_info.origin.position.x
                        world_y = check_y * self.costmap_info.resolution + self.costmap_info.origin.position.y
                        
                        # If it's also in global map, it's not dynamic
                        if self.global_map is not None:
                            if not self.is_point_occupied(world_x, world_y, self.global_map, self.global_map_info, threshold=50):
                                # Found dynamic obstacle!
                                self.get_logger().info(f'   Dynamic obstacle at waypoint {waypoint_idx} ({waypoint[0]:.2f}, {waypoint[1]:.2f})')
                                return True
                        else:
                            # No global map to compare, assume it's blocking
                            return True
        
        return False
    
    # ============================================================
    # UTILITY FUNCTIONS
    # ============================================================
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def transition_to_local_avoidance(self):
        """Transition to LOCAL_AVOIDANCE state with A* path planning."""
        self.get_logger().info('🔄 Transitioning to LOCAL_AVOIDANCE...')
        
        # STRATEGY 1: Try planning directly to the actual goal first
        self.get_logger().info('   Strategy 1: Planning A* directly to goal...')
        self.plan_astar_path_global((self.goal_x, self.goal_y))
        
        if len(self.current_path) > 0:
            self.get_logger().info('✅ A* path to goal found! Using direct path.')
            self.clearance_waypoint = (self.goal_x, self.goal_y)
            self.state = PlannerState.LOCAL_AVOIDANCE
            self.current_waypoint_index = 0
            return
        
        # STRATEGY 2: If direct path fails, find obstacle edge waypoint
        self.get_logger().info('   Strategy 2: Direct path failed, finding obstacle edge waypoint...')
        self.clearance_waypoint = self.find_obstacle_edge_waypoint()
        
        if self.clearance_waypoint is None:
            self.get_logger().error('❌ Could not find any valid waypoint! Stopping.')
            self.stop_robot()
            self.state = PlannerState.IDLE
            return
        
        self.get_logger().info(f'🎯 Obstacle edge waypoint: ({self.clearance_waypoint[0]:.2f}, {self.clearance_waypoint[1]:.2f})')
        
        # Plan A* path to edge waypoint using GLOBAL MAP
        self.plan_astar_path_global(self.clearance_waypoint)
        
        if len(self.current_path) == 0:
            self.get_logger().error('❌ Failed to plan path to edge waypoint! Stopping.')
            self.stop_robot()
            self.state = PlannerState.IDLE
            return
        
        # Transition to local avoidance
        self.state = PlannerState.LOCAL_AVOIDANCE
        self.current_waypoint_index = 0
    
    def find_obstacle_edge_waypoint(self):
        """
        Find optimal obstacle edge point (not extended beyond).
        Returns (x, y) tuple of edge point or None if not found.
        """
        if self.global_map is None or self.global_map_info is None:
            return None
        
        # Detect obstacle edges from GLOBAL MAP only
        edge_points = self.detect_obstacle_edges(
            self.global_map, 
            self.global_map_info
        )
        
        if len(edge_points) == 0:
            self.get_logger().warn('⚠️  No obstacle edges found, using fallback')
            return self.fallback_clearance_point()
        
        self.get_logger().info(f'🔍 Found {len(edge_points)} obstacle edge points')
        
        # Score edge points and select best one
        best_edge = None
        best_score = float('-inf')
        
        for edge_x, edge_y in edge_points:
            # Skip if edge point itself is occupied (use global map)
            if not self.is_free_space(edge_x, edge_y, self.global_map, self.global_map_info, radius=0.3):
                continue
            
            # Calculate score based on alignment with goal
            score = self.score_edge_point(edge_x, edge_y)
            
            if score > best_score:
                best_score = score
                best_edge = (edge_x, edge_y)
        
        if best_edge is None:
            self.get_logger().warn('⚠️  No valid edge points, using fallback')
            return self.fallback_clearance_point()
        
        self.get_logger().info(f'✨ Selected edge point with score: {best_score:.2f}')
        return best_edge
    
    def score_edge_point(self, x, y):
        """
        Score an edge point based on:
        - Distance to goal (closer is better)
        - Angle alignment with goal direction (smaller deviation is better)
        """
        # Distance to goal
        dist_to_goal = math.sqrt((x - self.goal_x)**2 + (y - self.goal_y)**2)
        
        # Angle deviation from goal direction
        goal_bearing = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        edge_bearing = math.atan2(y - self.current_y, x - self.current_x)
        angle_deviation = abs(normalize_angle(edge_bearing - goal_bearing))
        
        # Combined score (prefer closer to goal and aligned with goal direction)
        score = (
            -dist_to_goal * 0.5 +           # Prefer closer to goal
            -angle_deviation * 3.0          # Strongly prefer aligned with goal
        )
        
        return score
    
    def find_clearance_waypoint(self):
        """
        DEPRECATED: Old function replaced by find_obstacle_edge_waypoint.
        Kept for compatibility but not used.
        """
        return self.find_obstacle_edge_waypoint()
    
    def detect_obstacle_edges(self, occupancy_map, map_info):
        """
        Detect obstacle boundary edges using morphological gradient.
        Returns list of (x, y) world coordinates of edge points.
        """
        # Convert to binary (occupied/free)
        binary_map = (occupancy_map > 50).astype(np.uint8)
        
        # Use convolution for edge detection (simple gradient)
        kernel_y = np.array([[-1, -1, -1],
                             [ 0,  0,  0],
                             [ 1,  1,  1]])
        kernel_x = np.array([[-1, 0, 1],
                             [-1, 0, 1],
                             [-1, 0, 1]])
        
        # Apply convolution to detect edges
        try:
            from scipy import ndimage
            grad_x = ndimage.convolve(binary_map.astype(float), kernel_x)
            grad_y = ndimage.convolve(binary_map.astype(float), kernel_y)
        except ImportError:
            self.get_logger().warn('⚠️  scipy not available, using simple edge detection')
            # Fallback: simple numpy-based edge detection
            grad_x = np.zeros_like(binary_map, dtype=float)
            grad_y = np.zeros_like(binary_map, dtype=float)
            
            # Manual convolution
            for i in range(1, binary_map.shape[0] - 1):
                for j in range(1, binary_map.shape[1] - 1):
                    # Sobel-like operator
                    gx = (binary_map[i-1, j+1] + 2*binary_map[i, j+1] + binary_map[i+1, j+1] -
                          binary_map[i-1, j-1] - 2*binary_map[i, j-1] - binary_map[i+1, j-1])
                    gy = (binary_map[i+1, j-1] + 2*binary_map[i+1, j] + binary_map[i+1, j+1] -
                          binary_map[i-1, j-1] - 2*binary_map[i-1, j] - binary_map[i-1, j+1])
                    grad_x[i, j] = gx
                    grad_y[i, j] = gy
        
        gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        
        # Threshold to get edge points
        edges = gradient_magnitude > 1.0
        edge_coords = np.where(edges)
        
        # Convert to world coordinates
        edge_points = []
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        
        # Sample edges (take every 5th point to reduce computation)
        for i in range(0, len(edge_coords[0]), 5):
            grid_y = edge_coords[0][i]
            grid_x = edge_coords[1][i]
            
            world_x = grid_x * resolution + origin_x
            world_y = grid_y * resolution + origin_y
            
            edge_points.append((world_x, world_y))
        
        return edge_points
    
    def is_free_space(self, x, y, occupancy_map, map_info, radius=0.3):
        """
        Check if position (x, y) is in free space with safety radius.
        Returns True if free, False if occupied or out of bounds.
        """
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        height, width = occupancy_map.shape
        
        # Convert to grid coordinates
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        # Check area within radius
        cells_radius = int(radius / resolution)
        
        for dy in range(-cells_radius, cells_radius + 1):
            for dx in range(-cells_radius, cells_radius + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy
                
                # Check bounds
                if check_x < 0 or check_x >= width or check_y < 0 or check_y >= height:
                    return False
                
                # Check if occupied
                if occupancy_map[check_y, check_x] > 50:
                    return False
        
        return True
    
    def score_clearance_candidate(self, x, y):
        """
        Score a clearance waypoint candidate based on:
        - Distance to goal (closer is better)
        - Angle deviation from goal direction (smaller is better)
        - Distance from nearest obstacle (larger is better)
        """
        # Distance to goal
        dist_to_goal = math.sqrt((x - self.goal_x)**2 + (y - self.goal_y)**2)
        
        # Angle deviation from goal direction
        goal_bearing = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        candidate_bearing = math.atan2(y - self.current_y, x - self.current_x)
        angle_deviation = abs(normalize_angle(candidate_bearing - goal_bearing))
        
        # Distance from nearest obstacle (simplified - use local check)
        obstacle_distance = self.nearest_obstacle_distance(x, y)
        
        # Combined score (tune weights as needed)
        score = (
            -dist_to_goal * 0.5 +           # Prefer closer to goal
            -angle_deviation * 2.0 +        # Prefer aligned with goal
            obstacle_distance * 1.5         # Prefer far from obstacles
        )
        
        return score
    
    def nearest_obstacle_distance(self, x, y):
        """
        Find distance to nearest obstacle from point (x, y).
        Returns distance in meters.
        """
        if self.global_map is None or self.global_map_info is None:
            return 0.0
        
        resolution = self.global_map_info.resolution
        origin_x = self.global_map_info.origin.position.x
        origin_y = self.global_map_info.origin.position.y
        height, width = self.global_map.shape
        
        # Convert to grid
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        # Search in expanding radius
        max_search_radius = 20  # cells
        for radius in range(1, max_search_radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    check_x = grid_x + dx
                    check_y = grid_y + dy
                    
                    if check_x < 0 or check_x >= width or check_y < 0 or check_y >= height:
                        continue
                    
                    if self.global_map[check_y, check_x] > 50:
                        return math.sqrt(dx**2 + dy**2) * resolution
        
        return max_search_radius * resolution  # Max distance
    
    def fallback_clearance_point(self):
        """
        Fallback clearance point selection when edge detection fails.
        Returns a point in the general direction of the goal that's clear.
        """
        # Try points at different angles around the robot
        for angle_offset in [0, 0.5, -0.5, 1.0, -1.0, 1.57, -1.57]:
            test_distance = self.clearance_margin + 1.0
            test_x = self.current_x + test_distance * math.cos(self.current_yaw + angle_offset)
            test_y = self.current_y + test_distance * math.sin(self.current_yaw + angle_offset)
            
            if self.global_map is not None:
                if self.is_free_space(test_x, test_y, self.global_map, self.global_map_info):
                    return (test_x, test_y)
        
        # Last resort: point ahead of robot
        fallback_x = self.current_x + 2.0 * math.cos(self.current_yaw)
        fallback_y = self.current_y + 2.0 * math.sin(self.current_yaw)
        return (fallback_x, fallback_y)
    
    def distance_to_goal(self):
        """Calculate distance to goal."""
        if self.goal_x is None or self.goal_y is None:
            return float('inf')
        return math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
    
    # ============================================================
    # A* PATH PLANNING (GLOBAL MAP ONLY)
    # ============================================================
    
    def plan_astar_path_global(self, target_point):
        """
        Plan A* path using GLOBAL MAP only (rtabmap's /map).
        Validates path against global map for obstacles.
        
        Args:
            target_point: (x, y) tuple of target position
        """
        if self.global_map is None or self.global_map_info is None:
            self.get_logger().error('❌ No global map available for A* planning!')
            self.current_path = [target_point]  # Fallback: direct path
            return
        
        # Reset inflation radius to default before planning
        base_inflation = self.get_parameter('obstacle_inflation_radius').value
        self.obstacle_inflation_radius = base_inflation
        
        self.get_logger().info(f'🗺️  Planning A* on GLOBAL MAP to ({target_point[0]:.2f}, {target_point[1]:.2f})')
        self.get_logger().info(f'   Global map size: {self.global_map.shape}, resolution: {self.global_map_info.resolution:.3f}m')
        
        # Get map dimensions
        height, width = self.global_map.shape
        
        # Count obstacles in global map
        occupied_cells = np.sum(self.global_map > 50)
        free_cells = np.sum(self.global_map == 0)
        unknown_cells = np.sum(self.global_map < 0)
        self.get_logger().info(f'   Map stats: {occupied_cells} occupied, {free_cells} free, {unknown_cells} unknown')
        
        # Convert start and goal to grid coordinates
        start_grid = self.world_to_grid(self.current_x, self.current_y, self.global_map_info)
        goal_grid = self.world_to_grid(target_point[0], target_point[1], self.global_map_info)
        
        self.get_logger().info(f'   Start: world({self.current_x:.2f}, {self.current_y:.2f}) -> grid{start_grid}')
        self.get_logger().info(f'   Goal:  world({target_point[0]:.2f}, {target_point[1]:.2f}) -> grid{goal_grid}')
        
        if start_grid is None:
            self.get_logger().error('❌ Start position outside global map bounds!')
            self.get_logger().error(f'   Map origin: ({self.global_map_info.origin.position.x:.2f}, {self.global_map_info.origin.position.y:.2f})')
            self.get_logger().error(f'   Map size: {width} x {height} cells ({width * self.global_map_info.resolution:.2f}m x {height * self.global_map_info.resolution:.2f}m)')
            self.current_path = []  # Empty path signals failure
            return
        
        if goal_grid is None:
            self.get_logger().error('❌ Goal position outside global map bounds!')
            self.get_logger().error(f'   Map origin: ({self.global_map_info.origin.position.x:.2f}, {self.global_map_info.origin.position.y:.2f})')
            self.get_logger().error(f'   Map size: {width} x {height} cells ({width * self.global_map_info.resolution:.2f}m x {height * self.global_map_info.resolution:.2f}m)')
            max_x = self.global_map_info.origin.position.x + width * self.global_map_info.resolution
            max_y = self.global_map_info.origin.position.y + height * self.global_map_info.resolution
            self.get_logger().error(f'   Map bounds: X[{self.global_map_info.origin.position.x:.2f}, {max_x:.2f}], Y[{self.global_map_info.origin.position.y:.2f}, {max_y:.2f}]')
            self.current_path = []  # Empty path signals failure
            return
        
        # Check start and goal occupancy values
        start_value = self.global_map[start_grid[1], start_grid[0]]
        goal_value = self.global_map[goal_grid[1], goal_grid[0]]
        self.get_logger().info(f'   Start cell value: {start_value}, Goal cell value: {goal_value}')
        
        # Check if goal is in free space, if not find nearest free cell
        if goal_value > 50:
            self.get_logger().warn('   Goal is occupied, finding nearest free cell...')
            goal_grid = self.find_nearest_free_cell(goal_grid, self.global_map)
            if goal_grid is None:
                self.get_logger().error('❌ Could not find free cell near goal')
                self.current_path = [target_point]
                return
        
        # Inflate obstacles for safety margin
        inflated_map = self.inflate_obstacles(self.global_map, self.global_map_info)
        
        # Run A* algorithm with inflated map (try up to 3 times with increasing inflation)
        max_attempts = 3
        path_grid = None
        
        for attempt in range(max_attempts):
            path_grid = self.astar_search(start_grid, goal_grid, inflated_map, self.global_map_info)
            
            if path_grid is None or len(path_grid) == 0:
                self.get_logger().warn(f'⚠️  A* search failed on attempt {attempt + 1}/{max_attempts}')
                if attempt < max_attempts - 1:
                    # Increase inflation and retry
                    self.obstacle_inflation_radius += 0.2
                    inflated_map = self.inflate_obstacles(self.global_map, self.global_map_info)
                continue
            
            # Validate path doesn't go through obstacles in GLOBAL MAP
            if self.validate_path_clear(path_grid, self.global_map):
                self.get_logger().info(f'✅ Path validated: clear of obstacles in global map')
                break
            else:
                self.get_logger().warn(f'⚠️  Path goes through obstacles! Attempt {attempt + 1}/{max_attempts}')
                path_grid = None
                if attempt < max_attempts - 1:
                    # Increase inflation and retry
                    self.obstacle_inflation_radius += 0.3
                    inflated_map = self.inflate_obstacles(self.global_map, self.global_map_info)
        
        if path_grid is None or len(path_grid) == 0:
            self.get_logger().error('❌ Failed to find valid path after all attempts')
            self.current_path = [target_point]
            self.publish_path(self.current_path)
            return
        
        # Convert grid path to world coordinates
        path_world = []
        for grid_pos in path_grid:
            world_pos = self.grid_to_world(grid_pos[0], grid_pos[1], self.global_map_info)
            path_world.append(world_pos)
        
        # Smooth and downsample path
        self.current_path = self.smooth_path(path_world)
        
        self.get_logger().info(f'✅ A* path generated: {len(self.current_path)} waypoints')
        
        # Publish path for visualization
        self.publish_path(self.current_path)
    
    def plan_on_map(self, target_point, occupancy_map, map_info):
        """
        Plan A* path on a given occupancy map.
        Returns list of (x, y) world coordinates or None if planning fails.
        """
        # Convert start and goal to grid coordinates
        start_grid = self.world_to_grid(self.current_x, self.current_y, map_info)
        goal_grid = self.world_to_grid(target_point[0], target_point[1], map_info)
        
        if start_grid is None:
            self.get_logger().warn('   Start position outside map bounds')
            return None
        
        if goal_grid is None:
            self.get_logger().warn('   Goal position outside map bounds')
            return None
        
        # Check if goal is in free space, if not find nearest free cell
        height, width = occupancy_map.shape
        if occupancy_map[goal_grid[1], goal_grid[0]] > 50 or occupancy_map[goal_grid[1], goal_grid[0]] < 0:
            self.get_logger().warn('   Goal is occupied/unknown, finding nearest free cell...')
            goal_grid = self.find_nearest_free_cell(goal_grid, occupancy_map)
            if goal_grid is None:
                self.get_logger().warn('   Could not find free cell near goal')
                return None
        
        # Inflate obstacles for safety margin
        inflated_map = self.inflate_obstacles(occupancy_map, map_info)
        
        # Run A* algorithm with inflated map (try up to 3 times)
        max_attempts = 3
        for attempt in range(max_attempts):
            path_grid = self.astar_search(start_grid, goal_grid, inflated_map, map_info)
            
            if path_grid is None or len(path_grid) == 0:
                return None
            
            # Validate path doesn't go through obstacles in ORIGINAL map
            if self.validate_path_clear(path_grid, occupancy_map):
                self.get_logger().info(f'✅ Path validated: clear of obstacles')
                break
            else:
                self.get_logger().warn(f'⚠️  Path goes through obstacles! Attempt {attempt + 1}/{max_attempts}')
                if attempt < max_attempts - 1:
                    # Increase inflation and retry
                    self.obstacle_inflation_radius += 0.2
                    inflated_map = self.inflate_obstacles(occupancy_map, map_info)
                else:
                    self.get_logger().error('❌ Failed to find obstacle-free path after retries')
                    return None
        
        # Convert grid path to world coordinates
        path_world = []
        for grid_pos in path_grid:
            world_pos = self.grid_to_world(grid_pos[0], grid_pos[1], map_info)
            path_world.append(world_pos)
        
        # Smooth and downsample path
        return self.smooth_path(path_world)
    
    def inflate_obstacles(self, occupancy_map, map_info):
        """
        Inflate obstacles by safety margin to ensure paths stay away.
        Returns inflated occupancy map.
        """
        inflated = occupancy_map.copy()
        height, width = occupancy_map.shape
        
        # Calculate inflation radius in cells (use ceiling and add buffer)
        inflation_cells = int(math.ceil(self.obstacle_inflation_radius / map_info.resolution)) + 2
        
        self.get_logger().info(f'🔧 Inflating obstacles by {inflation_cells} cells ({inflation_cells * map_info.resolution:.2f}m)')
        
        # Find all occupied cells
        occupied = np.where(occupancy_map > 50)
        original_obstacles = len(occupied[0])
        
        # Inflate around each occupied cell
        for i in range(len(occupied[0])):
            center_y = occupied[0][i]
            center_x = occupied[1][i]
            
            # Inflate in circular pattern
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    # Check if within inflation radius (circular)
                    dist = math.sqrt(dx**2 + dy**2)
                    if dist > inflation_cells:
                        continue
                    
                    inflate_y = center_y + dy
                    inflate_x = center_x + dx
                    
                    # Check bounds
                    if inflate_x < 0 or inflate_x >= width or inflate_y < 0 or inflate_y >= height:
                        continue
                    
                    # Mark as highly occupied - use HIGH values to ensure blocking
                    if inflated[inflate_y, inflate_x] <= 50:
                        # Aggressive inflation: mark all as 100 (fully blocked)
                        inflated[inflate_y, inflate_x] = 100
        
        inflated_obstacles = np.sum(inflated > 50)
        self.get_logger().info(f'   Original obstacles: {original_obstacles}, After inflation: {inflated_obstacles}')
        
        return inflated
    
    def validate_path_clear(self, path_grid, occupancy_map):
        """
        Validate that path doesn't go through obstacles in original map.
        Returns True if path is clear, False if it intersects obstacles.
        """
        height, width = occupancy_map.shape
        
        for i, cell in enumerate(path_grid):
            grid_x, grid_y = cell
            
            # Check bounds
            if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
                self.get_logger().warn(f'   Path cell {i} out of bounds: ({grid_x}, {grid_y})')
                return False
            
            # Check if cell is occupied in ORIGINAL map (threshold: 50)
            cell_value = occupancy_map[grid_y, grid_x]
            if cell_value > 50:
                self.get_logger().warn(f'   Path cell {i} is OCCUPIED: value={cell_value} at ({grid_x}, {grid_y})')
                return False
        
        return True
    
    def find_nearest_free_cell(self, goal_grid, occupancy_map):
        """Find nearest free cell to goal position."""
        height, width = occupancy_map.shape
        max_search = 50  # cells
        
        for radius in range(1, max_search):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    test_x = goal_grid[0] + dx
                    test_y = goal_grid[1] + dy
                    
                    if test_x < 0 or test_x >= width or test_y < 0 or test_y >= height:
                        continue
                    
                    # Check if free (0-50 range, not unknown)
                    cell_value = occupancy_map[test_y, test_x]
                    if 0 <= cell_value <= 50:
                        return (test_x, test_y)
        
        return None
    
    def find_nearest_reachable_point(self, target_point):
        """
        Find nearest point toward target that is reachable.
        Samples points along the line to target and checks reachability.
        """
        # Sample points at different distances toward target
        dx = target_point[0] - self.current_x
        dy = target_point[1] - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:
            return None
        
        # Try progressively closer points
        for fraction in [0.8, 0.6, 0.4, 0.2]:
            test_x = self.current_x + dx * fraction
            test_y = self.current_y + dy * fraction
            
            # Check if reachable on survey map
            if self.aggregated_survey_map is not None:
                if self.is_free_space(test_x, test_y, self.aggregated_survey_map, self.costmap_info):
                    # Try to plan to this point
                    test_grid = self.world_to_grid(test_x, test_y, self.costmap_info)
                    start_grid = self.world_to_grid(self.current_x, self.current_y, self.costmap_info)
                    
                    if test_grid is not None and start_grid is not None:
                        path = self.astar_search(start_grid, test_grid, self.aggregated_survey_map, self.costmap_info)
                        if path is not None:
                            return (test_x, test_y)
        
        return None
    
    def astar_search(self, start, goal, occupancy_map, map_info):
        """
        A* pathfinding algorithm on occupancy grid.
        Uses Octile distance heuristic for better 8-connected grid performance.
        Returns list of (grid_x, grid_y) coordinates from start to goal.
        """
        height, width = occupancy_map.shape
        
        self.get_logger().info(f'🔍 A* search: start{start} -> goal{goal} on {width}x{height} grid')
        
        # Calculate straight-line distance
        straight_dist = math.sqrt((goal[0] - start[0])**2 + (goal[1] - start[1])**2)
        self.get_logger().info(f'   Straight-line distance: {straight_dist:.1f} cells ({straight_dist * map_info.resolution:.2f}m)')
        
        # Priority queue: (f_score, counter, position, g_score)
        counter = 0
        open_set = []
        heapq.heappush(open_set, (0, counter, start, 0))
        
        came_from = {}
        g_score = {start: 0}
        closed_set = set()
        
        # Maximum iterations to prevent infinite loops
        max_iterations = width * height // 4
        iterations = 0
        
        # A* main loop
        while open_set and iterations < max_iterations:
            iterations += 1
            _, _, current, current_g = heapq.heappop(open_set)
            
            # Check if reached goal (allow small tolerance)
            if abs(current[0] - goal[0]) <= 1 and abs(current[1] - goal[1]) <= 1:
                path = self.reconstruct_path(came_from, current)
                self.get_logger().info(f'✅ A* found path with {len(path)} nodes after {iterations} iterations')
                return path
            
            # Skip if already processed
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            # Explore neighbors (8-connected grid)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), 
                          (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Check bounds
                if neighbor[0] < 0 or neighbor[0] >= width or \
                   neighbor[1] < 0 or neighbor[1] >= height:
                    continue
                
                # STRICT obstacle avoidance: reject cells > 50
                cell_value = occupancy_map[neighbor[1], neighbor[0]]
                if cell_value > 50:
                    continue
                
                # Also check diagonal movements don't cut corners through obstacles
                if dx != 0 and dy != 0:
                    # Check adjacent cells for diagonal move
                    adj1 = occupancy_map[current[1] + dy, current[0]]
                    adj2 = occupancy_map[current[1], current[0] + dx]
                    if adj1 > 50 or adj2 > 50:
                        continue  # Don't cut through obstacle corners
                
                # ALLOW unknown cells (< 0) - treat as free space
                # This is important for rtabmap's global map which may have unexplored areas
                # We rely on /costmap for dynamic obstacle detection during execution
                
                # Calculate cost (diagonal = sqrt(2), straight = 1)
                move_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                
                # Add cost based on proximity to obstacles (prefer safer paths)
                # Higher cost for cells closer to inflated obstacle boundary
                obstacle_cost = 0.0
                if cell_value > 0:
                    obstacle_cost = (cell_value / 50.0) * 2.0  # 0 to 2.0 range for inflated areas
                
                tentative_g = current_g + move_cost + obstacle_cost
                
                # Check if this path is better
                if neighbor in g_score and tentative_g >= g_score[neighbor]:
                    continue
                
                # Update path
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                
                # Calculate heuristic - Octile distance (better for 8-connected grid)
                dx_goal = abs(neighbor[0] - goal[0])
                dy_goal = abs(neighbor[1] - goal[1])
                # Octile: D * (dx + dy) + (D2 - 2*D) * min(dx, dy)
                # where D=1.0 (straight), D2=1.414 (diagonal)
                h_score = 1.0 * (dx_goal + dy_goal) + (1.414 - 2.0) * min(dx_goal, dy_goal)
                
                f_score = tentative_g + h_score
                
                counter += 1
                heapq.heappush(open_set, (f_score, counter, neighbor, tentative_g))
        
        # No path found
        if iterations >= max_iterations:
            self.get_logger().warn(f'⚠️  A* reached max iterations ({max_iterations})')
        else:
            self.get_logger().warn('⚠️  A* could not find path to goal')
        return None
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from A* came_from dictionary."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def smooth_path(self, path_world, downsample_distance=0.3):
        """
        Smooth and downsample path to reduce waypoints.
        Keep waypoints at least downsample_distance apart.
        """
        if len(path_world) <= 2:
            return path_world
        
        smoothed = [path_world[0]]
        
        for i in range(1, len(path_world) - 1):
            # Calculate distance from last added waypoint
            dist = math.sqrt(
                (path_world[i][0] - smoothed[-1][0])**2 +
                (path_world[i][1] - smoothed[-1][1])**2
            )
            
            if dist >= downsample_distance:
                smoothed.append(path_world[i])
        
        # Always include final waypoint
        smoothed.append(path_world[-1])
        
        return smoothed
    
    def world_to_grid(self, x, y, map_info):
        """Convert world coordinates to grid coordinates."""
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        width = map_info.width
        height = map_info.height
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        # Check bounds
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return None
        
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x, grid_y, map_info):
        """Convert grid coordinates to world coordinates."""
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        
        world_x = grid_x * resolution + origin_x
        world_y = grid_y * resolution + origin_y
        
        return (world_x, world_y)
    
    def publish_path(self, path_waypoints):
        """Publish path for RViz visualization."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for waypoint in path_waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HybridNavigationPlanner()
    
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
