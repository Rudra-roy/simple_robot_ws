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
        
        self.obstacle_radius = self.get_parameter('obstacle_detection_radius').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.survey_angular_speed = self.get_parameter('survey_angular_speed').value
        self.clearance_margin = self.get_parameter('clearance_margin').value
        
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
        self.survey_snapshots = []  # Store costmap snapshots during rotation
        self.aggregated_survey_map = None
        
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
        
        # Store snapshot during survey
        if self.state == PlannerState.SURVEYING:
            self.survey_snapshots.append({
                'costmap': self.costmap.copy(),
                'robot_yaw': self.current_yaw,
                'info': msg.info
            })
    
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
        
        # Check if goal reached
        dist_to_goal = self.distance_to_goal()
        if dist_to_goal < self.goal_tolerance:
            self.stop_robot()
            self.state = PlannerState.IDLE
            self.get_logger().info('🎉 Goal reached!')
            return
        
        # Check for obstacles in the path
        if self.detect_obstacle_in_radius(self.obstacle_radius):
            self.get_logger().warn(f'⚠️  Obstacle detected within {self.obstacle_radius}m!')
            self.get_logger().info('   Switching to SURVEYING mode')
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
        
        # If angle error is large, rotate in place first
        if abs(angle_error) > 0.3:  # ~17 degrees
            cmd_vel.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            cmd_vel.linear.x = 0.0
        else:
            # Move forward with proportional control
            cmd_vel.linear.x = min(self.linear_speed, goal_distance * 0.5)
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
    
    def execute_surveying(self):
        """SURVEYING state - rotate 360° to gather obstacle data."""
        if self.survey_start_yaw is None:
            # Just started survey, record starting angle and total rotation
            self.survey_start_yaw = self.current_yaw
            self.survey_total_rotation = 0.0
            self.survey_last_yaw = self.current_yaw
            self.survey_snapshots = []
            self.get_logger().info(f'📡 Starting 360° survey at yaw={self.current_yaw:.2f}')
        
        # Calculate incremental rotation since last update
        delta_yaw = normalize_angle(self.current_yaw - self.survey_last_yaw)
        self.survey_total_rotation += abs(delta_yaw)
        self.survey_last_yaw = self.current_yaw
        
        # Check if completed full rotation (360° = 2*pi)
        if self.survey_total_rotation >= 2 * math.pi - 0.1:  # Allow small tolerance
            self.stop_robot()
            self.get_logger().info(f'✅ Survey complete! Rotated {self.survey_total_rotation:.2f} rad ({math.degrees(self.survey_total_rotation):.1f}°)')
            self.get_logger().info(f'   Collected {len(self.survey_snapshots)} snapshots')
            
            # Aggregate all survey data
            self.aggregate_survey_data()
            
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
        self.survey_snapshots = []
        self.aggregated_survey_map = None
    
    def aggregate_survey_data(self):
        """
        Aggregate all costmap snapshots from 360° rotation into single map.
        Takes the maximum occupancy value at each cell across all snapshots.
        """
        if len(self.survey_snapshots) == 0:
            self.get_logger().warn('⚠️  No survey snapshots to aggregate!')
            return
        
        # Use the most recent costmap info
        template = self.survey_snapshots[-1]['costmap']
        aggregated = np.zeros_like(template, dtype=np.int8)
        
        # Take maximum occupancy across all snapshots (most conservative)
        for snapshot in self.survey_snapshots:
            aggregated = np.maximum(aggregated, snapshot['costmap'])
        
        self.aggregated_survey_map = aggregated
        
        # Count occupied cells for logging
        occupied_cells = np.sum(aggregated > 50)
        total_cells = aggregated.size
        occupancy_percent = (occupied_cells / total_cells) * 100
        
        self.get_logger().info(f'📊 Aggregated survey map:')
        self.get_logger().info(f'   Occupied cells: {occupied_cells} ({occupancy_percent:.1f}%)')
        self.get_logger().info(f'   Map size: {aggregated.shape}')
    
    def execute_local_avoidance(self):
        """LOCAL_AVOIDANCE state - follow A* path to clearance waypoint."""
        if self.current_pose is None:
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
        
        # Check for new obstacles blocking current waypoint
        if self.detect_obstacle_in_radius(self.obstacle_radius):
            if self.obstacle_blocks_path():
                if self.replan_count >= self.max_replans:
                    self.get_logger().error('❌ Max replans reached! Trying to push through...')
                    # Don't re-survey, try to navigate anyway
                else:
                    self.get_logger().warn(f'⚠️  Obstacle blocking path! Replanning... ({self.replan_count + 1}/{self.max_replans})')
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
        
        # If angle error is large, rotate first
        if abs(angle_error) > 0.4:  # ~23 degrees
            cmd_vel.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            cmd_vel.linear.x = 0.0
        else:
            # Move forward with steering
            cmd_vel.linear.x = min(self.linear_speed, distance * 0.5)
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
    
    # ============================================================
    # UTILITY FUNCTIONS
    # ============================================================
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def transition_to_local_avoidance(self):
        """Transition to LOCAL_AVOIDANCE state with clearance waypoint calculation."""
        # Find the optimal clearance waypoint
        self.clearance_waypoint = self.find_clearance_waypoint()
        
        if self.clearance_waypoint is None:
            self.get_logger().error('❌ Could not find clearance waypoint! Stopping.')
            self.stop_robot()
            self.state = PlannerState.IDLE
            return
        
        self.get_logger().info(f'🎯 Clearance waypoint: ({self.clearance_waypoint[0]:.2f}, {self.clearance_waypoint[1]:.2f})')
        
        # Plan A* path to clearance waypoint
        self.plan_astar_path(self.clearance_waypoint)
        
        # Transition to local avoidance
        self.state = PlannerState.LOCAL_AVOIDANCE
        self.current_waypoint_index = 0
    
    def find_clearance_waypoint(self):
        """
        Find optimal clearance waypoint beyond obstacle edges.
        Returns (x, y) tuple or None if not found.
        """
        if self.aggregated_survey_map is None or self.costmap_info is None:
            return None
        
        # Detect obstacle edges
        edge_points = self.detect_obstacle_edges(
            self.aggregated_survey_map, 
            self.costmap_info
        )
        
        if len(edge_points) == 0:
            self.get_logger().warn('⚠️  No obstacle edges found, using fallback')
            return self.fallback_clearance_point()
        
        self.get_logger().info(f'🔍 Found {len(edge_points)} obstacle edge points')
        
        # Generate clearance waypoint candidates
        candidates = []
        
        for edge_x, edge_y in edge_points:
            # Vector from robot to edge
            dx = edge_x - self.current_x
            dy = edge_y - self.current_y
            edge_distance = math.sqrt(dx**2 + dy**2)
            
            if edge_distance < 0.1:  # Skip if too close
                continue
            
            # Extend beyond edge by clearance margin
            extend_factor = (edge_distance + self.clearance_margin) / edge_distance
            clearance_x = self.current_x + dx * extend_factor
            clearance_y = self.current_y + dy * extend_factor
            
            # Verify it's in free space on aggregated map
            if not self.is_free_space(clearance_x, clearance_y, self.aggregated_survey_map, self.costmap_info, radius=0.5):
                continue
            
            # Calculate score
            score = self.score_clearance_candidate(clearance_x, clearance_y)
            
            candidates.append({
                'point': (clearance_x, clearance_y),
                'score': score,
                'edge_point': (edge_x, edge_y)
            })
        
        if len(candidates) == 0:
            self.get_logger().warn('⚠️  No valid clearance candidates, using fallback')
            return self.fallback_clearance_point()
        
        # Select best candidate
        best = max(candidates, key=lambda x: x['score'])
        self.get_logger().info(f'✨ Selected clearance point with score: {best["score"]:.2f}')
        
        return best['point']
    
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
        if self.aggregated_survey_map is None or self.costmap_info is None:
            return 0.0
        
        resolution = self.costmap_info.resolution
        origin_x = self.costmap_info.origin.position.x
        origin_y = self.costmap_info.origin.position.y
        height, width = self.aggregated_survey_map.shape
        
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
                    
                    if self.aggregated_survey_map[check_y, check_x] > 50:
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
            
            if self.aggregated_survey_map is not None:
                if self.is_free_space(test_x, test_y, self.aggregated_survey_map, self.costmap_info):
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
    # A* PATH PLANNING
    # ============================================================
    
    def plan_astar_path(self, target_point):
        """
        Plan A* path from current position to target point.
        Uses aggregated survey map (local) + global map (if available).
        Implements dynamic replanning with multiple strategies.
        """
        self.get_logger().info(f'🗺️  Planning path to ({target_point[0]:.2f}, {target_point[1]:.2f})')
        
        # Strategy 1: Try A* on aggregated survey map (most reliable, local data)
        if self.aggregated_survey_map is not None and self.costmap_info is not None:
            self.get_logger().info('   Trying A* on survey map...')
            path = self.plan_on_map(target_point, self.aggregated_survey_map, self.costmap_info)
            if path is not None:
                self.current_path = path
                self.get_logger().info(f'✅ Survey-based path: {len(path)} waypoints')
                self.publish_path(path)
                return
        
        # Strategy 2: Try A* on global map (broader coverage)
        if self.global_map is not None and self.global_map_info is not None:
            self.get_logger().info('   Trying A* on global map...')
            path = self.plan_on_map(target_point, self.global_map, self.global_map_info)
            if path is not None:
                self.current_path = path
                self.get_logger().info(f'✅ Global map path: {len(path)} waypoints')
                self.publish_path(path)
                return
        
        # Strategy 3: Find nearest reachable point toward goal
        self.get_logger().warn('⚠️  A* failed, finding nearest reachable point...')
        reachable_point = self.find_nearest_reachable_point(target_point)
        
        if reachable_point is not None:
            self.current_path = [reachable_point]
            self.get_logger().info(f'✅ Using reachable waypoint: ({reachable_point[0]:.2f}, {reachable_point[1]:.2f})')
            self.publish_path(self.current_path)
            return
        
        # Strategy 4: Direct path (last resort)
        self.get_logger().warn('⚠️  All strategies failed! Using direct path.')
        self.current_path = [target_point]
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
        
        # Run A* algorithm with Octile distance heuristic
        path_grid = self.astar_search(start_grid, goal_grid, occupancy_map, map_info)
        
        if path_grid is None or len(path_grid) == 0:
            return None
        
        # Convert grid path to world coordinates
        path_world = []
        for grid_pos in path_grid:
            world_pos = self.grid_to_world(grid_pos[0], grid_pos[1], map_info)
            path_world.append(world_pos)
        
        # Smooth and downsample path
        return self.smooth_path(path_world)
    
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
                return self.reconstruct_path(came_from, current)
            
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
                
                # Check if occupied (threshold: 50) or unknown (-1)
                cell_value = occupancy_map[neighbor[1], neighbor[0]]
                if cell_value > 50:
                    continue
                
                # Skip unknown cells unless close to goal
                if cell_value < 0:
                    dist_to_goal = max(abs(neighbor[0] - goal[0]), abs(neighbor[1] - goal[1]))
                    if dist_to_goal > 5:  # Allow unknown cells close to goal
                        continue
                
                # Calculate cost (diagonal = sqrt(2), straight = 1)
                move_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                
                # Add cost based on proximity to obstacles (prefer safer paths)
                obstacle_cost = 0.0
                if cell_value > 0:
                    obstacle_cost = cell_value / 50.0  # 0 to 2.0 range
                
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
