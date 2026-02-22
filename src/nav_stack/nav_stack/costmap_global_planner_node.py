#!/usr/bin/env python3
"""
GPS-Based Global Planner Node
Navigates using GPS coordinates (haversine distance) and compass heading (bearing angle).
Uses ZED visual odometry for local obstacle detection in costmap.
No dependency on odometry for navigation - pure GPS/compass based.

Subscriptions:
    /sbg/gps_pos (SbgGpsPos) - GPS position from SBG
    /witmotion_eular/yaw (Float64) - Heading from Witmotion IMU (0-360°, 0=North)
    /waypoints (String) - Goal waypoint as "lat,lon,type"
    /zed/zed_node/odom (Odometry) - ZED visual odometry for obstacle detection
    /costmap (OccupancyGrid) - Local costmap for obstacle checking

Publications:
    /cmd_vel (Twist) - Velocity commands
    /mission_manager (String) - "reached" signal on goal completion
    /costmap_local_planner_trigger (Bool) - Trigger local planner on obstacle
    /costmap_planner_state (String) - Planner state coordination
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String, Float64
from sbg_driver.msg import SbgGpsPos
from visualization_msgs.msg import Marker
import math
import struct


class CostmapGlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('costmap_global_planner_node')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.6)
        self.declare_parameter('safety_margin', 0.2)
        self.declare_parameter('linear_velocity', 2.2)
        self.declare_parameter('angular_velocity', 4.0)
        self.declare_parameter('goal_tolerance', 1.0)  # 1 meter for GPS navigation
        self.declare_parameter('angular_tolerance', 5.0)  # 5 degrees for heading alignment
        self.declare_parameter('obstacle_check_distance', 4.0)
        self.declare_parameter('costmap_obstacle_threshold', 70)
        self.declare_parameter('gps_accuracy_threshold', 100000.0)  # Max acceptable GPS accuracy in meters
        
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value  # meters
        self.angular_tolerance = self.get_parameter('angular_tolerance').value  # degrees
        self.obstacle_check_distance = self.get_parameter('obstacle_check_distance').value
        self.costmap_threshold = self.get_parameter('costmap_obstacle_threshold').value
        self.gps_accuracy_threshold = self.get_parameter('gps_accuracy_threshold').value
        
        # GPS Navigation State
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None  # 0-360 degrees, 0=North, 90=East
        self.gps_accuracy = float('inf')
        self.gps_valid = False
        
        # Goal State (from /waypoints topic)
        self.goal_lat = None
        self.goal_lon = None
        self.waypoint_type = None
        
        # Local pose for obstacle detection (from ZED odom)
        self.local_pose = None
        self.local_pose_valid = False
        
        # Initialization flags
        self.gps_received = False
        self.heading_received = False
        self.initialized = False
        
        # Costmap and navigation state
        self.costmap = None
        self.is_aligned = False
        self.obstacle_detected = False
        self.robot_stopped = False
        self.gps_paused = False  # True when GPS accuracy is too low
        self.planner_state = "ACTIVE"
        self.current_waypoint_name = ''  # Waypoint name for logging
        
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
        # GPS position from SBG
        self.gps_sub = self.create_subscription(
            SbgGpsPos,
            '/sbg/gps_pos',
            self.gps_callback,
            qos_best_effort
        )
        
        # Heading from Witmotion IMU (0-360 degrees, 0=North)
        self.heading_sub = self.create_subscription(
            Float64,
            '/witmotion_eular/yaw',
            self.heading_callback,
            qos_best_effort
        )
        
        # Waypoint goals as "lat,lon,type" string
        self.waypoint_sub = self.create_subscription(
            String,
            '/waypoints',
            self.waypoint_callback,
            qos_reliable
        )
        
        # ZED visual odometry for local obstacle detection
        self.zed_odom_sub = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.zed_odom_callback,
            qos_best_effort
        )
        
        # Costmap for obstacle detection
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/costmap',
            self.costmap_callback,
            qos_reliable
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Visualization publishers
        self.robot_boundary_pub = self.create_publisher(
            PointCloud2,
            '/costmap_global/robot_boundary_cloud',
            10
        )
        
        self.path_marker_pub = self.create_publisher(
            Marker,
            '/costmap_global/planned_path_marker',
            10
        )
        
        self.goal_marker_pub = self.create_publisher(
            Marker,
            '/costmap_global/goal_marker',
            10
        )
        
        self.obstacle_check_pub = self.create_publisher(
            Marker,
            '/costmap_global/obstacle_check_zone',
            10
        )
        
        # Planner coordination
        self.local_planner_trigger_pub = self.create_publisher(
            Bool,
            '/costmap_local_planner_trigger',
            10
        )
        
        self.planner_state_pub = self.create_publisher(
            String,
            '/costmap_planner_state',
            10
        )
        
        self.goal_reached_pub = self.create_publisher(
            Bool,
            '/goal_reached',
            10
        )
        
        # Mission manager publisher for "reached" signal
        self.mission_manager_pub = self.create_publisher(
            String,
            '/mission_manager',
            10
        )
        
        # Goal bearing publisher for local planner (0-360 degrees, 0=North)
        self.goal_bearing_pub = self.create_publisher(
            Float64,
            '/goal_bearing',
            10
        )
        
        # Listen to state changes
        self.create_subscription(
            String,
            '/costmap_planner_state',
            self.planner_state_callback,
            10
        )
        
        # Timers
        self.create_timer(0.033, self.control_loop)  # 30Hz
        self.create_timer(0.2, self.publish_visualizations)
        self.create_timer(0.5, self.print_movement_instruction)
        
        self.get_logger().info('GPS-Based Global Planner initialized')
        self.get_logger().info('Using GPS (haversine) for distance, compass for bearing')
        self.get_logger().info('Using ZED odom for obstacle detection only')
        self.get_logger().info('Waiting for GPS fix and heading...')
    
    # ==================== GPS UTILITY METHODS ====================
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """
        Calculate the great-circle distance between two GPS points.
        Returns distance in meters.
        """
        R = 6371000.0  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat / 2.0) ** 2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2.0) ** 2)
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        
        return R * c
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate the bearing from point 1 to point 2.
        Returns bearing in degrees (0-360, 0=North, 90=East, clockwise).
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)
        
        x = math.sin(dlon) * math.cos(lat2_rad)
        y = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon))
        
        bearing_rad = math.atan2(x, y)
        bearing_deg = math.degrees(bearing_rad)
        
        # Normalize to 0-360
        return (bearing_deg + 360.0) % 360.0
    
    def normalize_angle_degrees(self, angle):
        """
        Normalize angle difference to [-180, 180] degrees.
        Handles wrap-around correctly (e.g., 355 to 5 = +10, not -350).
        """
        return ((angle + 180.0) % 360.0) - 180.0
    
    # ==================== CALLBACKS ====================
    
    def gps_callback(self, msg: SbgGpsPos):
        """Receive GPS position from SBG"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.gps_accuracy = msg.position_accuracy.x  # Horizontal accuracy
        
        was_valid = self.gps_valid
        self.gps_valid = (self.gps_accuracy <= self.gps_accuracy_threshold)
        
        if not self.gps_received:
            self.gps_received = True
            self.get_logger().info(f'GPS fix received: lat={self.current_lat:.7f}, lon={self.current_lon:.7f}, accuracy={self.gps_accuracy:.2f}m')
            self.check_initialization()
        
        # Handle GPS accuracy state changes
        if was_valid and not self.gps_valid:
            # GPS accuracy degraded - pause navigation
            self.gps_paused = True
            self.get_logger().warn(f'GPS accuracy degraded ({self.gps_accuracy:.2f}m > {self.gps_accuracy_threshold}m) - pausing navigation')
            self.stop_robot()
        elif not was_valid and self.gps_valid:
            # GPS accuracy recovered
            self.gps_paused = False
            self.get_logger().info(f'GPS accuracy recovered ({self.gps_accuracy:.2f}m)')
            # If there's a pending waypoint, log that navigation is starting
            if self.goal_lat is not None:
                distance = self.haversine_distance(self.current_lat, self.current_lon, self.goal_lat, self.goal_lon)
                bearing = self.calculate_bearing(self.current_lat, self.current_lon, self.goal_lat, self.goal_lon)
                self.get_logger().info(f'Starting navigation to waypoint - Distance: {distance:.1f}m, Bearing: {bearing:.1f}°')
    
    def heading_callback(self, msg: Float64):
        """Receive heading from Witmotion IMU (0-360 degrees, 0=North)"""
        self.current_heading = msg.data
        
        if not self.heading_received:
            self.heading_received = True
            self.get_logger().info(f'Heading received: {self.current_heading:.1f}°')
            self.check_initialization()
    
    def waypoint_callback(self, msg: String):
        """Receive waypoint as 'lat,lon,type' string. Waypoints are always stored and navigation starts when GPS is valid."""
        if not self.initialized:
            self.get_logger().warn('Cannot accept waypoint - waiting for GPS/heading initialization')
            return
        
        # Parse waypoint string: "lat,lon,type"
        try:
            parts = msg.data.strip().split(',')
            if len(parts) < 2:
                self.get_logger().error(f'Invalid waypoint format: "{msg.data}" - expected "lat,lon" or "lat,lon,type"')
                return
            
            lat = float(parts[0].strip())
            lon = float(parts[1].strip())
            waypoint_type = parts[2].strip() if len(parts) >= 3 else 'gnss'
            
            # Validate lat/lon ranges
            if not (-90.0 <= lat <= 90.0):
                self.get_logger().error(f'Invalid latitude: {lat} - must be in range [-90, 90]')
                return
            if not (-180.0 <= lon <= 180.0):
                self.get_logger().error(f'Invalid longitude: {lon} - must be in range [-180, 180]')
                return
            
            # Store valid goal (always accept, navigation will wait for GPS accuracy)
            self.goal_lat = lat
            self.goal_lon = lon
            self.waypoint_type = waypoint_type
            self.current_waypoint_name = f'{waypoint_type}_{lat:.5f}_{lon:.5f}'
            
            # Reset navigation state
            self.is_aligned = False
            self.obstacle_detected = False
            self.robot_stopped = False
            self.planner_state = "ACTIVE"
            self.last_instruction_time = None
            
            self.get_logger().info(f'New waypoint stored: lat={lat:.7f}, lon={lon:.7f}, type={waypoint_type}')
            
            # Calculate and log distance/bearing if GPS is currently valid
            if self.gps_valid and self.current_lat is not None:
                distance = self.haversine_distance(self.current_lat, self.current_lon, self.goal_lat, self.goal_lon)
                bearing = self.calculate_bearing(self.current_lat, self.current_lon, self.goal_lat, self.goal_lon)
                self.get_logger().info(f'Distance: {distance:.1f}m, Bearing: {bearing:.1f}°, Current heading: {self.current_heading:.1f}°')
            else:
                self.get_logger().warn(f'GPS accuracy too low ({self.gps_accuracy:.2f}m) - navigation will start when accuracy improves')
            
        except ValueError as e:
            self.get_logger().error(f'Failed to parse waypoint "{msg.data}": {e}')
    
    def zed_odom_callback(self, msg: Odometry):
        """Receive ZED visual odometry for obstacle detection"""
        self.local_pose = msg.pose.pose
        
        if not self.local_pose_valid:
            self.local_pose_valid = True
            self.get_logger().info('ZED odometry received - obstacle detection enabled')
    
    def check_initialization(self):
        """Check if all required data sources are available"""
        if self.gps_received and self.heading_received and self.gps_valid:
            if not self.initialized:
                self.initialized = True
                self.get_logger().info('Initialization complete - ready to accept waypoints')
        elif self.gps_received and self.heading_received and not self.gps_valid:
            self.get_logger().warn(f'GPS received but accuracy too low ({self.gps_accuracy:.2f}m) - waiting for better fix')
    
    def planner_state_callback(self, msg: String):
        """Listen to planner state changes"""
        if msg.data == "local_active":
            self.planner_state = "IDLE"
        elif msg.data == "global_active":
            self.planner_state = "ACTIVE"
            self.obstacle_detected = False
            self.robot_stopped = False  # Resume robot movement
            self.get_logger().info('Global planner resuming control')
    
    def odom_callback(self, msg: Odometry):
        """Update current robot pose"""
        if self.current_pose is None:
            self.get_logger().info(f'Odometry received: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
        self.current_pose = msg.pose.pose
    
    def costmap_callback(self, msg: OccupancyGrid):
        """Receive costmap updates - ONLY source of obstacle info"""
        self.costmap = msg
    
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle (radians)"""
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def get_distance_to_goal(self):
        """Calculate haversine distance to goal in meters"""
        if self.current_lat is None or self.goal_lat is None:
            return float('inf')
        
        return self.haversine_distance(self.current_lat, self.current_lon, 
                                        self.goal_lat, self.goal_lon)
    
    def get_angle_to_goal(self):
        """
        Calculate angle error to goal in degrees.
        Returns value in [-180, 180] where positive = turn left, negative = turn right.
        """
        if self.current_lat is None or self.goal_lat is None or self.current_heading is None:
            return 0.0
        
        # Calculate bearing to goal (0-360, 0=North, 90=East)
        target_bearing = self.calculate_bearing(self.current_lat, self.current_lon,
                                                 self.goal_lat, self.goal_lon)
        
        # Calculate angle difference and normalize to [-180, 180]
        angle_diff = self.normalize_angle_degrees(target_bearing - self.current_heading)
        
        return angle_diff
    
    def check_obstacles_in_costmap(self):
        """
        Check obstacles along bearing direction to goal using costmap data.
        Uses ZED visual odometry for local position and bearing angle for sweep direction.
        Creates swept circle check along the bearing direction within costmap bounds.
        """
        # Check prerequisites
        if self.costmap is None:
            return False
        
        if not self.local_pose_valid or self.local_pose is None:
            # Cannot check obstacles without ZED odom
            return False
        
        if self.goal_lat is None or self.current_lat is None or self.current_heading is None:
            return False
        
        # Costmap info
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        height = self.costmap.info.height
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        
        # Robot position from ZED visual odometry (local frame)
        robot_x = self.local_pose.position.x
        robot_y = self.local_pose.position.y
        
        # Calculate bearing to goal (GPS-based, 0-360 degrees, 0=North)
        target_bearing = self.calculate_bearing(self.current_lat, self.current_lon,
                                                 self.goal_lat, self.goal_lon)
        
        # Convert bearing to local frame direction
        # Bearing: 0=North, 90=East (clockwise from North)
        # Local frame (ZED/odom): 0=Forward(X+), 90=Left(Y+) (counter-clockwise from X)
        # Angle difference: bearing - heading gives the local-relative angle
        angle_diff_deg = self.normalize_angle_degrees(target_bearing - self.current_heading)
        
        # Convert to local frame angle (radians)
        # In local frame: 0 degrees = forward (X+), positive = counter-clockwise
        # angle_diff_deg: positive = turn left, negative = turn right
        local_angle_rad = math.radians(angle_diff_deg)
        
        # Direction vector in local frame
        dx_norm = math.cos(local_angle_rad)
        dy_norm = math.sin(local_angle_rad)
        
        # Check distance (use obstacle_check_distance as we don't have local goal distance)
        check_distance = self.obstacle_check_distance
        
        # Robot boundary radius for swept circle check
        boundary_radius = self.robot_radius + self.safety_margin
        
        # Sample along path with finer resolution for accuracy
        step_size = resolution
        num_samples = int(check_distance / step_size) + 1
        
        for i in range(num_samples):
            t = i * step_size
            
            # Skip if beyond check distance
            if t > check_distance:
                break
                
            line_x = robot_x + dx_norm * t
            line_y = robot_y + dy_norm * t
            
            # Check circle around each point
            num_circle_samples = 16
            for angle_idx in range(num_circle_samples):
                angle = 2.0 * math.pi * angle_idx / num_circle_samples
                
                check_x = line_x + boundary_radius * math.cos(angle)
                check_y = line_y + boundary_radius * math.sin(angle)
                
                # Convert to grid coordinates
                grid_x = int((check_x - origin_x) / resolution)
                grid_y = int((check_y - origin_y) / resolution)
                
                # Check bounds and obstacle
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    index = grid_y * width + grid_x
                    if index < len(self.costmap.data):
                        cost = self.costmap.data[index]
                        if cost > self.costmap_threshold:
                            # Found obstacle - report world coordinates and distance
                            self.get_logger().info(
                                f'Obstacle at path_dist={t:.2f}m, local=({check_x:.2f},{check_y:.2f}), '
                                f'robot=({robot_x:.2f},{robot_y:.2f}), bearing={target_bearing:.1f}°, cost={cost}'
                            )
                            return True
        
        return False
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def handle_waypoint_reached(self):
        """
        Handle waypoint completion.
        For GPS/GNSS waypoints, sends 'reached' to /mission_manager topic.
        """
        waypoint_name = self.current_waypoint_name
        waypoint_type = self.waypoint_type or 'gnss'
        
        self.get_logger().info(f'Waypoint reached: "{waypoint_name}" (type: {waypoint_type})')
        
        # Send "reached" signal to mission manager
        reached_msg = String()
        reached_msg.data = 'reached'
        self.mission_manager_pub.publish(reached_msg)
        self.get_logger().info('Sent "reached" to /mission_manager')
        
        # Also publish goal reached for compatibility
        goal_msg = Bool()
        goal_msg.data = True
        self.goal_reached_pub.publish(goal_msg)
        
        # Reset waypoint state
        self.current_waypoint_name = ''
        self.waypoint_type = None
    
    def print_movement_instruction(self):
        """Print movement instructions every 0.5 seconds"""
        # Check prerequisites for GPS-based navigation
        if self.current_lat is None or self.goal_lat is None or self.current_heading is None:
            return
        
        if self.planner_state != "ACTIVE":
            return
        
        if self.gps_paused:
            return
        
        # Check if enough time has passed since last instruction
        current_time = self.get_clock().now()
        if self.last_instruction_time is not None:
            elapsed = (current_time - self.last_instruction_time).nanoseconds / 1e9
            if elapsed < 0.4:  # Slightly less than 0.5 to account for timing
                return
        
        distance = self.get_distance_to_goal()
        angle_to_goal = self.get_angle_to_goal()  # Already in degrees
        
        if distance < self.goal_tolerance:
            return
        
        # Determine current action (angle_tolerance is in degrees)
        if not self.is_aligned:
            if abs(angle_to_goal) > self.angular_tolerance:
                direction = "LEFT" if angle_to_goal > 0 else "RIGHT"
                self.get_logger().info(f'Rotate {direction} {abs(angle_to_goal):.1f}°')
                self.last_instruction_time = current_time
            else:
                self.get_logger().info(f'Aligned to goal, switching to forward')
                self.last_instruction_time = current_time
        else:
            if abs(angle_to_goal) > self.angular_tolerance * 2:
                self.get_logger().info(f'Re-aligning needed')
                self.last_instruction_time = current_time
            else:
                self.get_logger().info(f'Move forward {distance:.2f}m to goal')
                self.last_instruction_time = current_time
    
    def control_loop(self):
        """Main control loop - GPS-based navigation"""
        # Check prerequisites for GPS navigation
        if self.current_lat is None or self.goal_lat is None or self.current_heading is None:
            return
        
        # Only execute if global planner is active
        if self.planner_state != "ACTIVE":
            return
        
        # Check GPS validity - pause if accuracy is too low
        if not self.gps_valid or self.gps_paused:
            return
        
        # Check if goal reached (1 meter threshold)
        distance = self.get_distance_to_goal()
        if distance < self.goal_tolerance:
            self.get_logger().info(f'Goal reached! Distance: {distance:.2f}m')
            self.stop_robot()
            
            # Handle waypoint completion
            self.handle_waypoint_reached()
            
            # Clear goal
            self.goal_lat = None
            self.goal_lon = None
            return
        
        # Check for obstacles in path (using costmap and ZED odom)
        obstacle_in_path = self.check_obstacles_in_costmap()
        
        if obstacle_in_path and not self.obstacle_detected:
            self.get_logger().info('Obstacle detected in costmap - triggering local planner')
            self.obstacle_detected = True
            self.robot_stopped = True
            self.stop_robot()
            
            # Trigger local planner
            trigger_msg = Bool()
            trigger_msg.data = True
            self.local_planner_trigger_pub.publish(trigger_msg)
            
            # Update state
            state_msg = String()
            state_msg.data = "local_active"
            self.planner_state_pub.publish(state_msg)
            
            return
        
        if self.robot_stopped:
            return
        
        # Navigate to goal using bearing angle
        angle_to_goal = self.get_angle_to_goal()  # In degrees, [-180, 180]
        
        # Publish bearing for local planner (0-360 degrees, 0=North)
        bearing = self.calculate_bearing(
            self.current_lat, self.current_lon,
            self.goal_lat, self.goal_lon
        )
        bearing_msg = Float64()
        bearing_msg.data = bearing
        self.goal_bearing_pub.publish(bearing_msg)
        
        cmd = Twist()
        
        # Check if alignment is needed (5 degree threshold)
        if abs(angle_to_goal) > self.angular_tolerance:
            # Need to rotate
            self.is_aligned = False
            # Positive angle = turn left, negative = turn right
            cmd.angular.z = self.angular_vel if angle_to_goal > 0 else -self.angular_vel
            cmd.linear.x = 0.0
        else:
            # Aligned - move forward
            if not self.is_aligned:
                self.get_logger().info(f'Aligned to goal (heading error: {angle_to_goal:.1f}°)')
                self.is_aligned = True
            cmd.linear.x = self.linear_vel
            # Proportional heading correction (convert degrees to appropriate scale)
            cmd.angular.z = 0.05 * angle_to_goal  # Small correction factor for degrees
        
        self.cmd_vel_pub.publish(cmd)
    
    def publish_visualizations(self):
        """Publish visualization markers"""
        self.publish_robot_boundary()
        if self.goal_lat is not None and self.local_pose_valid:
            self.publish_obstacle_check_zone()
    
    def publish_robot_boundary(self):
        """Publish point cloud circle around robot boundary using ZED local pose"""
        if not self.local_pose_valid or self.local_pose is None:
            return
        
        try:
            # Generate circle of points at robot boundary radius
            num_points = 360  # Dense circle (every 1 degree)
            radius = self.robot_radius + self.safety_margin
            
            # Get robot's current position from ZED odom
            robot_x = self.local_pose.position.x
            robot_y = self.local_pose.position.y
            
            # Generate circle points in odom frame
            points = []
            for i in range(num_points):
                angle = 2.0 * math.pi * i / num_points
                x = robot_x + radius * math.cos(angle)
                y = robot_y + radius * math.sin(angle)
                z = 0.0
                points.append([x, y, z])
            
            # Create PointCloud2 message
            cloud_msg = PointCloud2()
            cloud_msg.header.frame_id = "odom"
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
        except Exception as e:
            self.get_logger().error(f'Failed to publish boundary cloud: {e}')
    
    def publish_obstacle_check_zone(self):
        """Publish obstacle detection zone visualization along bearing direction"""
        if not self.local_pose_valid or self.local_pose is None:
            return
        
        if self.goal_lat is None or self.current_lat is None or self.current_heading is None:
            return
        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "costmap_obstacle_check"
        marker.id = 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Calculate bearing to goal
        target_bearing = self.calculate_bearing(self.current_lat, self.current_lon,
                                                 self.goal_lat, self.goal_lon)
        
        # Convert bearing to local frame angle
        angle_diff_deg = self.normalize_angle_degrees(target_bearing - self.current_heading)
        local_angle_rad = math.radians(angle_diff_deg)
        
        # Arrow from robot position in bearing direction
        marker.pose.position.x = self.local_pose.position.x
        marker.pose.position.y = self.local_pose.position.y
        marker.pose.position.z = 0.15
        
        # Orientation in local frame
        marker.pose.orientation.z = math.sin(local_angle_rad / 2.0)
        marker.pose.orientation.w = math.cos(local_angle_rad / 2.0)
        
        # Arrow size (length = obstacle_check_distance)
        marker.scale.x = self.obstacle_check_distance  # Length
        marker.scale.y = 0.08  # Width
        marker.scale.z = 0.08  # Height
        
        # Color: Red if obstacle detected, cyan if clear
        if self.obstacle_detected:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.5
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        self.obstacle_check_pub.publish(marker)
    

def main(args=None):
    rclpy.init(args=args)
    node = CostmapGlobalPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
