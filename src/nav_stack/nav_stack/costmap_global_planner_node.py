#!/usr/bin/env python3
"""
Costmap-Only Global Planner Node
Plans direct path to goal using only local costmap data.
No dependency on static /map - works in unknown environments.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker
import math


class CostmapGlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('costmap_global_planner_node')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.6)
        self.declare_parameter('safety_margin', 0.2)
        self.declare_parameter('linear_velocity', 1.2)
        self.declare_parameter('angular_velocity', 4.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('angular_tolerance', 0.15)
        self.declare_parameter('obstacle_check_distance', 4.0)
        self.declare_parameter('costmap_obstacle_threshold', 70)
        
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
        self.planner_state = "ACTIVE"
        self.last_instruction_time = None
        
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
            '/odom',
            self.odom_callback,
            qos_best_effort
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
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
        
        # Listen to state changes
        self.create_subscription(
            String,
            '/costmap_planner_state',
            self.planner_state_callback,
            10
        )
        
        # Timers
        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.2, self.publish_visualizations)
        self.create_timer(0.5, self.print_movement_instruction)
        
        self.get_logger().info('Costmap-Only Global Planner initialized')
        self.get_logger().info('No /map dependency - using /costmap only')
    
    def goal_callback(self, msg: PoseStamped):
        """Receive new goal pose"""
        self.goal_pose = msg
        self.is_aligned = False
        self.obstacle_detected = False
        self.robot_stopped = False
        self.planner_state = "ACTIVE"
        self.last_instruction_time = None
        
        self.get_logger().info(f'New goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        self.publish_goal_marker()
        
        # Print initial instruction immediately
        if self.current_pose is not None:
            distance = self.get_distance_to_goal()
            angle_to_goal = self.get_angle_to_goal()
            if abs(angle_to_goal) > self.angular_tolerance:
                direction = "LEFT" if angle_to_goal > 0 else "RIGHT"
                angle_deg = abs(math.degrees(angle_to_goal))
                self.get_logger().info(f'🔄 Rotate {direction} {angle_deg:.1f}° to align')
            else:
                self.get_logger().info(f'➡ Move forward {distance:.2f}m to goal')
    
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
        """Calculate angle to goal relative to robot's orientation"""
        if self.current_pose is None or self.goal_pose is None:
            return 0.0
        
        dx = self.goal_pose.pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.position.y
        
        goal_angle = math.atan2(dy, dx)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        angle_diff = self.normalize_angle(goal_angle - current_yaw)
        return angle_diff
    
    def check_obstacles_in_costmap(self):
        """
        Check obstacles along direct path to goal using ONLY costmap data.
        Creates bee-line swept circle check within costmap bounds.
        """
        if self.costmap is None or self.current_pose is None or self.goal_pose is None:
            return False
        
        # Costmap info
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        height = self.costmap.info.height
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        
        # Current and goal positions
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        
        # Direction to goal
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        
        if distance_to_goal < 0.01:
            return False
        
        # Normalize
        dx_norm = dx / distance_to_goal
        dy_norm = dy / distance_to_goal
        
        # CRITICAL FIX: Only check obstacles UP TO the goal distance, not beyond
        # This prevents triggering local planner for obstacles past the goal
        check_distance = distance_to_goal
        
        # Robot boundary radius (reduce to make path checking more forgiving)
        # Only check a narrow corridor to goal, not full robot radius
        boundary_radius = self.robot_radius * 0.7  # 70% of robot radius for more direct path
        
        # Sample along path
        num_samples = int(check_distance / resolution) + 1
        
        for i in range(num_samples):
            t = i * resolution
            
            # Skip if beyond goal distance
            if t > distance_to_goal:
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
                            # Found obstacle - check if it's close enough to matter
                            # Only trigger if obstacle is within 1.5m of current position
                            obstacle_dist = t  # Distance along path where obstacle was found
                            if obstacle_dist < 1.5:
                                return True
                            # Otherwise, obstacle is far away, keep going
        
        return False
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def print_movement_instruction(self):
        """Print movement instructions every 0.5 seconds"""
        if self.current_pose is None or self.goal_pose is None:
            return
        
        if self.planner_state != "ACTIVE":
            return
        
        # Check if enough time has passed since last instruction
        current_time = self.get_clock().now()
        if self.last_instruction_time is not None:
            elapsed = (current_time - self.last_instruction_time).nanoseconds / 1e9
            if elapsed < 0.4:  # Slightly less than 0.5 to account for timing
                return
        
        distance = self.get_distance_to_goal()
        angle_to_goal = self.get_angle_to_goal()
        
        if distance < self.goal_tolerance:
            return
        
        # Determine current action
        if not self.is_aligned:
            if abs(angle_to_goal) > self.angular_tolerance:
                direction = "LEFT" if angle_to_goal > 0 else "RIGHT"
                angle_deg = abs(math.degrees(angle_to_goal))
                self.get_logger().info(f'🔄 Rotate {direction} {angle_deg:.1f}°')
                self.last_instruction_time = current_time
            else:
                self.get_logger().info(f'✓ Aligned to goal, switching to forward')
                self.last_instruction_time = current_time
        else:
            if abs(angle_to_goal) > self.angular_tolerance * 2:
                self.get_logger().info(f'⚠ Re-aligning needed')
                self.last_instruction_time = current_time
            else:
                self.get_logger().info(f'➡ Move forward {distance:.2f}m to goal')
                self.last_instruction_time = current_time
    
    def control_loop(self):
        """Main control loop"""
        # Check prerequisites
        if self.current_pose is None or self.goal_pose is None:
            return
        
        # Only execute if global planner is active
        if self.planner_state != "ACTIVE":
            return
        
        # Check if goal reached
        distance = self.get_distance_to_goal()
        if distance < self.goal_tolerance:
            self.get_logger().info('Goal reached!')
            self.stop_robot()
            
            # Publish goal reached
            msg = Bool()
            msg.data = True
            self.goal_reached_pub.publish(msg)
            
            self.goal_pose = None
            return
        
        # Check for obstacles in path (using costmap only)
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
        
        # Navigate to goal
        angle_to_goal = self.get_angle_to_goal()
        
        cmd = Twist()
        
        # Check if alignment is needed
        if abs(angle_to_goal) > self.angular_tolerance:
            # Need to rotate
            self.is_aligned = False
            cmd.angular.z = self.angular_vel if angle_to_goal > 0 else -self.angular_vel
            cmd.linear.x = 0.0
        else:
            # Aligned - move forward
            if not self.is_aligned:
                self.get_logger().info('Aligned to goal')
                self.is_aligned = True
            cmd.linear.x = self.linear_vel
            cmd.angular.z = 0.3 * angle_to_goal  # Proportional correction
        
        self.cmd_vel_pub.publish(cmd)
    
    def publish_visualizations(self):
        """Publish visualization markers"""
        if self.goal_pose is not None:
            self.publish_goal_marker()
    
    def publish_goal_marker(self):
        """Publish goal marker"""
        if self.goal_pose is None:
            return
        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "costmap_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose = self.goal_pose.pose
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.goal_marker_pub.publish(marker)


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
