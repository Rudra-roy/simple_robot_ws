#!/usr/bin/env python3

import numpy as np
import rclpy
import rclpy.node
import tf2_ros
from geometry_msgs.msg import TwistStamped, PointStamped, Point, PoseStamped, Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from std_srvs.srv import Empty
import math

# Simple service definition for compute velocity commands
class ComputeVelocityCommandsRequest:
    def __init__(self):
        self.pose = PoseStamped()
        self.velocity = Twist()
        self.path = Path()

class ComputeVelocityCommandsResponse:
    def __init__(self):
        self.cmd_vel = TwistStamped()

class ComputeVelocityCommands:
    Request = ComputeVelocityCommandsRequest
    Response = ComputeVelocityCommandsResponse


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def closest_point_on_segment(p, v1, v2):
    """Find closest point on line segment v1-v2 to point p"""
    v1_to_p = p - v1
    v1_to_v2 = v2 - v1
    
    if np.dot(v1_to_v2, v1_to_v2) < 1e-6:  # Segment is too short
        return v1
    
    t = np.clip(np.dot(v1_to_p, v1_to_v2) / np.dot(v1_to_v2, v1_to_v2), 0.0, 1.0)
    return v1 + t * v1_to_v2


def smoothstep(edge0, edge1, x):
    """Smooth step function for gradual transitions"""
    x = np.clip((x - edge0) / (edge1 - edge0), 0, 1)
    return x * x * (3 - 2 * x)


class PathFollowerNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("path_follower")
        
        # Declare parameters
        self.declare_parameter("robot_frame", "camera_link")
        self.declare_parameter("lookahead", 0.5)
        self.declare_parameter("approach_distance", 1.5)
        self.declare_parameter("approach_linear_velocity", 0.2)
        self.declare_parameter("slow_linear_velocity", 0.5)
        self.declare_parameter("fast_linear_velocity", 1.0)
        self.declare_parameter("angular_velocity", 0.8)
        self.declare_parameter("rotate_in_place_start_angle", 1.4)
        self.declare_parameter("rotate_in_place_end_angle", 0.3)
        self.declare_parameter("regression_distance", 2.0)
        self.declare_parameter("driving_mode", "forward")
        self.declare_parameter("publish_target", True)
        
        # Get parameters
        self.robot_frame = self.get_parameter("robot_frame").value
        self.lookahead = self.get_parameter("lookahead").value
        self.approach_distance = self.get_parameter("approach_distance").value
        self.approach_linear_velocity = self.get_parameter("approach_linear_velocity").value
        self.slow_linear_velocity = self.get_parameter("slow_linear_velocity").value
        self.fast_linear_velocity = self.get_parameter("fast_linear_velocity").value
        self.angular_velocity = self.get_parameter("angular_velocity").value
        self.rotate_in_place_start_angle = self.get_parameter("rotate_in_place_start_angle").value
        self.rotate_in_place_end_angle = self.get_parameter("rotate_in_place_end_angle").value
        self.regression_distance = self.get_parameter("regression_distance").value
        self.driving_mode = self.get_parameter("driving_mode").value
        self.publish_target = self.get_parameter("publish_target").value
        
        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Service server for velocity commands
        self.service = self.create_service(
            ComputeVelocityCommands,
            "compute_velocity_commands",
            self.compute_velocity_commands_callback
        )
        
        # Publishers for visualization
        if self.publish_target:
            self.target_pub = self.create_publisher(PointStamped, "target_point", 10)
            self.marker_pub = self.create_publisher(Marker, "lookahead_marker", 10)
        
        # State variables
        self.rotating_in_place = False
        
        self.get_logger().info("Path follower node initialized")
    
    def compute_velocity_commands_callback(self, request, response):
        """Service callback to compute velocity commands from path"""
        try:
            # Extract data from request
            current_pose = request.pose
            current_velocity = request.velocity
            path = request.path
            
            if len(path.poses) < 2:
                self.get_logger().warn("Path too short for following")
                response.cmd_vel.twist.linear.x = 0.0
                response.cmd_vel.twist.angular.z = 0.0
                return response
            
            # Transform path to robot frame if needed
            robot_position = np.array([current_pose.pose.position.x,
                                     current_pose.pose.position.y])
            
            # Convert path to numpy array
            path_points = []
            for pose in path.poses:
                path_points.append([pose.pose.position.x, pose.pose.position.y])
            path_points = np.array(path_points)
            
            # Find lookahead point
            target_point, target_index = self.find_lookahead_point(robot_position, path_points)
            
            # Calculate distance to goal
            goal_position = path_points[-1]
            distance_to_goal = np.linalg.norm(goal_position - robot_position)
            
            # Get robot orientation
            robot_orientation = current_pose.pose.orientation
            robot_yaw = math.atan2(
                2 * (robot_orientation.w * robot_orientation.z + robot_orientation.x * robot_orientation.y),
                1 - 2 * (robot_orientation.y**2 + robot_orientation.z**2)
            )
            
            # Calculate target heading
            target_direction = target_point - robot_position
            target_heading = math.atan2(target_direction[1], target_direction[0])
            
            # Calculate heading error
            heading_error = normalize_angle(target_heading - robot_yaw)
            
            # Determine if we should rotate in place
            if not self.rotating_in_place and abs(heading_error) > self.rotate_in_place_start_angle:
                self.rotating_in_place = True
            elif self.rotating_in_place and abs(heading_error) < self.rotate_in_place_end_angle:
                self.rotating_in_place = False
            
            # Compute velocities
            if self.rotating_in_place:
                linear_vel = 0.0
                angular_vel = self.angular_velocity * np.sign(heading_error)
            else:
                # Determine linear velocity based on distance to goal and path curvature
                if distance_to_goal < self.approach_distance:
                    linear_vel = self.approach_linear_velocity
                else:
                    # Check path curvature ahead
                    is_straight = self.analyze_path_straightness(path_points, target_index)
                    if is_straight:
                        linear_vel = self.fast_linear_velocity
                    else:
                        linear_vel = self.slow_linear_velocity
                
                # Angular velocity with smooth control
                angular_vel = self.calculate_angular_velocity(heading_error)
            
            # Create response
            response.cmd_vel = TwistStamped()
            response.cmd_vel.header.stamp = self.get_clock().now().to_msg()
            response.cmd_vel.header.frame_id = self.robot_frame
            response.cmd_vel.twist.linear.x = linear_vel
            response.cmd_vel.twist.angular.z = angular_vel
            
            # Publish visualization
            if self.publish_target:
                self.publish_target_visualization(target_point, current_pose.header)
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error computing velocity commands: {str(e)}")
            response.cmd_vel = TwistStamped()
            response.cmd_vel.header.stamp = self.get_clock().now().to_msg()
            response.cmd_vel.header.frame_id = self.robot_frame
            return response
    
    def find_lookahead_point(self, robot_position, path_points):
        """Find the lookahead point on the path"""
        min_distance = float('inf')
        closest_index = 0
        
        # Find closest point on path
        for i in range(len(path_points) - 1):
            closest_point = closest_point_on_segment(
                robot_position, path_points[i], path_points[i + 1]
            )
            distance = np.linalg.norm(closest_point - robot_position)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        # Find lookahead point
        accumulated_distance = 0
        lookahead_index = closest_index
        
        for i in range(closest_index, len(path_points) - 1):
            segment_length = np.linalg.norm(path_points[i + 1] - path_points[i])
            if accumulated_distance + segment_length >= self.lookahead:
                # Interpolate along this segment
                remaining_distance = self.lookahead - accumulated_distance
                t = remaining_distance / segment_length
                lookahead_point = path_points[i] + t * (path_points[i + 1] - path_points[i])
                lookahead_index = i
                break
            accumulated_distance += segment_length
            lookahead_index = i + 1
        else:
            # Use the goal point if we can't find a lookahead point
            lookahead_point = path_points[-1]
            lookahead_index = len(path_points) - 1
        
        return lookahead_point, lookahead_index
    
    def analyze_path_straightness(self, path_points, start_index):
        """Analyze if the path ahead is straight"""
        if start_index >= len(path_points) - 2:
            return True  # Near end, consider straight
        
        # Look ahead up to regression_distance
        accumulated_distance = 0
        analysis_points = [path_points[start_index]]
        
        for i in range(start_index, len(path_points) - 1):
            segment_length = np.linalg.norm(path_points[i + 1] - path_points[i])
            if accumulated_distance + segment_length >= self.regression_distance:
                # Add interpolated point at regression distance
                remaining_distance = self.regression_distance - accumulated_distance
                t = remaining_distance / segment_length
                end_point = path_points[i] + t * (path_points[i + 1] - path_points[i])
                analysis_points.append(end_point)
                break
            analysis_points.append(path_points[i + 1])
            accumulated_distance += segment_length
        
        if len(analysis_points) < 3:
            return True
        
        # Simple curvature analysis - check if points deviate significantly from straight line
        start_point = analysis_points[0]
        end_point = analysis_points[-1]
        line_vector = end_point - start_point
        line_length = np.linalg.norm(line_vector)
        
        if line_length < 0.1:  # Too short to analyze
            return True
        
        max_deviation = 0
        for point in analysis_points[1:-1]:
            # Distance from point to line
            point_vector = point - start_point
            projection_length = np.dot(point_vector, line_vector) / line_length
            projection_point = start_point + (projection_length / line_length) * line_vector
            deviation = np.linalg.norm(point - projection_point)
            max_deviation = max(max_deviation, deviation)
        
        # Consider straight if deviation is less than 20cm
        return max_deviation < 0.2
    
    def calculate_angular_velocity(self, heading_error):
        """Calculate angular velocity with smooth control"""
        # Use smoothstep for gradual angular velocity changes
        max_error = math.pi / 4  # 45 degrees
        normalized_error = min(abs(heading_error), max_error) / max_error
        
        # Apply smoothstep for smooth acceleration
        smooth_factor = smoothstep(0, 1, normalized_error)
        angular_vel = smooth_factor * self.angular_velocity * np.sign(heading_error)
        
        return angular_vel
    
    def publish_target_visualization(self, target_point, header):
        """Publish target point for visualization"""
        # Publish target point
        point_msg = PointStamped()
        point_msg.header = header
        point_msg.point.x = target_point[0]
        point_msg.point.y = target_point[1]
        point_msg.point.z = 0.0
        self.target_pub.publish(point_msg)
        
        # Publish marker
        marker = Marker()
        marker.header = header
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target_point[0]
        marker.pose.position.y = target_point[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.id = 0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
