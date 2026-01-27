#!/usr/bin/env python3
"""
Mission Manager Node
Manages sequential waypoint navigation by receiving waypoints from /waypoints topic,
sending them one-by-one to the global planner, and waiting for confirmation before
proceeding to the next waypoint.

Waypoint names are extracted from frame_id of each pose in the Path message.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String


class MissionManagerNode(Node):
    """
    Mission Manager Node for sequential waypoint navigation.
    
    Subscribes to:
        /waypoints (nav_msgs/Path): List of waypoints with names in frame_id
        /waypoint_reached (std_msgs/Bool): Confirmation from global planner
    
    Publishes to:
        /goal_pose (geometry_msgs/PoseStamped): Current waypoint goal
        /current_waypoint_name (std_msgs/String): Name of current waypoint
        /mission_status (std_msgs/String): Current mission status
    """
    
    def __init__(self):
        super().__init__('mission_manager_node')
        
        # State variables
        self.waypoints = []  # List of (PoseStamped, waypoint_name) tuples
        self.current_waypoint_index = 0
        self.mission_active = False
        self.waiting_for_confirmation = False
        
        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers
        self.waypoints_sub = self.create_subscription(
            Path,
            '/waypoints',
            self.waypoints_callback,
            qos_reliable
        )
        
        self.waypoint_reached_sub = self.create_subscription(
            Bool,
            '/waypoint_reached',
            self.waypoint_reached_callback,
            qos_reliable
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            qos_reliable
        )
        
        self.current_waypoint_name_pub = self.create_publisher(
            String,
            '/current_waypoint_name',
            qos_reliable
        )
        
        self.mission_status_pub = self.create_publisher(
            String,
            '/mission_status',
            qos_reliable
        )
        
        # Status update timer
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Mission Manager Node initialized')
        self.get_logger().info('Waiting for waypoints on /waypoints topic...')
        self.publish_mission_status('IDLE')
    
    def waypoints_callback(self, msg: Path):
        """
        Receive waypoints from /waypoints topic.
        Each pose's frame_id contains the waypoint name.
        
        Note: Applies coordinate inversion (negate X and Y) to convert from
        Unity coordinate system to ROS coordinate system.
        """
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty waypoints message')
            return
        
        # Reset mission state
        self.waypoints = []
        self.current_waypoint_index = 0
        self.mission_active = False
        self.waiting_for_confirmation = False
        
        # Extract waypoints and their names
        for pose_stamped in msg.poses:
            waypoint_name = pose_stamped.header.frame_id
            if not waypoint_name:
                waypoint_name = f'waypoint_{len(self.waypoints)}'
            
            # Apply coordinate inversion for Unity-to-ROS conversion
            # Negate X and Y to correct for coordinate system difference
            tempx = pose_stamped.pose.position.x
            tempy = pose_stamped.pose.position.y

            pose_stamped.pose.position.x = tempy
            pose_stamped.pose.position.y = -tempx
            
            self.waypoints.append((pose_stamped, waypoint_name))
            self.get_logger().info(
                f'Waypoint [{len(self.waypoints)}]: {waypoint_name} '
                f'at ({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f})'
            )
        
        self.get_logger().info(f'Received {len(self.waypoints)} waypoints')
        
        # Start mission
        self.mission_active = True
        self.send_current_waypoint()
    
    def waypoint_reached_callback(self, msg: Bool):
        """
        Receive confirmation that current waypoint has been reached.
        """
        if not self.mission_active or not self.waiting_for_confirmation:
            return
        
        if msg.data:
            current_name = self.waypoints[self.current_waypoint_index][1]
            self.get_logger().info(f'Waypoint "{current_name}" reached!')
            
            self.waiting_for_confirmation = False
            self.current_waypoint_index += 1
            
            # Check if more waypoints remain
            if self.current_waypoint_index < len(self.waypoints):
                self.send_current_waypoint()
            else:
                self.mission_complete()
    
    def send_current_waypoint(self):
        """
        Send the current waypoint to the global planner.
        """
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().warn('No more waypoints to send')
            return
        
        pose_stamped, waypoint_name = self.waypoints[self.current_waypoint_index]
        
        # Create goal pose message
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = waypoint_name  # Include waypoint name in frame_id
        goal_msg.pose = pose_stamped.pose
        
        # Publish goal
        self.goal_pub.publish(goal_msg)
        
        # Publish current waypoint name
        name_msg = String()
        name_msg.data = waypoint_name
        self.current_waypoint_name_pub.publish(name_msg)
        
        self.waiting_for_confirmation = True
        
        self.get_logger().info(
            f'Sending waypoint [{self.current_waypoint_index + 1}/{len(self.waypoints)}]: '
            f'"{waypoint_name}" at ({pose_stamped.pose.position.x:.2f}, '
            f'{pose_stamped.pose.position.y:.2f})'
        )
        
        self.publish_mission_status('NAVIGATING')
    
    def mission_complete(self):
        """
        Handle mission completion - reset and wait for new waypoints.
        """
        self.get_logger().info('Mission complete! All waypoints reached.')
        self.get_logger().info('Resetting and waiting for new waypoints...')
        
        self.publish_mission_status('COMPLETE')
        
        # Reset state
        self.waypoints = []
        self.current_waypoint_index = 0
        self.mission_active = False
        self.waiting_for_confirmation = False
        
        self.publish_mission_status('IDLE')
    
    def publish_status(self):
        """
        Publish periodic status updates.
        """
        if not self.mission_active:
            return
        
        if self.current_waypoint_index < len(self.waypoints):
            waypoint_name = self.waypoints[self.current_waypoint_index][1]
            self.get_logger().debug(
                f'Status: Waypoint [{self.current_waypoint_index + 1}/{len(self.waypoints)}] '
                f'"{waypoint_name}" - Waiting: {self.waiting_for_confirmation}'
            )
    
    def publish_mission_status(self, status: str):
        """
        Publish mission status.
        """
        msg = String()
        msg.data = status
        self.mission_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Mission Manager shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
