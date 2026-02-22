#!/usr/bin/env python3
"""
Mission Coordinator Node
Central mission orchestration - loads missions, sequences waypoints, triggers detection tasks, controls LED.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from detection_interfaces.msg import DetectionResult, MissionState, TaskCommand
import yaml
import time
import os


class MissionCoordinatorNode(Node):
    """Central mission coordinator with state machine and LED control."""
    
    def __init__(self):
        super().__init__('mission_coordinator_node')
        
        # Declare parameters
        self.declare_parameter('mission_file',  '')
        self.declare_parameter('goal_reached_delay', 10.0)
        self.declare_parameter('task_timeout', 120.0)
        self.declare_parameter('auto_start', False)
        
        # Get parameters
        mission_file = self.get_parameter('mission_file').value
        self.goal_reached_delay = self.get_parameter('goal_reached_delay').value
        self.task_timeout = self.get_parameter('task_timeout').value
        auto_start = self.get_parameter('auto_start').value
        
        # Publishers
        self.waypoint_pub = self.create_publisher(String, '/waypoints', 10)
        self.task_command_pub = self.create_publisher(TaskCommand, '/mission/task_command', 10)
        self.mission_state_pub = self.create_publisher(MissionState, '/mission/state', 10)
        self.led_pub = self.create_publisher(String, '/mt_led_indicator/rover', 10)
        
        # Subscribers
        self.nav_result_sub = self.create_subscription(
            String, '/nav_stack/mission_manager', self.nav_result_callback, 10
        )
        self.aruco_result_sub = self.create_subscription(
            DetectionResult, '/detection/aruco/result', self.detection_result_callback, 10
        )
        self.mallet_result_sub = self.create_subscription(
            DetectionResult, '/detection/mallet/result', self.detection_result_callback, 10
        )
        self.bottle_result_sub = self.create_subscription(
            DetectionResult, '/detection/bottle/result', self.detection_result_callback, 10
        )
        self.start_sub = self.create_subscription(
            Bool, '/mission/start', self.start_callback, 10
        )
        self.abort_sub = self.create_subscription(
            Bool, '/mission/abort', self.abort_callback, 10
        )
        
        # State variables
        self.state = "IDLE"  # IDLE, WAYPOINT_SENT, NAVIGATING, GOAL_REACHED, TASK_EXECUTING, TASK_COMPLETE, MISSION_COMPLETE, ABORTED
        self.waypoints = []
        self.current_waypoint_index = 0
        self.current_task = ""
        self.goal_reached_time = None
        self.task_start_time = None
        self.task_complete_time = None  # Time when task completed and LED turned green
        
        # Load mission if provided
        if mission_file:
            self.load_mission_from_file(mission_file)
            if auto_start and self.waypoints:
                self.start_mission()
        
        self.get_logger().info('Mission Coordinator Node initialized')
        self.publish_state()
    
    def load_mission_from_file(self, filepath):
        """Load mission from YAML file."""
        try:
            if not os.path.exists(filepath):
                self.get_logger().error(f'Mission file not found: {filepath}')
                return False
            
            with open(filepath, 'r') as f:
                mission_data = yaml.safe_load(f)
            
            if 'waypoints' not in mission_data:
                self.get_logger().error('Mission file missing "waypoints" key')
                return False
            
            self.waypoints = mission_data['waypoints']
            
            # Auto-detect format: check if first waypoint has task_type
            if self.waypoints and 'task_type' not in self.waypoints[0]:
                # Simple format - convert to structured
                self.waypoints = [{'lat': wp['lat'], 'lon': wp['lon'], 'task_type': None, 'tolerance': 1.0} 
                                  for wp in self.waypoints]
            else:
                # Structured format - ensure defaults
                for wp in self.waypoints:
                    wp.setdefault('task_type', None)
                    wp.setdefault('tolerance', 1.0)
            
            self.get_logger().info(f'Loaded mission with {len(self.waypoints)} waypoints from {filepath}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to load mission file: {e}')
            return False
    
    def start_callback(self, msg: Bool):
        """Start mission on command."""
        if msg.data and self.state == "IDLE" and self.waypoints:
            self.start_mission()
    
    def abort_callback(self, msg: Bool):
        """Abort mission on command."""
        if msg.data:
            self.get_logger().warn('Mission aborted by user')
            self.state = "ABORTED"
            self.set_led("green")  # Stop indicator
            self.publish_state()
    
    def start_mission(self):
        """Start mission execution."""
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded, cannot start mission')
            return
        
        self.get_logger().info('Starting mission')
        self.current_waypoint_index = 0
        self.send_next_waypoint()
    
    def send_next_waypoint(self):
        """Send next waypoint to navigation stack."""
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('Mission complete!')
            self.state = "MISSION_COMPLETE"
            self.set_led("green")
            self.publish_state()
            return
        
        wp = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f'Sending waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
                               f'lat={wp["lat"]}, lon={wp["lon"]}, task={wp["task_type"]}')
        
        # Publish waypoint to nav stack (format: "lat,lon,type")
        waypoint_msg = String()
        task_type = wp['task_type'] if wp['task_type'] else "gnss"
        waypoint_msg.data = f"{wp['lat']},{wp['lon']},{task_type}"
        self.waypoint_pub.publish(waypoint_msg)
        
        # Set LED to red (navigating)
        self.set_led("red")
        
        # Update state
        self.state = "WAYPOINT_SENT"
        self.current_task = wp['task_type'] if wp['task_type'] else ""
        self.publish_state()
        
        # Immediately transition to NAVIGATING
        self.state = "NAVIGATING"
        self.publish_state()
    
    def nav_result_callback(self, msg: String):
        """Handle navigation result from nav stack."""
        if msg.data == "reached" and self.state == "NAVIGATING":
            self.get_logger().info('Navigation goal reached')
            self.state = "GOAL_REACHED"
            self.goal_reached_time = time.time()
            
            # Keep LED red - will turn green only after task completes
            # (LED stays red during navigation → goal → task execution)
            self.publish_state()
    
    def detection_result_callback(self, msg: DetectionResult):
        """Handle detection results from detection nodes."""
        if self.state == "TASK_EXECUTING" and msg.detection_type == self.current_task:
            if msg.state == "reached" and msg.success:
                self.get_logger().info(f'{msg.detection_type.capitalize()} task completed successfully')
                # Set LED to green when task completes successfully
                self.set_led("green")
                self.task_complete_time = time.time()  # Start 10s delay
                self.state = "TASK_COMPLETE"
                self.publish_state()
            elif msg.state == "failed":
                self.get_logger().warn(f'{msg.detection_type.capitalize()} task failed, moving to next waypoint')
                # Set LED to green even if task failed (waypoint complete)
                self.set_led("green")
                self.task_complete_time = time.time()  # Start 10s delay
                self.state = "TASK_COMPLETE"
                self.publish_state()
    
    def update_loop(self):
        """Main update loop for state machine."""
        current_time = time.time()
        
        # Handle GOAL_REACHED state - immediately start task or complete
        if self.state == "GOAL_REACHED" and self.goal_reached_time is not None:
            # No delay - immediately proceed
            if self.current_task:
                self.get_logger().info(f'Starting {self.current_task} detection task')
                self.state = "TASK_EXECUTING"
                self.task_start_time = current_time
                self.publish_state()
                
                # Send task command to detection node
                task_cmd = TaskCommand()
                task_cmd.task_type = self.current_task
                task_cmd.command = "start"
                self.task_command_pub.publish(task_cmd)
            else:
                # No task, set LED green and start 10s delay
                self.set_led("green")
                self.task_complete_time = current_time  # Start 10s delay
                self.state = "TASK_COMPLETE"
                self.publish_state()
        
        # Handle TASK_EXECUTING timeout (DISABLED - tasks can run indefinitely)
        # if self.state == "TASK_EXECUTING" and self.task_start_time is not None:
        #     elapsed = current_time - self.task_start_time
        #     
        #     if elapsed >= self.task_timeout:
        #         self.get_logger().warn(f'Task timeout after {self.task_timeout}s, moving to next waypoint')
        #         
        #         # Abort task
        #         task_cmd = TaskCommand()
        #         task_cmd.task_type = self.current_task
        #         task_cmd.command = "abort"
        #         self.task_command_pub.publish(task_cmd)
        #         
        #         self.set_led("green")
        #         self.state = "TASK_COMPLETE"
        #         self.publish_state()
        
        # Handle TASK_COMPLETE state - wait 10s after LED green before next waypoint
        if self.state == "TASK_COMPLETE" and self.task_complete_time is not None:
            elapsed = current_time - self.task_complete_time
            
            if elapsed >= self.goal_reached_delay:  # 10 second delay
                self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} complete, moving to next')
                self.current_waypoint_index += 1
                self.current_task = ""
                self.goal_reached_time = None
                self.task_start_time = None
                self.task_complete_time = None
                self.send_next_waypoint()
    
    def set_led(self, color: str):
        """Set LED color."""
        msg = String()
        msg.data = color
        self.led_pub.publish(msg)
        self.get_logger().debug(f'LED set to {color}')
    
    def publish_state(self):
        """Publish current mission state."""
        msg = MissionState()
        msg.state = self.state
        msg.current_waypoint_index = self.current_waypoint_index
        msg.total_waypoints = len(self.waypoints)
        msg.current_task = self.current_task
        msg.progress = (self.current_waypoint_index / len(self.waypoints) * 100.0) if self.waypoints else 0.0
        self.mission_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinatorNode()
    
    # Create timer for update loop
    timer = node.create_timer(0.1, node.update_loop)  # 10Hz
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
