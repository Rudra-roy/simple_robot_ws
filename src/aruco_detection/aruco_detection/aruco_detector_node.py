#!/usr/bin/env python3
"""
ArUco Detector Node
Refactored from ar_with_new_logic.py for cleaner architecture.
Searches for and tracks ArUco markers using 360° rotation search pattern.
"""

import cv2
import numpy as np
from cv2 import aruco
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from detection_interfaces.msg import DetectionResult, TaskCommand
import time
import asyncio
import websockets
import threading
import base64


class VideoStreamServer:
    """WebSocket server for broadcasting video stream to clients."""
    
    def __init__(self, port):
        self.port = port
        self.clients = set()
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.disconnected_clients = set()
        self.frame_count = 0

    async def handler(self, websocket):
        """Handle new WebSocket connection."""
        print(f"[WebSocket] New client connected. Total clients: {len(self.clients) + 1}")
        self.clients.add(websocket)
        try:
            while True:
                await websocket.recv()
        except websockets.exceptions.ConnectionClosed:
            print(f"[WebSocket] Client disconnected.")
        finally:
            self.clients.remove(websocket)
            print(f"[WebSocket] Client removed. Total clients: {len(self.clients)}")

    async def broadcast_frames(self):
        """Broadcast frames to all connected clients."""
        while True:
            with self.frame_lock:
                if self.current_frame is not None:
                    _, buffer = cv2.imencode('.jpg', self.current_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                    
                    for client in self.clients:
                        try:
                            await asyncio.wait_for(client.send(jpg_as_text), timeout=0.5)
                        except (asyncio.TimeoutError, Exception):
                            self.disconnected_clients.add(client)
                            break
                    
                    # Remove disconnected clients
                    for client in self.disconnected_clients:
                        self.clients.discard(client)
                    self.disconnected_clients.clear()
                    
            await asyncio.sleep(0.03)  # ~30 FPS

    def update_frame(self, frame):
        """Update the current frame to be sent to clients."""
        with self.frame_lock:
            self.current_frame = frame
            self.frame_count += 1
            if self.frame_count % 100 == 0:
                print(f"[WebSocket] Frames processed: {self.frame_count}, Connected clients: {len(self.clients)}")

    async def start_server(self):
        """Start the WebSocket server."""
        print(f"[WebSocket] Starting server on 0.0.0.0:{self.port}")
        async with websockets.serve(self.handler, "0.0.0.0", self.port):
            print(f"[WebSocket] Server is now listening on port {self.port}")
            await self.broadcast_frames()


class ArucoDetectorNode(Node):
    """ArUco marker detection and tracking node."""
    
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # Declare parameters
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('linear_speed', 80.0)
        self.declare_parameter('angular_speed', 60.0)
        self.declare_parameter('stop_distance', 1.5)
        self.declare_parameter('tracking_timeout', 1.5)
        self.declare_parameter('websocket_port', 8000)
        self.declare_parameter('aruco_dict', '4x4_50')
        self.declare_parameter('focal_length', 941)
        self.declare_parameter('rect_width', 300)
        self.declare_parameter('rect_height', 180)
        self.declare_parameter('rotation_increment', 50.0)
        
        # Get parameters
        camera_device = self.get_parameter('camera_device').value
        self.marker_size = self.get_parameter('marker_size').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.tracking_timeout = self.get_parameter('tracking_timeout').value
        websocket_port = self.get_parameter('websocket_port').value
        self.focal_length = self.get_parameter('focal_length').value
        self.rect_width = self.get_parameter('rect_width').value
        self.rect_height = self.get_parameter('rect_height').value
        self.rotation_increment = self.get_parameter('rotation_increment').value
        
        # Publishers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.detection_result_pub = self.create_publisher(
            DetectionResult, '/detection/aruco/result', 10
        )
        self.status_pub = self.create_publisher(String, '/detection/aruco/status', 10)
        
        # Subscribers
        self.orientation_sub = self.create_subscription(
            Float64, '/witmotion_eular/yaw', self.orientation_callback, 10
        )
        self.task_command_sub = self.create_subscription(
            TaskCommand, '/mission/task_command', self.task_command_callback, 10
        )
        
        # Initialize video capture
        if isinstance(camera_device, str):
            self.cap = cv2.VideoCapture(camera_device)
        else:
            self.cap = cv2.VideoCapture(camera_device)
        
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device: {camera_device}')
            raise RuntimeError('Camera initialization failed')
        
        # Load ArUco dictionary
        aruco_dict_name = self.get_parameter('aruco_dict').value
        self.aruco_dict = self.get_aruco_dictionary(aruco_dict_name)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_params.errorCorrectionRate = 0.6
        
        # Create timer for camera processing
        self.timer = self.create_timer(0.0167, self.camera_callback)  # 60Hz
        
        # Initialize WebSocket server
        self.video_server = VideoStreamServer(websocket_port)
        self.websocket_thread = threading.Thread(
            target=self.run_websocket_server,
            daemon=True
        )
        self.websocket_thread.start()
        
        # State variables
        self.state = "WAITING"
        self.detection_start_time = time.time()
        self.last_tracking_time = None
        self.last_instruction = None
        self.current_yaw = 0.0
        self.start_yaw = None
        self.total_rotation = 0.0
        
        self.get_logger().info('ArUco Detector Node initialized')
        self.publish_status('ArUco Detector initialized and waiting for task command')
    
    def get_aruco_dictionary(self, dict_name):
        """Get ArUco dictionary by name."""
        dict_map = {
            '4x4_50': aruco.DICT_4X4_50,
            '4x4_100': aruco.DICT_4X4_100,
            '4x4_250': aruco.DICT_4X4_250,
            '4x4_1000': aruco.DICT_4X4_1000,
            '5x5_50': aruco.DICT_5X5_50,
            '5x5_100': aruco.DICT_5X5_100,
            '5x5_250': aruco.DICT_5X5_250,
            '5x5_1000': aruco.DICT_5X5_1000,
            '6x6_50': aruco.DICT_6X6_50,
            '6x6_100': aruco.DICT_6X6_100,
            '6x6_250': aruco.DICT_6X6_250,
            '6x6_1000': aruco.DICT_6X6_1000,
        }
        return aruco.getPredefinedDictionary(dict_map.get(dict_name, aruco.DICT_4X4_50))
    
    def task_command_callback(self, msg: TaskCommand):
        """Handle task commands from mission manager."""
        if msg.task_type != "aruco":
            return  # Ignore commands for other detectors
        
        if msg.command == "start" and self.state == "WAITING":
            self.get_logger().info('Received start command, beginning ArUco search')
            self.publish_status('Starting ArUco search')
            self.state = "DETECTING"
            self.detection_start_time = time.time()
            self.total_rotation = 0.0
            self.start_yaw = None
            self.publish_detection_result("searching", False, 0.0, 0.0)
        
        elif msg.command == "abort":
            self.get_logger().info('Received abort command, stopping')
            self.stop_robot()
            self.state = "WAITING"
            self.publish_detection_result("failed", False, 0.0, 0.0)
    
    def orientation_callback(self, msg: Float64):
        """Update current yaw from IMU."""
        self.current_yaw = msg.data
    
    def publish_status(self, message: str):
        """Publish status message for logging."""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(message)
    
    def publish_detection_result(self, state: str, success: bool, distance: float, confidence: float):
        """Publish detection result message."""
        result = DetectionResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = "base_link"
        result.detection_type = "aruco"
        result.success = success
        result.distance = distance
        result.confidence = confidence
        result.state = state
        
        # Set target pose (relative to robot)
        result.target_pose = Pose()
        result.target_pose.position = Point(x=distance, y=0.0, z=0.0)
        result.target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.detection_result_pub.publish(result)
    
    def stop_robot(self):
        """Stop all robot movement."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.vel_publisher.publish(msg)
    
    def calculate_distance(self, marker_width_pixels):
        """Calculate distance to marker using known marker size."""
        distance = (self.marker_size * self.focal_length) / marker_width_pixels
        return distance
    
    def draw_guides(self, frame, draw_frame):
        """Draw center crosshairs and target rectangle."""
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        # Draw crosshairs
        cv2.line(draw_frame, (0, center_y), (width, center_y), (255, 255, 255), 1)
        cv2.line(draw_frame, (center_x, 0), (center_x, height), (255, 255, 255), 1)
        
        # Draw target rectangle
        rect_x1 = center_x - self.rect_width // 2
        rect_y1 = center_y - self.rect_height // 2
        rect_x2 = center_x + self.rect_width // 2
        rect_y2 = center_y + self.rect_height // 2
        cv2.rectangle(draw_frame, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 255, 0), 1)
        
        return rect_x1, rect_y1, rect_x2, rect_y2
    
    def get_movement_instruction(self, marker_center, rect_bounds, distance):
        """Determine movement command based on marker position."""
        if distance < self.stop_distance:
            return "Stop"
        
        rect_x1, _, rect_x2, _ = rect_bounds
        x, _ = marker_center
        
        if x < rect_x1:
            return "Move Left"
        elif x > rect_x2:
            return "Move Right"
        else:
            return "Move Forward"
    
    def detect_aruco(self, frame):
        """Detect ArUco markers in frame."""
        display_frame = frame.copy()
        corners, ids, _ = aruco.detectMarkers(display_frame, self.aruco_dict, parameters=self.aruco_params)
        rect_bounds = self.draw_guides(frame, display_frame)
        return corners, ids, display_frame, rect_bounds
    
    def start_rotation(self):
        """Start rotation to search for marker."""
        if self.start_yaw is None:
            self.start_yaw = self.current_yaw
        
        msg = Twist()
        msg.angular.z = -self.angular_speed / 1.3
        self.vel_publisher.publish(msg)
    
    def publish_movement_command(self, instruction, distance):
        """Publish movement command based on instruction."""
        msg = Twist()
        
        if instruction == "Stop":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.vel_publisher.publish(msg)
            self.publish_status('Target reached! Stopping robot.')
            self.last_instruction = instruction
            return True
        
        elif instruction == "Move Forward":
            msg.linear.x = self.linear_speed
        elif instruction == "Move Left":
            msg.angular.z = self.angular_speed
        elif instruction == "Move Right":
            msg.angular.z = -self.angular_speed
        
        self.last_instruction = instruction
        self.vel_publisher.publish(msg)
        return False
    
    def handle_tracking_loss(self):
        """Handle temporary loss of marker during tracking."""
        current_time = time.time()
        if self.last_tracking_time is None:
            self.last_tracking_time = current_time
        
        time_since_last_track = current_time - self.last_tracking_time
        
        if time_since_last_track < self.tracking_timeout:
            # Continue last movement
            msg = Twist()
            if self.last_instruction == "Move Forward":
                self.publish_status('ArUco temporarily lost, continuing forward')
                msg.linear.x = self.linear_speed
            elif self.last_instruction == "Move Left":
                self.publish_status('ArUco temporarily lost, continuing left turn')
                msg.angular.z = self.angular_speed
            elif self.last_instruction == "Move Right":
                self.publish_status('ArUco temporarily lost, continuing right turn')
                msg.angular.z = -self.angular_speed
            
            self.vel_publisher.publish(msg)
            return True
        else:
            # Timeout exceeded, stop and switch to detection
            self.publish_status('ArUco lost for too long, switching to detection')
            self.stop_robot()
            self.state = "DETECTING"
            self.detection_start_time = current_time
            self.last_tracking_time = None
            self.last_instruction = None
            return False
    
    def run_websocket_server(self):
        """Run the WebSocket server in a separate thread."""
        try:
            asyncio.set_event_loop(asyncio.new_event_loop())
            loop = asyncio.get_event_loop()
            self.get_logger().info(f'Starting WebSocket server on port {self.video_server.port}')
            loop.run_until_complete(self.video_server.start_server())
        except Exception as e:
            self.get_logger().error(f'WebSocket server failed to start: {e}')
    
    def camera_callback(self):
        """Main camera processing loop."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
        
        corners, ids, display_frame, rect_bounds = self.detect_aruco(frame)
        current_time = time.time()
        
        # State machine
        if self.state == "WAITING":
            # Just update display, no movement
            if ids is not None:
                aruco.drawDetectedMarkers(display_frame, corners)
        
        elif self.state == "DETECTING":
            if ids is not None:
                self.state = "TRACKING"
                self.last_tracking_time = current_time
                self.publish_status('ArUco detected, starting tracking')
                self.publish_detection_result("tracking", False, 0.0, 0.8)
            elif current_time - self.detection_start_time >= 3.0:
                self.state = "ROTATING"
                self.publish_status('No marker detected, starting rotation search')
                self.detection_start_time = current_time
        
        elif self.state == "ROTATING":
            if ids is not None:
                self.publish_status('ArUco detected during rotation')
                self.state = "TRACKING"
                self.last_tracking_time = current_time
                self.stop_robot()
                self.publish_detection_result("tracking", False, 0.0, 0.8)
            else:
                self.start_rotation()
                
                if self.start_yaw is not None:
                    angle_diff = (self.current_yaw - self.start_yaw) % 360
                    
                    if angle_diff >= self.rotation_increment and angle_diff <= (360 - self.rotation_increment):
                        self.total_rotation += self.rotation_increment
                        self.start_yaw = self.current_yaw
                        
                        if self.total_rotation >= 360.0:
                            self.publish_status('360° rotation complete, ArUco not found')
                            self.stop_robot()
                            self.state = "WAITING"
                            self.publish_detection_result("failed", False, 0.0, 0.0)
                            return
                        
                        self.stop_robot()
                        self.state = "DETECTING"
                        self.detection_start_time = current_time
                        self.start_yaw = None
        
        elif self.state == "TRACKING":
            if ids is not None:
                self.last_tracking_time = current_time
                
                for i in range(len(ids)):
                    marker_corners = corners[i][0]
                    marker_center = tuple(np.mean(marker_corners, axis=0).astype(int))
                    marker_width_pixels = np.linalg.norm(marker_corners[0] - marker_corners[1])
                    
                    distance = self.calculate_distance(marker_width_pixels)
                    instruction = self.get_movement_instruction(marker_center, rect_bounds, distance)
                    
                    self.publish_status(f'Distance: {distance:.2f}m, Instruction: {instruction}')
                    self.publish_detection_result("tracking", False, distance, 0.9)
                    
                    target_reached = self.publish_movement_command(instruction, distance)
                    
                    # Draw markers
                    aruco.drawDetectedMarkers(display_frame, corners)
                    
                    # Display information
                    cv2.putText(display_frame, f"ID: {ids[i][0]} Distance: {distance:.2f}m",
                                (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    instruction_color = (0, 0, 255) if instruction == "Stop" else (255, 255, 255)
                    cv2.putText(display_frame, f"Instruction: {instruction}",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2)
                    
                    if target_reached:
                        self.state = "WAITING"
                        self.publish_detection_result("reached", True, distance, 1.0)
                        self.publish_status('Target reached successfully')
            else:
                # Handle temporary tracking loss
                continuing = self.handle_tracking_loss()
                if continuing:
                    cv2.putText(display_frame, "ArUco temporarily lost, continuing movement",
                                (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Display state
        cv2.putText(display_frame, f"State: {self.state}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        
        # Update WebSocket stream
        try:
            self.video_server.update_frame(display_frame)
            cv2.imshow("aruco", display_frame)
        except Exception as e:
            self.get_logger().error(f'Error updating frame for WebSocket: {e}')
        
        cv2.waitKey(1)
    
    def __del__(self):
        """Cleanup on node destruction."""
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
