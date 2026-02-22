#!/usr/bin/env python3
"""
YOLO Object Detector Base Class
Shared functionality for all YOLO-based object detection nodes (mallet, bottle, etc.)
"""

import cv2
import numpy as np
from ultralytics import YOLO
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

    async def handler(self, websocket):
        """Handle new WebSocket connection."""
        self.clients.add(websocket)
        try:
            while True:
                await websocket.recv()
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.remove(websocket)

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

    async def start_server(self):
        """Start the WebSocket server."""
        async with websockets.serve(self.handler, "0.0.0.0", self.port):
            await self.broadcast_frames()


class YOLODetectorBase(Node):
    """Base class for YOLO-based object detection nodes."""
    
    def __init__(self, node_name, object_type):
        super().__init__(node_name)
        
        self.object_type = object_type  # "mallet", "bottle", etc.
        
        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('reference_height', 0.10)
        self.declare_parameter('focal_length', 941)
        self.declare_parameter('linear_speed', 90.0)
        self.declare_parameter('angular_speed', 60.0)
        self.declare_parameter('stop_distance', 1.5)
        self.declare_parameter('tracking_timeout', 1.5)
        self.declare_parameter('confidence_threshold', 0.65)
        self.declare_parameter('websocket_port', 8000)
        self.declare_parameter('rect_width', 300)
        self.declare_parameter('rect_height', 180)
        self.declare_parameter('rotation_increment', 60.0)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        camera_device = self.get_parameter('camera_device').value
        self.reference_height = self.get_parameter('reference_height').value
        self.focal_length = self.get_parameter('focal_length').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.tracking_timeout = self.get_parameter('tracking_timeout').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        websocket_port = self.get_parameter('websocket_port').value
        self.rect_width = self.get_parameter('rect_width').value
        self.rect_height = self.get_parameter('rect_height').value
        self.rotation_increment = self.get_parameter('rotation_increment').value
        
        # Publishers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.detection_result_pub = self.create_publisher(
            DetectionResult, f'/detection/{object_type}/result', 10
        )
        self.status_pub = self.create_publisher(String, f'/detection/{object_type}/status', 10)
        
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
        
        # Load YOLO model
        if not model_path:
            raise ValueError(f'{object_type} model_path parameter is required')
        
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)
        
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
        
        self.get_logger().info(f'{object_type.capitalize()} Detector Node initialized')
        self.publish_status(f'{object_type.capitalize()} detector initialized and waiting for task command')
    
    def task_command_callback(self, msg: TaskCommand):
        """Handle task commands from mission manager."""
        if msg.task_type != self.object_type:
            return  # Ignore commands for other detectors
        
        if msg.command == "start" and self.state == "WAITING":
            self.get_logger().info(f'Received start command, beginning {self.object_type} search')
            self.publish_status(f'Starting {self.object_type} search')
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
        result.detection_type = self.object_type
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
    
    def calculate_distance(self, bbox_height):
        """Calculate distance to object using bounding box height."""
        distance = (self.reference_height * self.focal_length) / bbox_height
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
    
    def get_movement_instruction(self, object_center, rect_bounds, distance):
        """Determine movement command based on object position."""
        if distance < self.stop_distance:
            return "Stop"
        
        rect_x1, _, rect_x2, _ = rect_bounds
        x, _ = object_center
        
        if x < rect_x1:
            return "Move Left"
        elif x > rect_x2:
            return "Move Right"
        else:
            return "Move Forward"
    
    def detect_object(self, frame, run_inference=True):
        """Detect objects using YOLO."""
        display_frame = frame.copy()
        rect_bounds = self.draw_guides(frame, display_frame)
        detected_objects = []
        
        if run_inference:
            # Run YOLO detection
            results = self.model(frame)
            
            for result in results[0].boxes:
                x1, y1, x2, y2 = result.xyxy[0]
                conf = result.conf[0]
                if conf > self.confidence_threshold:
                    bbox = [int(x1), int(y1), int(x2), int(y2)]
                    center_x = (bbox[0] + bbox[2]) // 2
                    center_y = (bbox[1] + bbox[3]) // 2
                    detected_objects.append({
                        'bbox': bbox,
                        'center': (center_x, center_y),
                        'confidence': float(conf)
                    })
        
        return detected_objects, display_frame, rect_bounds
    
    def start_rotation(self):
        """Start rotation to search for object."""
        if self.start_yaw is None:
            self.start_yaw = self.current_yaw
        
        msg = Twist()
        msg.angular.z = -45.0
        self.vel_publisher.publish(msg)
    
    def publish_movement_command(self, instruction, distance):
        """Publish movement command based on instruction."""
        msg = Twist()
        
        if instruction == "Stop":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.vel_publisher.publish(msg)
            self.publish_status('Target reached! Stopping robot.')
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
        """Handle temporary loss of object during tracking."""
        current_time = time.time()
        if self.last_tracking_time is None:
            self.last_tracking_time = current_time
        
        time_since_last_track = current_time - self.last_tracking_time
        
        if time_since_last_track < self.tracking_timeout:
            # Continue last movement
            msg = Twist()
            if self.last_instruction == "Move Forward":
                self.publish_status('Object temporarily lost, continuing forward')
                msg.linear.x = self.linear_speed
            elif self.last_instruction == "Move Left":
                self.publish_status('Object temporarily lost, continuing left turn')
                msg.angular.z = self.angular_speed
            elif self.last_instruction == "Move Right":
                self.publish_status('Object temporarily lost, continuing right turn')
                msg.angular.z = -self.angular_speed
            
            self.vel_publisher.publish(msg)
            return True
        else:
            # Timeout exceeded, stop and switch to detection
            self.publish_status('Object lost for too long, switching to detection')
            self.stop_robot()
            self.state = "DETECTING"
            self.detection_start_time = current_time
            self.last_tracking_time = None
            self.last_instruction = None
            return False
    
    def run_websocket_server(self):
        """Run the WebSocket server in a separate thread."""
        asyncio.set_event_loop(asyncio.new_event_loop())
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.video_server.start_server())
    
    def camera_callback(self):
        """Main camera processing loop."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
        
        detected_objects, display_frame, rect_bounds = self.detect_object(frame, run_inference=(self.state != "WAITING"))
        current_time = time.time()
        
        # State machine
        if self.state == "WAITING":
            # Just update display, no movement
            pass
        
        elif self.state == "DETECTING":
            if detected_objects:
                self.state = "TRACKING"
                self.last_tracking_time = current_time
                self.publish_status(f'{self.object_type.capitalize()} detected, starting tracking')
                self.publish_detection_result("tracking", False, 0.0, detected_objects[0]['confidence'])
            elif current_time - self.detection_start_time >= 3.0:
                self.state = "ROTATING"
                self.publish_status(f'No {self.object_type} detected, starting rotation search')
                self.detection_start_time = current_time
        
        elif self.state == "ROTATING":
            if detected_objects:
                self.publish_status(f'{self.object_type.capitalize()} detected during rotation')
                self.state = "TRACKING"
                self.last_tracking_time = current_time
                self.stop_robot()
                self.publish_detection_result("tracking", False, 0.0, detected_objects[0]['confidence'])
            else:
                self.start_rotation()
                
                if self.start_yaw is not None:
                    angle_diff = (self.current_yaw - self.start_yaw) % 360
                    
                    if angle_diff >= self.rotation_increment and angle_diff <= (360 - self.rotation_increment):
                        self.total_rotation += self.rotation_increment
                        self.start_yaw = self.current_yaw
                        
                        if self.total_rotation >= 360.0:
                            self.publish_status(f'360° rotation complete, {self.object_type} not found')
                            self.stop_robot()
                            self.state = "WAITING"
                            self.publish_detection_result("failed", False, 0.0, 0.0)
                            return
                        
                        self.stop_robot()
                        self.state = "DETECTING"
                        self.detection_start_time = current_time
                        self.start_yaw = None
        
        elif self.state == "TRACKING":
            if detected_objects:
                self.last_tracking_time = current_time
                
                # Use the first detected object
                obj = detected_objects[0]
                bbox = obj['bbox']
                object_center = obj['center']
                confidence = obj['confidence']
                bbox_height = bbox[3] - bbox[1]
                
                distance = self.calculate_distance(bbox_height)
                instruction = self.get_movement_instruction(object_center, rect_bounds, distance)
                
                self.publish_status(f'Distance: {distance:.2f}m, Instruction: {instruction}')
                self.publish_detection_result("tracking", False, distance, confidence)
                
                target_reached = self.publish_movement_command(instruction, distance)
                
                # Draw bounding box
                cv2.rectangle(display_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
                cv2.circle(display_frame, object_center, 5, (0, 0, 255), -1)
                
                # Display information
                cv2.putText(display_frame, f"{self.object_type.capitalize()}: {confidence:.2f} Distance: {distance:.2f}m",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                instruction_color = (0, 0, 255) if instruction == "Stop" else (255, 255, 255)
                cv2.putText(display_frame, f"Instruction: {instruction}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2)
                
                if target_reached:
                    self.state = "WAITING"
                    self.publish_detection_result("reached", True, distance, confidence)
                    self.publish_status('Target reached successfully')
            else:
                # Handle temporary tracking loss
                continuing = self.handle_tracking_loss()
                if continuing:
                    cv2.putText(display_frame, "Object temporarily lost, continuing movement",
                                (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Display state
        cv2.putText(display_frame, f"State: {self.state}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        
        # Update WebSocket stream
        try:
            self.video_server.update_frame(display_frame)
        except Exception as e:
            self.get_logger().error(f'Error updating frame for WebSocket: {e}')
        
        cv2.waitKey(1)
    
    def __del__(self):
        """Cleanup on node destruction."""
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()
