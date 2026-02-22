import cv2
import numpy as np
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import Twist
# from sbg_driver.msg import SbgEkfEuler
import time
from math import degrees

# Constants from original code
RECT_WIDTH = 150
RECT_HEIGHT = 90
LINEAR_SPEED = 90.0
ANGULAR_SPEED = 60.0
STOP_DISTANCE = 1.5
TRACKING_TIMEOUT = 1.5  # Tolerance time in seconds for losing object while tracking



#libraries for socket output
import asyncio
import websockets
import threading
import base64

# Constants from original code
MARKER_SIZE = 0.15
RECT_WIDTH = 300
RECT_HEIGHT = 180
LINEAR_SPEED = 80.0
ANGULAR_SPEED = 60.0
STOP_DISTANCE = 1.5
TRACKING_TIMEOUT = 1.5  # Tolerance time in seconds for losing ArUco while tracking
WEBSOCKET_PORT = 8001

#WebSocket server for video streaming
# This server will broadcast the video stream to all connected clients
# It uses asyncio and websockets to handle multiple clients
# The server will send the video frames as base64 encoded JPEG images
class VideoStreamServer:
    def __init__(self):
        self.clients = set()
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.disconnected_clients = set()

    async def handler(self, websocket):
        """Handle new WebSocket connection."""
        self.clients.add(websocket)
        try:
            # Keep connection alive
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
                            try:
                                await asyncio.wait_for(client.send(jpg_as_text), timeout=0.5)
                            except asyncio.TimeoutError:
                                print("Client send timeout")
                                self.disconnected_clients.add(client)
                                break
                        except:
                           print("Client disconnected")
                           self.disconnected_clients.add(client)
                           break
                    # Remove disconnected clients
                    for client in self.disconnected_clients:
                        self.clients.remove(client)
                    self.disconnected_clients.clear()
                    
                    
            await asyncio.sleep(0.03) # ~30 FPS

    def update_frame(self, frame):
        """Update the current frame to be sent to clients."""
        with self.frame_lock:
            self.current_frame = frame

    async def start_server(self):
        """Start the WebSocket server."""
        async with websockets.serve(self.handler, "0.0.0.0", WEBSOCKET_PORT):
            await self.broadcast_frames()


class YOLOSearchTrackNode(Node):
    def __init__(self):
        super().__init__('yolo_search_track_node')
        
        # Publishers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detection_status_pub = self.create_publisher(String, '/bottle_detection_status', 10)
        
        # Subscribers
        # self.orientation = self.create_subscription(
        #     SbgEkfEuler, "/sbg/ekf_euler", self.orientation_callback, 10
        # )

        self.orientation = self.create_subscription(Float64, "/witmotion_eular/yaw", self.orientation_callback, 10)

        self.continue_sub = self.create_subscription(
            String, "/continue_search_bottle", self.continue_callback, 10
        )
        
        self.target_reached_pub = self.create_publisher(String, '/autonomous_status', 10)
        
        self.point_publish_cmd = self.create_publisher(String, '/point_publish_cmd', 10)

        self.light_status_pub = self.create_publisher(String, '/light_status', 10)

        
        # Initialize video capture
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # Initialize YOLO model
        self.model = YOLO('/home/mt10/mt10_ws/src/mt10_control/mt10_control/bottle.pt')
        
        # Create timer for camera callback
        self.timer = self.create_timer(0.0167, self.camera_callback)  # 60Hz
        

        # Initialize WebSocket server
        self.video_server = VideoStreamServer()
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
        self.current_angle = 0.0
        self.target_angle = 60.0
        self.total_rotation = 0.0
        self.my_yaw = 0.0
        self.start_yaw = None
        
        self.log_info('YOLO Bottle Search and Track Node initialized')
    
    def continue_callback(self, msg: String):
        if self.state == "WAITING" and msg.data == "continue":
            self.log_info('Received continue command, resuming search')
            msg.data= 'red'
            self.light_status_pub.publish(msg)
            self.state = "DETECTING"
            self.detection_start_time = time.time()
            self.total_rotation = 0.0
            self.start_yaw = None

    def log_info(self, msg_text: str):
        self.get_logger().info(msg_text)
        status_msg = String()
        status_msg.data = msg_text
        self.detection_status_pub.publish(status_msg)
    
    def publish_target_reached(self):
        msg = String()
        msg.data = 'reached'
        self.target_reached_pub.publish(msg)
        self.log_info('Published target reached message')
        self.light_status_pub.publish(String(data="green"))
        time.sleep(12.0)
    
    def orientation_callback(self, msg: Float64):
        self.my_yaw = msg.data
    
    def get_movement_instruction(self, object_center, rect_bounds, distance):
        if distance < STOP_DISTANCE:
            return "Stop"
        rect_x1, _, rect_x2, _ = rect_bounds
        x, _ = object_center
        
        if x < rect_x1:
            return "Move Left"
        elif x > rect_x2:
            return "Move Right"
        else:
            return "Move Forward"
    
    def calculate_distance(self, bbox_height, bbox_width):
        if bbox_height <= bbox_width:
            bbox_height = bbox_width
        # Estimate distance based on bounding box height
        reference_height = 0.30  # Average height of a person in meters
        focal_length_pixels = 775  # Same as original
        distance = (reference_height * focal_length_pixels) / bbox_height
        return distance
    
    def draw_guides(self, frame, draw_frame):
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        cv2.line(draw_frame, (0, center_y), (width, center_y), (255, 255, 255), 1)
        cv2.line(draw_frame, (center_x, 0), (center_x, height), (255, 255, 255), 1)
        
        rect_x1 = center_x - RECT_WIDTH // 2
        rect_y1 = center_y - RECT_HEIGHT // 2
        rect_x2 = center_x + RECT_WIDTH // 2
        rect_y2 = center_y + RECT_HEIGHT // 2
        cv2.rectangle(draw_frame, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 255, 0), 1)
        
        return rect_x1, rect_y1, rect_x2, rect_y2
    
    def object_detection(self, frame, run_inference=True):
        display_frame = frame.copy()
        rect_bounds = self.draw_guides(frame, display_frame)
        detected_objects = []
        
        if run_inference:
        # Run YOLO detection
            results = self.model(frame)
            
            for result in results[0].boxes:
                x1, y1, x2, y2 = result.xyxy[0]
                conf = result.conf[0]
                if conf > 0.65:
                    bbox = [int(x1), int(y1), int(x2), int(y2)]
                    center_x = (bbox[0] + bbox[2]) // 2
                    center_y = (bbox[1] + bbox[3]) // 2
                    detected_objects.append({
                        'bbox': bbox,
                        'center': (center_x, center_y),
                        'confidence': conf
                    })
        
        return detected_objects, display_frame, rect_bounds
    
    # Keep all movement and state management methods unchanged
    def start_rotation(self):
        self.state = "ROTATING"
        if self.start_yaw is None:
            self.start_yaw = self.my_yaw
        msg = Twist()
        msg.angular.z = -45.0
    
        self.vel_publisher.publish(msg)
    
    def publish_movement_command(self, instruction):
        msg = Twist()
        
        if instruction == "Stop":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.log_info('Target reached! Stopping robot. Waiting for continue command...')
            
            self.vel_publisher.publish(msg)
            return True
        else:
            if instruction == "Move Forward":
                msg.linear.x = LINEAR_SPEED
            elif instruction == "Move Left":
                msg.angular.z = ANGULAR_SPEED
            elif instruction == "Move Right":
                msg.angular.z = -ANGULAR_SPEED
        
        self.last_instruction = instruction
        
        self.vel_publisher.publish(msg)
            
        return False
    
    def handle_tracking_loss(self):
        current_time = time.time()
        if self.last_tracking_time is None:
            self.last_tracking_time = current_time
            
        time_since_last_track = current_time - self.last_tracking_time
        
        if time_since_last_track < TRACKING_TIMEOUT:
            msg = Twist()
            if self.last_instruction == "Move Forward":
                self.log_info('Object temporarily lost, continuing forward motion')
                msg.linear.x = LINEAR_SPEED
            elif self.last_instruction == "Move Left":
                self.log_info('Object temporarily lost, continuing left turn')
                msg.angular.z = ANGULAR_SPEED
            elif self.last_instruction == "Move Right":
                self.log_info('Object temporarily lost, continuing right turn')
                msg.angular.z = -ANGULAR_SPEED
                
            self.vel_publisher.publish(msg)
            return True
        else:
            self.log_info('Object lost for too long, switching to detection')
            msg = Twist()
            self.vel_publisher.publish(msg)
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
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
        
        if self.state == "WAITING":
            run_inference = False
        else:
            run_inference = True
        
        detected_objects, display_frame, rect_bounds = self.object_detection(frame, run_inference)
        current_time = time.time()
        
        if self.state == "WAITING":
            self.log_info("Waiting...")
            # Just update display, no movement
            if detected_objects:
                for obj in detected_objects:
                    bbox = obj['bbox']
                    cv2.rectangle(display_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
        
        elif self.state == "DETECTING":
            if detected_objects:
                self.state = "TRACKING"
                self.last_tracking_time = current_time
                self.log_info('Object detected, starting tracking')
            elif current_time - self.detection_start_time >= 5.0:
                self.start_rotation()
                self.detection_start_time = current_time
        
        elif self.state == "ROTATING":
            self.start_rotation()
            if self.start_yaw is not None:
                angle_diff = ((self.my_yaw - self.start_yaw) + 360) % 360
                self.log_info(f"Angle diff= {angle_diff}")
                self.log_info(f"Total rotation= {self.total_rotation}")
                if angle_diff >= 60.0 and angle_diff < 350.0:
                    self.total_rotation += 60.0
                    if self.total_rotation >= 360.0:
                        self.log_info('360 rotation done, Object not found. Waiting for next decision.')
                        msg = Twist()
                        self.vel_publisher.publish(msg)
                        self.state = "WAITING"

                        self.publish_target_reached()
                        return
                    
                    msg = Twist()
                    self.vel_publisher.publish(msg)
                    self.state = "DETECTING"
                    self.detection_start_time = current_time
                    self.start_yaw = None
        
        elif self.state == "TRACKING":
            if detected_objects:
                self.last_tracking_time = current_time
                # Track the largest (closest) object
                largest_object = max(detected_objects, key=lambda x: (x['bbox'][2] - x['bbox'][0]) * (x['bbox'][3] - x['bbox'][1]))
                
                bbox = largest_object['bbox']
                center = largest_object['center']
                bbox_height = bbox[3] - bbox[1]
                bbox_width = bbox[2] - bbox[0]
                
                distance = self.calculate_distance(bbox_height, bbox_width)
                instruction = self.get_movement_instruction(center, rect_bounds, distance)
                
                self.log_info(f"Distance to object: {distance:.2f} m")
                self.log_info(f"Movement instruction: {instruction}")
                
                target_reached = self.publish_movement_command(instruction)
                
                # Display detection
                cv2.rectangle(display_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
                
                # Display information
                cv2.putText(display_frame, f"Distance: {distance:.2f}m", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                instruction_color = (0, 0, 255) if instruction == "Stop" else (255, 255, 255)
                cv2.putText(display_frame, f"Instruction: {instruction}", 
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, instruction_color, 2)
                
                if target_reached:
                    self.state = "WAITING"
                    self.publish_target_reached()
            else:
                continuing = self.handle_tracking_loss()
                if continuing:
                    cv2.putText(display_frame, "Object temporarily lost, continuing movement", 
                            (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Display state
        cv2.putText(display_frame, f"State: {self.state}", 
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        # Display state (Sending to websocket server)
        try:
            self.video_server.update_frame(display_frame)
        except Exception as e:
            self.get_logger().error(f"Error updating frame for WebSocket: {e}")
        
        # cv2.imshow('Bottle Search and Track', display_frame)
        cv2.waitKey(1)
    
    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOSearchTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
