#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, Twist
import math


class GPSNavigationNode(Node):
    def __init__(self):
        super().__init__('gps_navigation_node')
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps_fix',
            self.gps_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/gps_target',
            self.target_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self.target_lat = None
        self.target_lon = None
        
        self.max_linear_vel = 2.0
        self.max_angular_vel = 3.5
        
        self.timer = self.create_timer(0.1, self.navigation_update)
        
        self.log_counter = 0
        self.log_interval = 10
        
        self.get_logger().info('GPS Navigation Node started')
        self.get_logger().info(f'Max Linear Velocity: {self.max_linear_vel} m/s')
        self.get_logger().info(f'Max Angular Velocity: {self.max_angular_vel} rad/s')
        self.get_logger().info('Publishing velocity commands at 10 Hz')
    
    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
    
    def imu_callback(self, msg):
        orientation_q = msg.orientation
        siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_heading = math.degrees(yaw)
        if self.current_heading < 0:
            self.current_heading += 360.0
    
    def target_callback(self, msg):
        self.target_lat = msg.pose.position.x
        self.target_lon = msg.pose.position.y
        self.get_logger().info(f'New target received: Lat={self.target_lat:.6f}, Lon={self.target_lon:.6f}')
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000.0
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi / 2.0) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda / 2.0) ** 2
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        
        distance = R * c
        return distance
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)
        
        y = math.sin(delta_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - \
            math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        
        theta = math.atan2(y, x)
        bearing = (math.degrees(theta) + 360.0) % 360.0
        
        return bearing
    
    def navigation_update(self):
        if None in [self.current_lat, self.current_lon, self.current_heading, 
                    self.target_lat, self.target_lon]:
            if self.target_lat is None:
                self.get_logger().info('Waiting for target GPS coordinates', throttle_duration_sec=5.0)
            elif self.current_lat is None:
                self.get_logger().info('Waiting for GPS fix', throttle_duration_sec=5.0)
            elif self.current_heading is None:
                self.get_logger().info('Waiting for IMU data', throttle_duration_sec=5.0)
            return
        
        distance = self.haversine_distance(
            self.current_lat, self.current_lon,
            self.target_lat, self.target_lon
        )
        
        target_bearing = self.calculate_bearing(
            self.current_lat, self.current_lon,
            self.target_lat, self.target_lon
        )
        
        angle_diff = target_bearing - self.current_heading
        if angle_diff > 180.0:
            angle_diff -= 360.0
        elif angle_diff < -180.0:
            angle_diff += 360.0
        
        if abs(angle_diff) < 5.0:
            direction = "ALIGNED"
        elif angle_diff > 0:
            direction = "TURN RIGHT"
        else:
            direction = "TURN LEFT"
        
        cmd_vel = Twist()
        
        self.log_counter += 1
        should_log = (self.log_counter % self.log_interval == 0)
        
        if distance < 1.0:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            if should_log:
                self.get_logger().info('=' * 70)
                self.get_logger().info('TARGET REACHED!')
                self.get_logger().info(f'Linear Velocity: {cmd_vel.linear.x:.2f} m/s')
                self.get_logger().info(f'Angular Velocity: {cmd_vel.angular.z:.2f} rad/s')
                self.get_logger().info('=' * 70)
        else:
            if abs(angle_diff) > 10.0:
                if angle_diff > 0:
                    cmd_vel.angular.z = self.max_angular_vel
                else:
                    cmd_vel.angular.z = -self.max_angular_vel
                cmd_vel.linear.x = 0.0
            else:
                cmd_vel.linear.x = self.max_linear_vel
                cmd_vel.angular.z = 0.0
            
            if should_log:
                self.get_logger().info('=' * 70)
                self.get_logger().info(f'Current Position: Lat={self.current_lat:.6f}, Lon={self.current_lon:.6f}')
                self.get_logger().info(f'Target Position:  Lat={self.target_lat:.6f}, Lon={self.target_lon:.6f}')
                self.get_logger().info(f'Distance to Target: {distance:.2f} meters')
                self.get_logger().info(f'Current Heading: {self.current_heading:.2f} degrees')
                self.get_logger().info(f'Target Bearing:  {target_bearing:.2f} degrees')
                self.get_logger().info(f'Angle Difference: {angle_diff:.2f} degrees')
                self.get_logger().info(f'Direction Command: {direction}')
                self.get_logger().info(f'Linear Velocity: {cmd_vel.linear.x:.2f} m/s')
                self.get_logger().info(f'Angular Velocity: {cmd_vel.angular.z:.2f} rad/s')
                self.get_logger().info('=' * 70)
        
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = GPSNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
