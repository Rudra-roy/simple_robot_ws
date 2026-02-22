import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

def quaternion_to_euler(q_w, q_x, q_y, q_z):
    # Roll (x-axis rotation)
    roll = math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))
    
    # Pitch (y-axis rotation)
    pitch = math.asin(2 * (q_w * q_y - q_z * q_x))
    
    # Yaw (z-axis rotation)
    yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
    
    return roll, pitch, yaw

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (in radians) to quaternion.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q_w = cr * cp * cy + sr * sp * sy
    q_x = sr * cp * cy - cr * sp * sy
    q_y = cr * sp * cy + sr * cp * sy
    q_z = cr * cp * sy - sr * sp * cy

    return q_w, q_x, q_y, q_z

class QuaternionToEulerNode(Node):
    def __init__(self):
        super().__init__('quaternion_to_euler')
        
        # Subscribe to /orientation for quaternion data
        self.subscription = self.create_subscription(
            Quaternion,
            '/orientation',
            self.orientation_callback,
            10
        )
        
        # Subscribe to /imu/data_raw for acceleration and angular velocity
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_data_callback,
            10
        )
        
        # Publishers for Euler angles
        self.yaw = self.create_publisher(Float64, "/witmotion_eular/yaw", 10)
        self.roll = self.create_publisher(Float64, "/witmotion_eular/roll", 10)
        self.pitch = self.create_publisher(Float64, "/witmotion_eular/pitch", 10)
        
        # Publisher for corrected IMU message
        self.imu_corrected_pub = self.create_publisher(Imu, "/imu/data_corrected", 10)

        self.get_logger().info('Quaternion to Euler node has started.')

        self.yaw_value = Float64()
        self.roll_value = Float64()
        self.pitch_value = Float64()
        
        # Store IMU sensor data
        self.latest_imu_data = None

    def imu_data_callback(self, msg):
        """Store the latest IMU data (acceleration and angular velocity)"""
        self.latest_imu_data = msg

    def orientation_callback(self, msg):
        # Extract quaternion components from orientation message
        x = msg.x
        y = msg.y
        z = msg.z
        w = msg.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = quaternion_to_euler(w, x, y, z)
        roll  = (((math.degrees(roll)%360))*(-1)+(360))
        self.yaw_value.data = roll

        self.roll_value.data = (math.degrees(pitch)%360)*(-1)+360
        self.pitch_value.data = (math.degrees(yaw)%360)*(-1)+360

        
        self.roll.publish(self.roll_value)
        print(f"roll: {self.roll_value.data}")

        
        self.pitch.publish(self.pitch_value)
        print(f"Pitch: {self.pitch_value.data}")


       
        self.yaw_value.data = (self.yaw_value.data - 157.0) % 360
        self.yaw.publish(self.yaw_value)
        print(f"Yaw: {self.yaw_value.data}")
        
        # Only publish IMU message if we have received IMU data
        if self.latest_imu_data is None:
            return
        
        # Calculate quaternion from processed yaw, roll, pitch values
        # Convert back to radians
        roll_rad = math.radians(self.roll_value.data)
        pitch_rad = math.radians(self.pitch_value.data)
        yaw_rad = math.radians(self.yaw_value.data)
        
        # Compute new quaternion
        q_w, q_x, q_y, q_z = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
        
        # Create a new IMU message with corrected orientation
        imu_corrected = Imu()
        imu_corrected.header = self.latest_imu_data.header
        imu_corrected.header.stamp = self.get_clock().now().to_msg()
        
        # Copy linear acceleration from /imu/data_raw
        imu_corrected.linear_acceleration = self.latest_imu_data.linear_acceleration
        imu_corrected.linear_acceleration_covariance = self.latest_imu_data.linear_acceleration_covariance
        
        # Copy angular velocity from /imu/data_raw
        imu_corrected.angular_velocity = self.latest_imu_data.angular_velocity
        imu_corrected.angular_velocity_covariance = self.latest_imu_data.angular_velocity_covariance
        
        # Set the corrected quaternion orientation calculated from roll_value, pitch_value, yaw_value
        imu_corrected.orientation.x = q_x
        imu_corrected.orientation.y = q_y
        imu_corrected.orientation.z = q_z
        imu_corrected.orientation.w = q_w
        imu_corrected.orientation_covariance = self.latest_imu_data.orientation_covariance
        
        # Publish the corrected IMU message
        self.imu_corrected_pub.publish(imu_corrected)


def main(args=None):
    rclpy.init(args=args)
    node = QuaternionToEulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
