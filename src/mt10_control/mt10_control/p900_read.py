import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64
from sbg_driver.msg import SbgGpsPos
import serial
from rclpy.timer import Timer

class SerialCommunicator(Node):
    def __init__(self):
        super().__init__('serial_communicator')
        
        # Setup serial communication
        self.ser = serial.Serial('/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0D6KGW-if00-port0', 56700, timeout=1)  # Adjust port and baudrate as needed

        # Timer to read from serial every 1 second
        # self.timer = self.create_timer(0.001, self.read_from_serial)

        # Publisher to send data to another node
        # self.publisher = self.create_publisher(String, '/minipc_read_telemetry', 1)

        # Subscriber to receive data and send it to the serial port
        # self.create_subscription(String, '/minipc_write_telemetry', self.write_to_serial, 1)
        
        # Subscriber for sending host machine topic data
        # self.create_subscription(Float64, "/witmotion_eular/yaw", self.yaw_callback, 10)
        # self.create_subscription(SbgGpsPos, "/sbg/gps_pos", self.gps_callback, 10)
        # self.create_subscription(String, "/status", self.status_callback, 10)
       
        # Create Publisher for converting serial data to ros data
        self.witmotion_pub = self.create_publisher(Float64, "/witmotion_eular/yaw", 10)
        self.gps_pos_pub = self.create_publisher(SbgGpsPos, "/best_gps", 10)
        self.aruco_status_pub = self.create_publisher(String, "/aruco_status", 10)
        self.mallet_status_pub = self.create_publisher(String, "/mallet_status", 10)
        self.bottle_status_pub = self.create_publisher(String, "/bottle_status", 10)
        self.roll_pub = self.create_publisher(Float64, "/witmotion_eular/roll", 10)
        self.pitch_pub = self.create_publisher(Float64, "/witmotion_eular/pitch", 10)
        self.status_pub = self.create_publisher(String, "/status", 10)


        

    def read_from_serial(self):
        """ Read data from the serial device and publish it if available """
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').strip()  # Read and decode serial data
            self.get_logger().info(f'Read from serial: {data}')

            topic, ros_data = data.split("#")

            if topic == "/witmotion_eular/yaw":
                msg = Float64()
                msg.data = int(ros_data)
                self.witmotion_pub.publish(msg)

            elif topic == "/best_gps_acc":
                msg = SbgGpsPos()
                latitude,longitude,positional_accuracy_x,positional_accuracy_y,positional_accuracy_z = ros_data.split("*")
                msg.latitude = float(latitude)
                msg.longitude = float(longitude)
                msg.position_accuracy.x = float(position_accuracy_x)
                msg.position_accuracy.y = float(position_accuracy_y)
                msg.position_accuracy.z = float(position_accuracy_z)
                self.gps_pos_pub.publish(msg)

            elif topic == "/status":
                msg = String()
                msg.data = ros_data
                self.status_pub.publish(msg)

            elif topic == "/aruco_status":
                msg = String()
                msg.data = ros_data
                self.aruco_status_pub.publish(msg)

            elif topic == "/mallet_status":
                msg = String()
                msg.data = ros_data
                self.aruco_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SerialCommunicator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()  # Close the serial port when shutting down
        rclpy.shutdown()

if __name__ == '__main__':
    main()

