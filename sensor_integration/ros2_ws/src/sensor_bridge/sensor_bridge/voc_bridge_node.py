import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class VOCBridge(Node):
    def __init__(self):
        super().__init__('voc_bridge')
        self.pub = self.create_publisher(String, 'air_quality/voc', 10)
        try:
            self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info('Connected to /dev/ttyUSB0')
        except serial.SerialException:
            self.get_logger().error('Failed to open /dev/ttyUSB0')
            exit(1)
        self.timer = self.create_timer(1.0, self.read_serial)

    def read_serial(self):
        line = self.serial.readline().decode(errors='ignore').strip()
        if line:
            msg = String()
            msg.data = line
            self.pub.publish(msg)
            self.get_logger().info(line)

def main(args=None):
    rclpy.init(args=args)
    node = VOCBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
