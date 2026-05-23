import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFreshRelayNode(Node):
    def __init__(self):
        super().__init__('scan_fresh_relay')
        self.pub = self.create_publisher(LaserScan, '/scan_fresh', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.get_logger().info("Scan fresh relay node created successfully")


    def cb(self, msg: LaserScan):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanFreshRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()