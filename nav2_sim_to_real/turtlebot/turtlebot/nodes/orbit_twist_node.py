#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class OrbitTwistNode(Node):

    def __init__(self):
        super().__init__('orbit_twist')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Tune these ---
        self.linear = 0.6      # m/s
        self.radius = 0.6       # meters

        # clockwise → negative angular.z
        self.angular = - self.linear / self.radius

        self.get_logger().info(f"Orbit started: v={self.linear}, w={self.angular:.3f}")

        self.timer = self.create_timer(0.05, self.loop)

    def loop(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = OrbitTwistNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()