#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration


MAP_FRAME = "map"
BASE_FRAME = "base_link"

# Cone position
CONE_X = 0.90
CONE_Y = 3.00

# Orbit tuning
TARGET_RADIUS = 0.60
LINEAR_SPEED = 0.10

BASE_GAIN = 0.6              # small gain
DEADBAND = 0.05              # ignore small error
MAX_CORRECTION = 0.25        # limit correction

CLOCKWISE = True
CMD_RATE = 5.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class OrbitTwistCorrectedNode(Node):

    def __init__(self):
        super().__init__("orbit_refined")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sign = -1.0 if CLOCKWISE else 1.0
        self.timer = self.create_timer(1.0 / CMD_RATE, self.loop)

        self.get_logger().info("Refined orbit started")

    def loop(self):

        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                BASE_FRAME,
                Time(),
                timeout=Duration(seconds=0.2)
            )
        except Exception:
            return

        xr = tf.transform.translation.x
        yr = tf.transform.translation.y

        dx = xr - CONE_X
        dy = yr - CONE_Y
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            return

        # --- Base orbit ---
        base_omega = self.sign * (LINEAR_SPEED / TARGET_RADIUS)

        # --- Error ---
        error = dist - TARGET_RADIUS

        # --- Apply deadband ---
        if abs(error) < DEADBAND:
            correction = 0.0
        else:
            correction = BASE_GAIN * error
            correction = clamp(correction, -MAX_CORRECTION, MAX_CORRECTION)

        # --- Combine ---
        omega = base_omega + self.sign * correction

        msg = Twist()
        msg.linear.x = LINEAR_SPEED
        msg.angular.z = omega
        self.pub.publish(msg)

        self.get_logger().info(
            f"dist={dist:.3f}, err={error:.3f}, corr={correction:.3f}, w={omega:.3f}"
        )


def main():
    rclpy.init()
    node = OrbitTwistCorrectedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()