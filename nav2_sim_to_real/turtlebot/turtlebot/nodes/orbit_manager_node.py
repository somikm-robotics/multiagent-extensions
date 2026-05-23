#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from rcl_interfaces.srv import SetParameters

from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath, NavigateToPose

import tf2_ros
from tf_transformations import quaternion_from_euler


# ==========================================================
# EDIT THESE FOR THE REAL ROBOT TEST
# ==========================================================

# Cone / obstacle center in MAP frame
HAZARD_X = 2.55
HAZARD_Y = -1.30
HAZARD_RADIUS = 0.15scr

# Orbit behavior
ORBIT_RADIUS = 0.70
DIRECTION = "clockwise"   # keep clockwise for RHS camera
LINEAR_VEL = 0.08
GAIN = 0.5

# Staging
STAGING_EXTRA_MARGIN = 0.10   # extra clearance beyond orbit radius
STAGING_SUCCESS_DIST = 0.25

# Nav2 / TF
MAP_FRAME = "map"
BASE_FRAME = "base_link"
TF_TIMEOUT_SEC = 2.0

FOLLOW_PATH_ACTION = "/follow_path"
NAVIGATE_TO_POSE_ACTION = "/navigate_to_pose"
CONTROLLER_SERVER_SET_PARAMS_SRV = "/controller_server/set_parameters"

ORBIT_CONTROLLER_ID = "Orbit"
GOAL_CHECKER_ID = "goal_checker"
PROGRESS_CHECKER_ID = "progress_checker"

# Topics from your OrbitController plugin
ORBIT_ANGLE_TOPIC = "/orbit_angle"
ORBIT_DONE_TOPIC = "/orbit_done"


# ==========================================================
# Helpers
# ==========================================================

def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_from_yaw(yaw: float):
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    return qx, qy, qz, qw


def compute_staging_pose(
    tf_buffer: tf2_ros.Buffer,
    cx: float,
    cy: float,
    orbit_radius: float,
    direction: str,
    logger
):
    """
    Choose staging point on the same radial side as current robot position,
    then orient robot tangentially for requested orbit direction.
    """

    if not tf_buffer.can_transform(
        MAP_FRAME,
        BASE_FRAME,
        Time(),
        timeout=Duration(seconds=TF_TIMEOUT_SEC)
    ):
        return None

    tf = tf_buffer.lookup_transform(
        MAP_FRAME,
        BASE_FRAME,
        Time(),
        timeout=Duration(seconds=TF_TIMEOUT_SEC)
    )

    xr = tf.transform.translation.x
    yr = tf.transform.translation.y

    dx = xr - cx
    dy = yr - cy
    d = math.hypot(dx, dy)

    if d < 1e-6:
        logger.warn("Robot is too close to obstacle center to compute staging pose.")
        return None

    ux = dx / d
    uy = dy / d

    staging_radius = orbit_radius + STAGING_EXTRA_MARGIN
    x_s = cx + staging_radius * ux
    y_s = cy + staging_radius * uy

    # Tangent direction at staging point
    # radial vector from center to staging = (ux, uy)
    if direction.lower() in ("clockwise", "cw"):
        tx = uy
        ty = -ux
    else:
        tx = -uy
        ty = ux

    yaw = math.atan2(ty, tx)
    yaw = normalize_angle(yaw)

    logger.info(f"Orbit center = ({cx:.2f}, {cy:.2f})")
    logger.info(f"Orbit radius = {orbit_radius:.2f}")
    logger.info(f"Staging radius = {staging_radius:.2f}")
    logger.info(f"Computed staging = ({x_s:.2f}, {y_s:.2f}), yaw={yaw:.2f}")

    return x_s, y_s, yaw


# ==========================================================
# Node
# ==========================================================

class OrbitManagerNode(Node):
    def __init__(self):
        super().__init__("orbit_manager")

        self._started = False
        self._orbit_done = False
        self._active_goal_handle = None
        self._last_capture_deg = 0.0

        self._staging_x = 0.0
        self._staging_y = 0.0
        self._staging_yaw = 0.0

        self.create_subscription(Float32, ORBIT_ANGLE_TOPIC, self._angle_cb, 10)
        self.create_subscription(Bool, ORBIT_DONE_TOPIC, self._done_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.set_params_client = self.create_client(
            SetParameters,
            CONTROLLER_SERVER_SET_PARAMS_SRV
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            NAVIGATE_TO_POSE_ACTION
        )

        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            FOLLOW_PATH_ACTION
        )

        self.create_timer(0.2, self._start_once)

    def _start_once(self):
        if self._started:
            return
        self._started = True
        self.start_orbit()
        

    def start_orbit(self):
        # self.get_logger().info("Starting orbit sequence: Nav2 -> staging -> Orbit controller")
        self.get_logger().info("Starting orbit sequence: Nav2 -> Orbit controller")
       # self._navigate_to_staging_pose()
        self._set_orbit_params()

    # ------------------------------------------------------
    # Step 1: navigate to staging
    # ------------------------------------------------------

    def _navigate_to_staging_pose(self):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action not available")
            return

        result = compute_staging_pose(
            self.tf_buffer,
            HAZARD_X,
            HAZARD_Y,
            ORBIT_RADIUS,
            DIRECTION,
            self.get_logger()
        )

        if result is None:
            self.get_logger().warn("TF not ready yet. Retrying staging computation...")
            self._started = False
            return

        x_s, y_s, yaw = result
        self._staging_x = x_s
        self._staging_y = y_s
        self._staging_yaw = yaw

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x_s
        goal.pose.pose.position.y = y_s
        goal.pose.pose.position.z = 0.0

        qx, qy, qz, qw = quat_from_yaw(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f"Navigating to staging pose ({x_s:.2f}, {y_s:.2f})")

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"NavigateToPose send failed: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().error("NavigateToPose rejected")
            return

        self.get_logger().info("NavigateToPose accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as e:
            self.get_logger().error(f"Navigation result error: {e}")
            return

        if not self.tf_buffer.can_transform(
            MAP_FRAME,
            BASE_FRAME,
            Time(),
            timeout=Duration(seconds=1.0)
        ):
            self.get_logger().error("TF unavailable when checking staging distance.")
            return

        tf = self.tf_buffer.lookup_transform(
            MAP_FRAME,
            BASE_FRAME,
            Time(),
            timeout=Duration(seconds=1.0)
        )

        xr = tf.transform.translation.x
        yr = tf.transform.translation.y
        dist = math.hypot(xr - self._staging_x, yr - self._staging_y)

        self.get_logger().info(f"Distance to staging: {dist:.3f} m (status={status})")

        # 4 = SUCCEEDED
        if status == 4 or dist < STAGING_SUCCESS_DIST:
            if status != 4:
                self.get_logger().warn("Navigation did not report success, but robot is close enough to staging.")
            self.get_logger().info("Starting orbit controller.")
            self._set_orbit_params()
            return

        self.get_logger().error("Navigation failed and robot is too far from staging.")

    # ------------------------------------------------------
    # Step 2: configure orbit plugin
    # ------------------------------------------------------

    def _set_orbit_params(self):
        if not self.set_params_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Service not available: {CONTROLLER_SERVER_SET_PARAMS_SRV}")
            return

        req = SetParameters.Request()
        req.parameters = [
            Parameter("Orbit.center_x", Parameter.Type.DOUBLE, HAZARD_X).to_parameter_msg(),
            Parameter("Orbit.center_y", Parameter.Type.DOUBLE, HAZARD_Y).to_parameter_msg(),
            Parameter("Orbit.radius", Parameter.Type.DOUBLE, ORBIT_RADIUS).to_parameter_msg(),
            Parameter("Orbit.direction", Parameter.Type.STRING, DIRECTION).to_parameter_msg(),
            Parameter("Orbit.linear_vel", Parameter.Type.DOUBLE, LINEAR_VEL).to_parameter_msg(),
            Parameter("Orbit.gain", Parameter.Type.DOUBLE, GAIN).to_parameter_msg(),
            Parameter("Orbit.use_costmap_safety", Parameter.Type.BOOL, False).to_parameter_msg(),
        ]

        future = self.set_params_client.call_async(req)
        future.add_done_callback(self._set_params_done_cb)

    def _set_params_done_cb(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"SetParameters call failed: {e}")
            return

        if not all(r.successful for r in resp.results):
            self.get_logger().error("SetParameters rejected by controller_server.")
            return

        self.get_logger().info("Orbit parameters set successfully.")
        self._send_follow_path_goal()

    # ------------------------------------------------------
    # Step 3: trigger orbit controller through FollowPath
    # ------------------------------------------------------

    def _send_follow_path_goal(self):
        if not self.follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowPath action not available")
            return

        path = self._make_minimal_path_in_map()
        if path is None:
            self.get_logger().error("Could not build trigger path for Orbit controller.")
            return

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = ORBIT_CONTROLLER_ID

        if hasattr(goal, "goal_checker_id"):
            goal.goal_checker_id = GOAL_CHECKER_ID
        if hasattr(goal, "progress_checker_id"):
            goal.progress_checker_id = PROGRESS_CHECKER_ID

        self.get_logger().info(f"Sending FollowPath goal with controller_id='{ORBIT_CONTROLLER_ID}'")
        send_future = self.follow_path_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _make_minimal_path_in_map(self) -> Path:

        self.get_logger().info("Waiting for TF (map -> base_link)...")

        # BLOCK until TF is available
        while rclpy.ok():
            if self.tf_buffer.can_transform(
                MAP_FRAME,
                BASE_FRAME,
                Time(),
                timeout=Duration(seconds=0.2)
            ):
                break

            rclpy.spin_once(self, timeout_sec=0.1)

        # Now TF is guaranteed
        tf = self.tf_buffer.lookup_transform(
            MAP_FRAME,
            BASE_FRAME,
            Time(),
            timeout=Duration(seconds=1.0),
        )

        offset = 0.2   # small forward step (NOT 2.0 anymore)

        yaw = self._staging_yaw

        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(tf.transform.translation.x) + offset * math.cos(yaw)
        pose.pose.position.y = float(tf.transform.translation.y) + offset * math.sin(yaw)
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = quat_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        path = Path()
        path.header = pose.header
        path.poses = [pose, pose]

        return path

    def _make_minimal_path_in_map2(self) -> Optional[Path]:
        """
        Create a tiny forward tangent path from current pose.
        This is only to trigger controller_server to hand control to Orbit plugin.
        """

        try:
            if not self.tf_buffer.can_transform(
                MAP_FRAME,
                BASE_FRAME,
                Time(),
                timeout=Duration(seconds=2.0)
            ):
                return None

            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                BASE_FRAME,
                Time(),
                timeout=Duration(seconds=2.0)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed when making path: {e}")
            return None

        xr = tf.transform.translation.x
        yr = tf.transform.translation.y

        tangent_step = 0.20

        p0 = PoseStamped()
        p0.header.frame_id = MAP_FRAME
        p0.header.stamp = self.get_clock().now().to_msg()
        p0.pose.position.x = xr
        p0.pose.position.y = yr
        p0.pose.position.z = 0.0

        qx, qy, qz, qw = quat_from_yaw(self._staging_yaw)
        p0.pose.orientation.x = qx
        p0.pose.orientation.y = qy
        p0.pose.orientation.z = qz
        p0.pose.orientation.w = qw

        p1 = PoseStamped()
        p1.header = p0.header
        p1.pose.position.x = xr + tangent_step * math.cos(self._staging_yaw)
        p1.pose.position.y = yr + tangent_step * math.sin(self._staging_yaw)
        p1.pose.position.z = 0.0
        p1.pose.orientation = p0.pose.orientation

        path = Path()
        path.header = p0.header
        path.poses = [p0, p1]
        return path

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"FollowPath send failed: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().error("FollowPath goal rejected.")
            return

        self._active_goal_handle = goal_handle
        self.get_logger().info("FollowPath goal accepted. Orbit started.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as e:
            self.get_logger().error(f"Error getting FollowPath result: {e}")
            return

        self.get_logger().info(f"FollowPath finished with status: {status}")

    # ------------------------------------------------------
    # Orbit callbacks
    # ------------------------------------------------------

    def _angle_cb(self, msg: Float32):
        self._last_capture_deg = float(msg.data)

    def _done_cb(self, msg: Bool):
        if not msg.data or self._orbit_done:
            return

        self._orbit_done = True
        self.get_logger().info("orbit_done received. Cancelling FollowPath.")
        self._cancel_follow_path_and_exit()

    def _cancel_follow_path_and_exit(self):
        if self._active_goal_handle is None:
            self.get_logger().warn("No active FollowPath goal to cancel.")
            rclpy.shutdown()
            return

        cancel_future = self._active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_cb)

    def _cancel_done_cb(self, future):
        try:
            cancel_response = future.result()
        except Exception as e:
            self.get_logger().error(f"Cancel request failed: {e}")
            rclpy.shutdown()
            return

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("FollowPath goal successfully canceled.")
        else:
            self.get_logger().warn("No active goals were canceled.")

        self.get_logger().info("Orbit sequence complete.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = OrbitManagerNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()