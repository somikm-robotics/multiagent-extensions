#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.time import Time



from std_msgs.msg import Float32, Bool, Empty
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from rcl_interfaces.srv import SetParameters

from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav2_msgs.action import NavigateToPose

from tf_transformations import quaternion_from_euler

import tf2_ros
import math

# =========================
# Constants (edit these)
# =========================

# Hazard center (map frame)
HAZARD_X = 1.0
HAZARD_Y = 1.5
HAZARD_RADIUS = 0.25

# Orbit behavior
SAFETY_MARGIN = 0.55                 # meters
ORBIT_RADIUS = HAZARD_RADIUS + SAFETY_MARGIN
DIRECTION = "clockwise"              # "clockwise" or "counter_clockwise"
CAPTURE_INTERVAL_DEG = 30.0          # trigger capture every 30 degrees

# Nav2 / TF wiring
CONTROLLER_SERVER_SET_PARAMS_SRV = "/controller_server/set_parameters"
FOLLOW_PATH_ACTION = "/follow_path"
ORBIT_CONTROLLER_ID = "Orbit"
GOAL_CHECKER_ID = "goal_checker"   # must exist in your nav2 params (or can be left empty)
PROGRESS_CHECKER_ID = "progress_checker"   # only used if supported by your Nav2 distro

MAP_FRAME = "map"
BASE_FRAME = "agilex_diff_drive/base_link"
TF_TIMEOUT_SEC = 0.5

# Topics
ORBIT_ANGLE_TOPIC = "/orbit_angle"
ORBIT_DONE_TOPIC = "/orbit_done"
CAPTURE_IMAGE_TOPIC = "/capture_image"


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

def quat_from_yaw(yaw: float):
    """Return quaternion tuple (x,y,z,w) from yaw."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def compute_staging_pose(tf_buffer, cx, cy, r, direction: str, logger):

    # --------------------------------------------------
    # 1️⃣ Ensure transform is available
    # --------------------------------------------------
    if not tf_buffer.can_transform(
        MAP_FRAME,
        BASE_FRAME,
        Time(),
        timeout=Duration(seconds=2.0)
    ):
        return None

    tf = tf_buffer.lookup_transform(
        MAP_FRAME,
        BASE_FRAME,
        Time(),
        timeout=Duration(seconds=2.0),
    )

    # Robot position in map frame
    xr = tf.transform.translation.x
    yr = tf.transform.translation.y

    # --------------------------------------------------
    # 2️⃣ Compute radial direction (hazard → robot)
    # --------------------------------------------------
    dx = xr - cx
    dy = yr - cy
    d = math.hypot(dx, dy)

    if d < 1e-6:
        return None  # extremely unlikely, but safe guard

    ux = dx / d
    uy = dy / d

    # --------------------------------------------------
    # 3️⃣ Choose safe staging radius
    # --------------------------------------------------
    inflation_radius = 0.35   # match your Nav2 YAML
    safety_margin = 0.6       # comfortable clearance for staging

    # r_entry = r + inflation_radius + safety_margin

    entry_margin = 1.2
    r_entry = r + entry_margin

    # --------------------------------------------------
    # 4️⃣ Compute staging point (stay on same side)
    # --------------------------------------------------
    x_s = cx + r_entry * ux
    y_s = cy + r_entry * uy

    # --------------------------------------------------
    # 5️⃣ Compute tangential heading
    # --------------------------------------------------
    if direction.lower() in ("clockwise", "cw"):
        # Tangent vector for clockwise
        tx =  uy
        ty = -ux
    else:
        # Tangent vector for counter-clockwise
        tx = -uy
        ty =  ux
    
    dx = x_s - cx
    dy = y_s - cy

    # Tangential heading for clockwise orbit
    tx = -dy
    ty = dx

    yaw = math.atan2(ty, tx) + math.pi
    yaw = normalize_angle(yaw)


    logger.info(f"Orbit center = ({cx:.2f},{cy:.2f})")
    logger.info(f"Entry radius = {r_entry:.2f}")
    logger.info(f"Computed staging = ({x_s:.2f},{y_s:.2f})")

    return x_s, y_s, yaw

class OrbitManager(Node):
    """
    Starts a 1-rev orbit by:
      1) setting Orbit plugin params
      2) sending FollowPath goal with controller_id='Orbit'
    Triggers /capture_image every CAPTURE_INTERVAL_DEG based on /orbit_angle.
    Cancels FollowPath when /orbit_done is received and exits.
    """

    def __init__(self):
        super().__init__("orbit_manager")

        # self.declare_parameter("use_sim_time", True)
        # Internal state
        self._last_capture_deg = 0.0
        self._orbit_done = False
        self._active_goal_handle: Optional[object] = None

        # staging
        self._staging_x = 0.0
        self._staging_y = 0.0
        self._staging_yaw = 0.0

        # Pub/Sub
        # self.capture_pub = self.create_publisher(Empty, CAPTURE_IMAGE_TOPIC, 10)
        self.create_subscription(Float32, ORBIT_ANGLE_TOPIC, self._angle_cb, 10)
        self.create_subscription(Bool, ORBIT_DONE_TOPIC, self._done_cb, 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Service client
        self.set_params_client = self.create_client(SetParameters, CONTROLLER_SERVER_SET_PARAMS_SRV)

        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.follow_path_client = ActionClient(self, FollowPath, FOLLOW_PATH_ACTION)


        # Start sequence once after node is up
        self._started = False
        self.create_timer(0.1, self._start_once)

    def _start_once(self):
        if self._started:
            return
        # self._started = True
        self.start_orbit()

    # ---------------- Orbit sequence ----------------

    def start_orbit(self):
        self.get_logger().info("Trying to Navigate to orbit staging position...")
        self._navigate_to_staging_pose()
        # self._set_orbit_params(HAZARD_X, HAZARD_Y, ORBIT_RADIUS, "clockwise")
        

    # def _set_params_done_cb(self, future):
    #     resp = future.result()

    #     if not all(r.successful for r in resp.results):
    #         self.get_logger().error("SetParameters rejected.")
    #         rclpy.shutdown()
    #         return

    #     self.get_logger().info("Orbit parameters set successfully.")
    #     self._started = True
    #     self._send_follow_path_goal()

    def _set_orbit_params(self, cx: float, cy: float, r: float, direction: str) -> bool:
        if not self.set_params_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Service not available: {CONTROLLER_SERVER_SET_PARAMS_SRV}")
            return False

        req = SetParameters.Request()
        # Parameter names must match your plugin namespace: "Orbit.*"
        req.parameters = [
            Parameter("Orbit.center_x", Parameter.Type.DOUBLE, cx).to_parameter_msg(),
            Parameter("Orbit.center_y", Parameter.Type.DOUBLE, cy).to_parameter_msg(),
            Parameter("Orbit.radius",   Parameter.Type.DOUBLE, r).to_parameter_msg(),
            Parameter("Orbit.direction", Parameter.Type.STRING, direction).to_parameter_msg(),
        ]

        future = self.set_params_client.call_async(req)
        future.add_done_callback(self._set_params_done_cb)
        
    def _set_params_done_cb(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"SetParameters call failed: {e}")
            rclpy.shutdown()
            return

        if not all(r.successful for r in resp.results):
            self.get_logger().error("SetParameters rejected.")
            rclpy.shutdown()
            return

        self.get_logger().info("Orbit parameters set successfully.")
        self._send_follow_path_goal()

    def _navigate_to_staging_pose(self):

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action not available")
            return

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        # goal.pose.pose.position.x = 1.0
        # goal.pose.pose.position.y = 0.7
        # goal.pose.pose.position.z = 0.0

        # # Facing +X direction (yaw = 0)
        # goal.pose.pose.orientation.w = 1.0
        # goal.pose.pose.orientation.z = 0.0
        result = compute_staging_pose(self.tf_buffer, HAZARD_X, HAZARD_Y, HAZARD_RADIUS, DIRECTION, self.get_logger())

        if result is None:
            self.get_logger().warn("TF not ready yet. Retrying staging computation...")
            return   # <-- simply exit and let timer call again

        
        self._started = True
        x_s, y_s, yaw = result
        self._staging_x = x_s
        self._staging_y = y_s
        self._staging_yaw = yaw
        self.get_logger().info(f"TF ready now...Navigating to Staging pose now - {x_s}, {y_s}")

        goal.pose.pose.position.x = x_s
        goal.pose.pose.position.y = y_s

        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)

        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("NavigateToPose rejected")
            return

        self.get_logger().info("Navigating to staging pose...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    # def _nav_result_cb(self, future):
    #     result = future.result()

    #     if result.status != 4:  # 4 = SUCCEEDED
    #         self.get_logger().error("Navigation failed")
    #         return

    #     self.get_logger().info("Reached staging pose. Starting orbit.")

    #     self._set_orbit_params(HAZARD_X, HAZARD_Y, ORBIT_RADIUS, "clockwise")
            

    def _nav_result_cb(self, future):

        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().error(f"Navigation result error: {e}")
            return

        # Get robot current pose in map frame
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

        dx = xr - self._staging_x
        dy = yr - self._staging_y
        dist = math.hypot(dx, dy)

        self.get_logger().info(f"Distance to staging: {dist:.3f} m")

        # 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info("Reached staging pose. Starting orbit.")
            self._set_orbit_params(HAZARD_X, HAZARD_Y, ORBIT_RADIUS, "clockwise")
            return

        # If navigation failed but robot is close enough
        if dist < 0.25:
            self.get_logger().warn(
                "Navigation failed but close enough to staging. Starting orbit."
            )
            self._set_orbit_params(HAZARD_X, HAZARD_Y, ORBIT_RADIUS, "clockwise")
            return

        # Otherwise real failure
        self.get_logger().error("Navigation failed and too far from staging.")

    def _send_follow_path_goal(self):
        if not self.follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"FollowPath action not available: {FOLLOW_PATH_ACTION}")
            rclpy.shutdown()
            return

        path = self._make_minimal_path_in_map()
        if path is None:
            self.get_logger().warn("TF not ready yet. Retrying...")
            self.create_timer(0.5, self.start_orbit)
            return

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = ORBIT_CONTROLLER_ID

        # Keep these if your Nav2 config has them; otherwise set empty strings.
        if hasattr(goal, "goal_checker_id"):
            goal.goal_checker_id = GOAL_CHECKER_ID
        if hasattr(goal, "progress_checker_id"):
            goal.progress_checker_id = PROGRESS_CHECKER_ID

        self.get_logger().info(f"Sending FollowPath goal with controller_id='{ORBIT_CONTROLLER_ID}' ...")
        # send_future = self.follow_path_client.send_goal_async(goal)
        # rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        send_future = self.follow_path_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

        # goal_handle = send_future.result()
        # if goal_handle is None or not goal_handle.accepted:
        #     self.get_logger().error("FollowPath goal rejected.")
        #     rclpy.shutdown()
        #     return

        # self._active_goal_handle = goal_handle
        # self.get_logger().info("Orbit started.")

        # (Optional) fire a capture immediately at 0°
        # self.capture_pub.publish(Empty())
        # self.get_logger().info("Capture trigger @ start (0°)")

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Send goal call failed: {e}")
            rclpy.shutdown()
            return

        if not goal_handle.accepted:
            self.get_logger().error("FollowPath goal rejected by controller_server.")
            rclpy.shutdown()
            return

        self.get_logger().info("FollowPath goal accepted. Orbit started.")
        self._active_goal_handle = goal_handle

        # Now register callback for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        try:
            result = future.result().result
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f"Error getting FollowPath result: {e}")
            return

        self.get_logger().info(f"FollowPath finished with status: {status}")
        rclpy.shutdown()

    def _make_minimal_path_in_map(self) -> Optional[Path]:
        try:
            # Wait until transform becomes available
            if not self.tf_buffer.can_transform(
                MAP_FRAME,
                BASE_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0)
            ):
                self.get_logger().warn("Transform not yet available, waiting...")
                return None

            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                BASE_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0),
            )

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

        offset = 2.0   # NOT 0.3 or 0.5

        yaw = self._staging_yaw

        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(tf.transform.translation.x) + offset * math.cos(yaw)
        pose.pose.position.y = float(tf.transform.translation.y) + offset * math.sin(yaw)
        pose.pose.position.z = 0.0

        # Orientation doesn't matter for Orbit controller; set yaw=0
        qx, qy, qz, qw = quat_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        path = Path()
        path.header = pose.header
        # Provide at least 2 poses to satisfy validators in some setups
        path.poses = [pose, pose]
        return path

    # ---------------- Callbacks ----------------

    def _angle_cb(self, msg: Float32):
        deg = float(msg.data)

        # Trigger every CAPTURE_INTERVAL_DEG
        if (deg - self._last_capture_deg) >= (CAPTURE_INTERVAL_DEG - 1e-6):
            # self.capture_pub.publish(Empty())
            # self.get_logger().info(f"Capture trigger @ {deg:.1f}°")
            self._last_capture_deg = deg

    def _done_cb(self, msg: Bool):
        if not msg.data or self._orbit_done:
            return
        self._orbit_done = True
        self.get_logger().info("orbit_done received. Cancelling FollowPath to return to normal navigation.")
        self._cancel_follow_path_and_exit()

    def _cancel_follow_path_and_exit(self):
        if self._active_goal_handle is None:
            self.get_logger().warn("No active FollowPath goal to cancel. Exiting anyway.")
            rclpy.shutdown()
            return

        # cancel_future = self._active_goal_handle.cancel_goal_async()
        # rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=10.0)
        cancel_future = self._active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_cb)

        # self.get_logger().info("FollowPath cancelled. Orbit complete. Exiting Orbiting_Manager.")
        # rclpy.shutdown()

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

        self.get_logger().info("Orbit complete. Exiting Orbiting_Manager.")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = OrbitManager()
    rclpy.spin(node)


if __name__ == "__main__":
    main()