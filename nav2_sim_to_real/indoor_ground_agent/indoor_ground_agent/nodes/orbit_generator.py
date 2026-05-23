import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from ros_gz_interfaces.srv import SpawnEntity
from geometry_msgs.msg import Pose

class OrbitGenerator(Node):
    def __init__(self):
        super().__init__('orbit_generator')

        self.client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')

        # ----- Obstacle geometry (known) -----
        self.cx = 1.0
        self.cy = 1.88
        self.obstacle_radius = 0.25

        # ----- Costmap-ish geometry (from your YAML) -----
        self.inflation_radius = 0.4
        self.safety_margin = 0.25  # start here

        # ----- Orbit behavior -----
        self.clockwise = True
        self.desired_spacing = 0.30  # meters between waypoints
        self.min_points = 8

        # ----- Radius search -----
        self.radius_shrink_step = 0.05  # meters
        self.max_shrink_iters = 30

        # ----- Map cache -----
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.map_msg = None
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)

        self.get_logger().info("Waiting for /map ...")

        # -- spawn temp markers
        self.spawned = False
        self.spawn_client = self.create_client(
            SpawnEntity,
            '/world/indoor_world/create'
        )

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Fortress spawn service...")

    def _on_map(self, msg: OccupancyGrid):
        if self.map_msg is None:
            self.map_msg = msg
            self.get_logger().info(f"Got /map ({msg.info.width}x{msg.info.height} @ {msg.info.resolution} m/px)")

            if self.spawned:
                return

            self.spawned = True
            self.send_orbit()

    # ----------------------------
    # Geometry helpers
    # ----------------------------
    def _compute_radius_nominal(self) -> float:
        # Respect obstacle + inflation + margin. (Robot radius not in your costmap config.)
        return self.obstacle_radius + self.inflation_radius + self.safety_margin

    def _map_bounds(self):
        info = self.map_msg.info
        res = info.resolution
        xmin = info.origin.position.x
        ymin = info.origin.position.y
        xmax = xmin + info.width * res
        ymax = ymin + info.height * res
        return xmin, ymin, xmax, ymax

    def _clamp_radius_to_map(self, r: float) -> float:
        xmin, ymin, xmax, ymax = self._map_bounds()

        # keep a margin away from map edge
        edge_margin = 0.10

        r_max = min(
            self.cx - (xmin + edge_margin),
            (xmax - edge_margin) - self.cx,
            self.cy - (ymin + edge_margin),
            (ymax - edge_margin) - self.cy
        )
        return min(r, r_max)

    def _compute_num_points_circle(self, r: float) -> int:
        circumference = 2.0 * math.pi * r
        n = int(math.ceil(circumference / self.desired_spacing))
        return max(self.min_points, n)

    def _yaw_tangent(self, angle: float) -> float:
        # Tangent heading: forward along motion direction
        if self.clockwise:
            return angle - math.pi / 2.0
        return angle + math.pi / 2.0

    # ----------------------------
    # Map validity check
    # ----------------------------
    def _is_free_cell(self, x: float, y: float) -> bool:
        msg = self.map_msg
        info = msg.info
        res = info.resolution

        # Convert world -> map pixel
        mx = int((x - info.origin.position.x) / res)
        my = int((y - info.origin.position.y) / res)

        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return False

        idx = my * info.width + mx
        val = msg.data[idx]

        # Treat unknown as invalid for orbiting
        if val < 0:
            return False

        # 0..100: free..occupied. Consider occupied > 50 invalid.
        return val <= 50

    def _generate_circle_poses(self, r: float):
        poses = []
        n = self._compute_num_points_circle(r)

        for i in range(n):
            angle = (2.0 * math.pi / n) * i
            if self.clockwise:
                angle = -angle

            x = self.cx + r * math.cos(angle)
            y = self.cy + r * math.sin(angle)

            yaw = self._yaw_tangent(angle)

            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = rclpy.time.Time().to_msg()  # “latest”
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)

            poses.append(ps)

        return poses

    def _all_poses_valid(self, poses) -> bool:
        for p in poses:
            if not self._is_free_cell(p.pose.position.x, p.pose.position.y):
                return False
        return True

    # ----------------------------
    # Main
    # ----------------------------
    def send_orbit(self):
        if self.map_msg is None:
            return

        self.client.wait_for_server()

        r = self._compute_radius_nominal()
        r = self._clamp_radius_to_map(r)

        if r <= 0.0:
            self.get_logger().error("Computed orbit radius <= 0 after clamping to map bounds.")
            return

        chosen = None
        for k in range(self.max_shrink_iters):
            poses = self._generate_circle_poses(r)
            if self._all_poses_valid(poses):
                chosen = (r, poses)
                break
            r -= self.radius_shrink_step
            if r <= 0.2:
                break

        if chosen is None:
            self.get_logger().error("Could not find a valid orbit radius (all candidates hit occupied/unknown cells).")
            return

        r_final, poses_final = chosen
        self.get_logger().info(f"Orbit radius chosen: {r_final:.2f} m | waypoints: {len(poses_final)} | clockwise={self.clockwise}")

        self.get_logger().info("Generated orbit waypoints:")

        for i, p in enumerate(poses):
            self.get_logger().info(
                f"[{i:02d}] x={p.pose.position.x:.3f}, "
                f"y={p.pose.position.y:.3f}"
            )

        # Extract coordinates
        coords = [(p.pose.position.x, p.pose.position.y) for p in poses]

        self.get_logger().info("Spawning orbit markers...")

        for idx, (x, y) in enumerate(coords):
            self.spawn_marker(x, y, idx)
            self.get_logger().info(f"Spawned orbit marker - {idx} at {x}, {y}")
            # goal = NavigateThroughPoses.Goal()
            # goal.poses = poses_final
            # self.client.send_goal_async(goal)

    def spawn_marker(self, x, y, idx):

        sdf = f"""
        <sdf version="1.7">
        <model name="orbit_marker_{idx}">
            <static>true</static>
            <link name="link">
            <visual name="dot_visual">
                <geometry>
                <cylinder>
                    <radius>0.05</radius>
                    <length>0.01</length>
                </cylinder>
                </geometry>
                <material>
                <ambient>1 0 0 1</ambient>
                <diffuse>1 0 0 1</diffuse>
                </material>
            </visual>
            </link>
        </model>
        </sdf>
        """

        req = SpawnEntity.Request()

        req.entity_factory.name = f"orbit_marker_{idx}"
        req.entity_factory.sdf = sdf
        req.entity_factory.allow_renaming = True

        req.entity_factory.pose.position.x = float(x)
        req.entity_factory.pose.position.y = float(y)
        req.entity_factory.pose.position.z = 0.02

        req.entity_factory.relative_to = "world"

        future = self.spawn_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Spawned orbit marker at {x:.2f}, {y:.2f}"
            )
        )

def main(args=None):
    rclpy.init(args=args)
    node = OrbitGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()