import math
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from tf2_ros import Buffer, TransformException, TransformListener

from .common import (
    Pose2D,
    distance_point_to_line,
    min_range,
    normalize_angle,
    project_along_line,
    yaw_from_quaternion,
)


class Mode(Enum):
    FOLLOW_MLINE = auto()
    BUG_CIRCUMFILTRATE = auto()
    STOP_AT_BOUNDARY = auto()
    SHIFT_LATERAL = auto()
    TURN_180 = auto()
    FINISHED = auto()


class CorridorShuttleBug2(Node):
    """Reactive corridor coverage controller.

    This node publishes /cmd_vel directly for the corridor pass. In production, use it as a
    Behavior Tree action or gate it behind a safety mux so Nav2 collision monitoring can stop it.
    """

    def __init__(self) -> None:
        super().__init__("corridor_shuttle_bug2")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("front_glass_topic", "/front_low_range")
        self.declare_parameter("wall_side", "left")
        self.declare_parameter("desired_wall_distance", 0.35)
        self.declare_parameter("front_stop_distance", 0.20)
        self.declare_parameter("side_stop_distance", 0.22)
        self.declare_parameter("linear_speed", 0.16)
        self.declare_parameter("turn_speed", 0.45)
        self.declare_parameter("corridor_length", 8.0)
        self.declare_parameter("mop_width", 0.42)
        self.declare_parameter("overlap", 0.08)
        self.declare_parameter("max_lanes", 20)
        self.declare_parameter("mline_tolerance", 0.12)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.wall_side = self.get_parameter("wall_side").value
        self.desired_wall_distance = float(self.get_parameter("desired_wall_distance").value)
        self.front_stop_distance = float(self.get_parameter("front_stop_distance").value)
        self.side_stop_distance = float(self.get_parameter("side_stop_distance").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.corridor_length = float(self.get_parameter("corridor_length").value)
        self.lane_shift = float(self.get_parameter("mop_width").value) - float(self.get_parameter("overlap").value)
        self.max_lanes = int(self.get_parameter("max_lanes").value)
        self.mline_tolerance = float(self.get_parameter("mline_tolerance").value)

        self.scan: Optional[LaserScan] = None
        self.low_front_range = float("inf")
        self.odom_pose: Optional[Pose2D] = None
        self.start_pose: Optional[Pose2D] = None
        self.lane_start: Optional[Tuple[float, float]] = None
        self.lane_goal: Optional[Tuple[float, float]] = None
        self.hit_projection = 0.0
        self.mode = Mode.FOLLOW_MLINE
        self.lane_index = 0
        self.direction = 1.0
        self.shift_start: Optional[Pose2D] = None
        self.turn_target_yaw = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.on_scan, 10)
        self.create_subscription(Range, self.get_parameter("front_glass_topic").value, self.on_low_range, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.05, self.control_loop)

    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def on_low_range(self, msg: Range) -> None:
        self.low_front_range = msg.range if math.isfinite(msg.range) else float("inf")

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_pose = Pose2D(p.x, p.y, yaw)

    def control_loop(self) -> None:
        pose = self.lookup_robot_pose()
        if pose is None or self.scan is None:
            return
        if self.start_pose is None:
            self.initialize_lane(pose)

        sectors = self.scan_sectors()
        if self.mode == Mode.FOLLOW_MLINE:
            cmd = self.follow_mline(pose, sectors)
        elif self.mode == Mode.BUG_CIRCUMFILTRATE:
            cmd = self.bug_follow_obstacle(pose, sectors)
        elif self.mode == Mode.STOP_AT_BOUNDARY:
            cmd = self.stop_then_start_shift(pose)
        elif self.mode == Mode.SHIFT_LATERAL:
            cmd = self.shift_lateral(pose, sectors)
        elif self.mode == Mode.TURN_180:
            cmd = self.turn_180(pose)
        else:
            cmd = Twist()
        self.cmd_pub.publish(cmd)

    def lookup_robot_pose(self) -> Optional[Pose2D]:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return self.odom_pose
        q = tf.transform.rotation
        return Pose2D(tf.transform.translation.x, tf.transform.translation.y, yaw_from_quaternion(q))

    def initialize_lane(self, pose: Pose2D) -> None:
        self.start_pose = pose
        self.lane_start = (pose.x, pose.y)
        self.lane_goal = (
            pose.x + self.direction * self.corridor_length * math.cos(pose.yaw),
            pose.y + self.direction * self.corridor_length * math.sin(pose.yaw),
        )
        self.get_logger().info("Starting corridor shuttle coverage.")

    def scan_sectors(self) -> Dict[str, float]:
        assert self.scan is not None
        ranges = self.scan.ranges

        def sector(deg_min: float, deg_max: float) -> float:
            values: List[float] = []
            for i, value in enumerate(ranges):
                angle = self.scan.angle_min + i * self.scan.angle_increment
                deg = math.degrees(angle)
                if deg_min <= deg <= deg_max:
                    values.append(value)
            return min_range(values)

        front_lidar = min(sector(-12, 12), sector(348, 360))
        return {
            "front": min(front_lidar, self.low_front_range),
            "front_lidar": front_lidar,
            "left": sector(65, 115),
            "right": sector(-115, -65),
            "front_left": sector(20, 65),
            "front_right": sector(-65, -20),
        }

    def follow_mline(self, pose: Pose2D, sectors: Dict[str, float]) -> Twist:
        if self.boundary_reached(pose, sectors):
            self.mode = Mode.STOP_AT_BOUNDARY
            return Twist()
        if sectors["front"] < self.front_stop_distance:
            self.hit_projection = self.current_projection(pose)
            self.mode = Mode.BUG_CIRCUMFILTRATE
            return Twist()

        side = sectors["left"] if self.wall_side == "left" else sectors["right"]
        sign = 1.0 if self.wall_side == "left" else -1.0
        wall_error = side - self.desired_wall_distance

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = sign * 1.2 * wall_error
        return cmd

    def bug_follow_obstacle(self, pose: Pose2D, sectors: Dict[str, float]) -> Twist:
        if self.lane_start and self.lane_goal:
            on_mline = distance_point_to_line(pose.x, pose.y, self.lane_start, self.lane_goal) < self.mline_tolerance
            progressed = self.current_projection(pose) > self.hit_projection + 0.08
            front_clear = sectors["front"] > self.front_stop_distance * 2.0
            if on_mline and progressed and front_clear:
                self.mode = Mode.FOLLOW_MLINE

        cmd = Twist()
        follow_left = self.wall_side == "left"
        obstacle_side = sectors["left"] if follow_left else sectors["right"]
        front_diag = sectors["front_left"] if follow_left else sectors["front_right"]
        sign = 1.0 if follow_left else -1.0

        if sectors["front"] < self.front_stop_distance or front_diag < self.side_stop_distance:
            cmd.angular.z = -sign * self.turn_speed
        else:
            error = obstacle_side - self.desired_wall_distance
            cmd.linear.x = self.linear_speed * 0.75
            cmd.angular.z = sign * 1.4 * error
        return cmd

    def stop_then_start_shift(self, pose: Pose2D) -> Twist:
        self.shift_start = pose
        self.mode = Mode.SHIFT_LATERAL
        return Twist()

    def shift_lateral(self, pose: Pose2D, sectors: Dict[str, float]) -> Twist:
        if self.shift_start is None:
            self.shift_start = pose
        if sectors["left"] < self.side_stop_distance and sectors["right"] < self.side_stop_distance:
            self.mode = Mode.FINISHED
            return Twist()

        moved = math.hypot(pose.x - self.shift_start.x, pose.y - self.shift_start.y)
        if moved >= self.lane_shift:
            self.turn_target_yaw = normalize_angle(pose.yaw + math.pi)
            self.mode = Mode.TURN_180
            return Twist()

        cmd = Twist()
        cmd.linear.x = 0.08
        cmd.angular.z = self.turn_speed if self.wall_side == "left" else -self.turn_speed
        return cmd

    def turn_180(self, pose: Pose2D) -> Twist:
        error = normalize_angle(self.turn_target_yaw - pose.yaw)
        if abs(error) < 0.06:
            self.lane_index += 1
            if self.lane_index >= self.max_lanes:
                self.mode = Mode.FINISHED
                return Twist()
            self.direction *= -1.0
            self.lane_start = (pose.x, pose.y)
            self.lane_goal = (
                pose.x + self.direction * self.corridor_length * math.cos(pose.yaw),
                pose.y + self.direction * self.corridor_length * math.sin(pose.yaw),
            )
            self.mode = Mode.FOLLOW_MLINE
            return Twist()

        cmd = Twist()
        cmd.angular.z = self.turn_speed if error > 0.0 else -self.turn_speed
        return cmd

    def boundary_reached(self, pose: Pose2D, sectors: Dict[str, float]) -> bool:
        lidar_wall = sectors["front_lidar"] < self.front_stop_distance
        odom_limit = self.current_projection(pose) >= 1.0
        boxed_in = (
            sectors["front"] < self.front_stop_distance
            and sectors["left"] < self.side_stop_distance
            and sectors["right"] < self.side_stop_distance
        )
        return lidar_wall or odom_limit or boxed_in

    def current_projection(self, pose: Pose2D) -> float:
        if not self.lane_start or not self.lane_goal:
            return 0.0
        return project_along_line(pose.x, pose.y, self.lane_start, self.lane_goal)


def main() -> None:
    rclpy.init()
    node = CorridorShuttleBug2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
