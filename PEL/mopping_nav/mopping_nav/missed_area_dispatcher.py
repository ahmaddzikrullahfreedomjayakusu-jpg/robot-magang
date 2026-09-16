import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class MissedAreaDispatcher(Node):
    """Sends Nav2 to remaining unmopped cells after the shuttle pass."""

    def __init__(self) -> None:
        super().__init__("missed_area_dispatcher")
        self.declare_parameter("coverage_topic", "/mopping/coverage_grid")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("min_goal_spacing", 0.25)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.min_goal_spacing = float(self.get_parameter("min_goal_spacing").value)
        self.grid: Optional[OccupancyGrid] = None
        self.active = False
        self.goal_in_flight = False
        self.last_goal: Optional[Tuple[float, float]] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(OccupancyGrid, self.get_parameter("coverage_topic").value, self.on_grid, 10)
        self.create_subscription(Bool, "/mopping/cleanup_missed_cells", self.on_enable, 10)
        self.create_timer(1.0, self.tick)

    def on_grid(self, msg: OccupancyGrid) -> None:
        self.grid = msg

    def on_enable(self, msg: Bool) -> None:
        self.active = msg.data

    def tick(self) -> None:
        if not self.active or self.goal_in_flight or self.grid is None:
            return
        robot = self.lookup_robot_xy()
        if robot is None:
            return
        target = self.nearest_unmopped_cell(robot[0], robot[1])
        if target is None:
            self.get_logger().info("No missed cells remain.")
            self.active = False
            return
        if self.last_goal and math.hypot(target[0] - self.last_goal[0], target[1] - self.last_goal[1]) < self.min_goal_spacing:
            return
        self.send_goal(target)

    def lookup_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(f"TF unavailable: {exc}", throttle_duration_sec=3.0)
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def nearest_unmopped_cell(self, robot_x: float, robot_y: float) -> Optional[Tuple[float, float]]:
        assert self.grid is not None
        best = None
        best_dist = float("inf")
        width = self.grid.info.width
        for idx, value in enumerate(self.grid.data):
            if value != 0:
                continue
            row = idx // width
            col = idx % width
            x = self.grid.info.origin.position.x + (col + 0.5) * self.grid.info.resolution
            y = self.grid.info.origin.position.y + (row + 0.5) * self.grid.info.resolution
            dist = (x - robot_x) ** 2 + (y - robot_y) ** 2
            if dist < best_dist:
                best = (x, y)
                best_dist = dist
        return best

    def send_goal(self, xy: Tuple[float, float]) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn("Nav2 navigate_to_pose action is not available yet.")
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = xy[0]
        goal_msg.pose.pose.position.y = xy[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.goal_in_flight = True
        self.last_goal = xy
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.goal_in_flight = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_goal_done)

    def on_goal_done(self, _future) -> None:
        self.goal_in_flight = False


def main() -> None:
    rclpy.init()
    node = MissedAreaDispatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
