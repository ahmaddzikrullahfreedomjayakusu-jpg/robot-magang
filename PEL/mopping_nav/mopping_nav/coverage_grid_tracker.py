import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from .common import grid_index


class CoverageGridTracker(Node):
    """Tracks cells already crossed by the mop footprint.

    Cell value convention on /mopping/coverage_grid:
      0   = belum dipel
      100 = sudah dipel
      -1  = unknown/outside known static map
    """

    def __init__(self) -> None:
        super().__init__("coverage_grid_tracker")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("coverage_resolution", 0.10)
        self.declare_parameter("mop_width", 0.42)
        self.declare_parameter("mop_length", 0.28)
        self.declare_parameter("publish_period", 1.0)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.resolution = float(self.get_parameter("coverage_resolution").value)
        self.mop_width = float(self.get_parameter("mop_width").value)
        self.mop_length = float(self.get_parameter("mop_length").value)

        self.static_map: Optional[OccupancyGrid] = None
        self.grid = OccupancyGrid()
        self.coverage_data: List[int] = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.on_map,
            10,
        )
        self.grid_pub = self.create_publisher(OccupancyGrid, "/mopping/coverage_grid", 10)
        self.percent_pub = self.create_publisher(Float32, "/mopping/coverage_percent", 10)
        self.target_pub = self.create_publisher(PoseStamped, "/mopping/next_missed_cell", 10)
        self.create_timer(float(self.get_parameter("publish_period").value), self.update)

    def on_map(self, msg: OccupancyGrid) -> None:
        self.static_map = msg
        width_m = msg.info.width * msg.info.resolution
        height_m = msg.info.height * msg.info.resolution
        width = int(math.ceil(width_m / self.resolution))
        height = int(math.ceil(height_m / self.resolution))

        self.grid.header.frame_id = self.map_frame
        self.grid.info.resolution = self.resolution
        self.grid.info.width = width
        self.grid.info.height = height
        self.grid.info.origin = msg.info.origin
        self.coverage_data = [0] * (width * height)

        # Mark cells outside free static-map space as unknown so coverage percent only counts reachable floor.
        for row in range(height):
            for col in range(width):
                x = self.grid.info.origin.position.x + (col + 0.5) * self.resolution
                y = self.grid.info.origin.position.y + (row + 0.5) * self.resolution
                static_idx = grid_index(
                    x,
                    y,
                    msg.info.origin.position.x,
                    msg.info.origin.position.y,
                    msg.info.resolution,
                    msg.info.width,
                    msg.info.height,
                )
                if static_idx is None or msg.data[static_idx[2]] != 0:
                    self.coverage_data[row * width + col] = -1

        self.get_logger().info(
            f"Coverage grid ready: {width}x{height} cells at {self.resolution:.2f} m"
        )

    def update(self) -> None:
        if not self.coverage_data:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(f"TF unavailable: {exc}", throttle_duration_sec=3.0)
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        self.mark_mop_footprint(x, y)
        self.publish_grid_and_progress()
        self.publish_next_missed_cell(x, y)

    def mark_mop_footprint(self, robot_x: float, robot_y: float) -> None:
        radius = 0.5 * math.hypot(self.mop_width, self.mop_length)
        cells = int(math.ceil(radius / self.resolution))
        center = grid_index(
            robot_x,
            robot_y,
            self.grid.info.origin.position.x,
            self.grid.info.origin.position.y,
            self.resolution,
            self.grid.info.width,
            self.grid.info.height,
        )
        if center is None:
            return
        row0, col0, _ = center
        for row in range(row0 - cells, row0 + cells + 1):
            for col in range(col0 - cells, col0 + cells + 1):
                if 0 <= row < self.grid.info.height and 0 <= col < self.grid.info.width:
                    wx = self.grid.info.origin.position.x + (col + 0.5) * self.resolution
                    wy = self.grid.info.origin.position.y + (row + 0.5) * self.resolution
                    if math.hypot(wx - robot_x, wy - robot_y) <= radius:
                        idx = row * self.grid.info.width + col
                        if self.coverage_data[idx] == 0:
                            self.coverage_data[idx] = 100

    def publish_grid_and_progress(self) -> None:
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.grid.data = self.coverage_data
        self.grid_pub.publish(self.grid)

        cleanable = sum(1 for value in self.coverage_data if value >= 0)
        done = sum(1 for value in self.coverage_data if value == 100)
        percent = Float32()
        percent.data = 100.0 * done / cleanable if cleanable else 0.0
        self.percent_pub.publish(percent)

    def publish_next_missed_cell(self, robot_x: float, robot_y: float) -> None:
        target = self.find_nearest_unmopped_cell(robot_x, robot_y)
        if target is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def find_nearest_unmopped_cell(self, robot_x: float, robot_y: float) -> Optional[Tuple[float, float]]:
        best = None
        best_dist = float("inf")
        for idx, value in enumerate(self.coverage_data):
            if value != 0:
                continue
            row = idx // self.grid.info.width
            col = idx % self.grid.info.width
            x = self.grid.info.origin.position.x + (col + 0.5) * self.resolution
            y = self.grid.info.origin.position.y + (row + 0.5) * self.resolution
            dist = (x - robot_x) ** 2 + (y - robot_y) ** 2
            if dist < best_dist:
                best = (x, y)
                best_dist = dist
        return best


def main() -> None:
    rclpy.init()
    node = CoverageGridTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
