#!/usr/bin/env python3
"""ROS 2 Action Client: boustrophedon coverage path via Nav2 FollowWaypoints.

Generates a lawnmower ("naik-turun") coverage path over a rectangular area
and sends the whole list of waypoints in one shot to Nav2's
/follow_waypoints action server (package nav2_waypoint_follower). Nav2 --
global planner + local costmap + controller_server -- is what actually
drives between waypoints and swerves around dynamic obstacles before
returning to the lane; this node's only job is deciding WHERE the
waypoints are.

Path shape: straight lanes parallel to the X axis, spaced swath_width
apart along Y, direction alternating each lane (classic lawnmower / N or
U-turn at each lane end, depending on how Nav2's planner rounds the
corner between two waypoints).
"""

import math
from typing import List

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class CoverageWaypointNode(Node):
    """Generates a lawnmower coverage path and drives it via Nav2's FollowWaypoints action."""

    def __init__(self) -> None:
        super().__init__("coverage_waypoint_node")

        # --------------------------------------------------------------
        # Configurable parameters -- override via a YAML params file or
        # `--ros-args -p name:=value` at launch, no code edit needed.
        # --------------------------------------------------------------
        self.declare_parameter("swath_width", 0.4)  # jarak antar lajur (lebar kain pel), meter
        self.declare_parameter(
            "room_bounds", [0.0, 0.0, 5.0, 3.0]
        )  # [x_min, y_min, x_max, y_max] meter -- GANTI sesuai batas ruangan di map.yaml kamu
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("auto_start", True)  # kirim waypoint otomatis begitu node siap

        self.swath_width: float = float(self.get_parameter("swath_width").value)
        self.room_bounds: List[float] = [float(v) for v in self.get_parameter("room_bounds").value]
        self.global_frame: str = self.get_parameter("global_frame").value

        self._client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._waypoints: List[PoseStamped] = []
        self._retry_attempts = 0
        self._retry_timer = None

        self.get_logger().info(
            f"coverage_waypoint_node ready: swath_width={self.swath_width} m, "
            f"room_bounds={self.room_bounds} ({self.global_frame} frame)"
        )

        if bool(self.get_parameter("auto_start").value):
            self.send_waypoints()

    # ------------------------------------------------------------------
    # Path generation
    # ------------------------------------------------------------------

    def generate_lawnmower_path(self) -> List[PoseStamped]:
        """Sweep room_bounds in straight lanes spaced swath_width apart along Y."""
        x_min, y_min, x_max, y_max = self.room_bounds
        if x_max <= x_min or y_max <= y_min or self.swath_width <= 0.0:
            self.get_logger().error("Invalid room_bounds/swath_width -- check parameters")
            return []

        waypoints: List[PoseStamped] = []
        y = y_min + self.swath_width / 2.0
        forward = True
        lane_count = 0

        while y <= y_max - self.swath_width / 2.0 + 1e-6:
            x_start, x_end = (x_min, x_max) if forward else (x_max, x_min)
            yaw = 0.0 if forward else math.pi

            # Two points per lane: entering the straight run and reaching its
            # far end. Nav2's own planner fills in the U/N-shaped turn to the
            # next lane's start point.
            waypoints.append(self._make_pose(x_start, y, yaw))
            waypoints.append(self._make_pose(x_end, y, yaw))

            y += self.swath_width
            forward = not forward
            lane_count += 1

        self.get_logger().info(f"Generated {lane_count} lanes / {len(waypoints)} waypoints")
        return waypoints

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = yaw_to_quaternion(yaw)
        return pose

    # ------------------------------------------------------------------
    # Nav2 action client
    # ------------------------------------------------------------------

    def send_waypoints(self) -> None:
        self._waypoints = self.generate_lawnmower_path()
        if not self._waypoints:
            return

        self.get_logger().info("Waiting for /follow_waypoints action server...")
        self._client.wait_for_server()

        # wait_for_server() only confirms the action NAME exists -- it says
        # nothing about whether the underlying Nav2 lifecycle node has
        # actually reached the "active" state yet. In practice
        # lifecycle_manager_navigation can take tens of seconds to activate
        # everything after the action server first appears; sending once
        # and giving up on the first "Action server is inactive. Rejecting
        # the goal." (as this used to do) means the robot never moves.
        # Retrying on a timer until Nav2 is truly ready fixes that.
        self._retry_attempts = 0
        self._retry_timer = self.create_timer(3.0, self._try_send_goal)
        self._try_send_goal()

    def _try_send_goal(self) -> None:
        self._retry_attempts += 1
        goal = FollowWaypoints.Goal()
        goal.poses = self._waypoints

        self.get_logger().info(
            f"Sending {len(self._waypoints)} waypoints to Nav2 (attempt {self._retry_attempts})"
        )
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg) -> None:
        idx = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f"Heading to waypoint {idx}", throttle_duration_sec=2.0)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                "Waypoint goal rejected (Nav2 probably still activating) -- retrying in 3s",
                throttle_duration_sec=2.0,
            )
            if self._retry_attempts >= 20:
                self.get_logger().error(
                    "Still rejected after 20 attempts (~1 min) -- Nav2 likely failed to "
                    "activate at all, check controller_server/bt_navigator/waypoint_follower logs"
                )
            return
        self._retry_timer.cancel()
        self.get_logger().info("Waypoint goal accepted, coverage run starting")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        wrapped = future.result()
        status = wrapped.status
        result = wrapped.result

        if status == GoalStatus.STATUS_SUCCEEDED and not result.missed_waypoints:
            self.get_logger().info("Coverage path finished: all waypoints reached")
        else:
            missed_idx = [m.index for m in result.missed_waypoints]
            self.get_logger().warn(
                f"Coverage path finished with status {status}; missed waypoints: {missed_idx}"
            )


def main() -> None:
    rclpy.init()
    node = CoverageWaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
