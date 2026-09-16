"""Reactive lane-sweep coverage driver for the mopping robot.

Replaces Nav2's DWB controller / BT navigator for actual driving -- Nav2
proved too unpredictable (jerky, went in circles) on this robot, and the
motor byte values it was driving through were never validated at anything
but full speed anyway. This node drives the hoverboard directly by
publishing the exact proven byte values from robotmaganglidar1.py straight
to /motor_rpm, using its own live /scan sectors (front / front-left /
front-right) for obstacle reaction:

    MOTOR_FORWARD       97   straight
    MOTOR_FORWARD_SLOW  107  gentle turn (slowed side)
    MOTOR_FORWARD_VERY_SLOW 117  sharp turn (slowed side)
    MOTOR_REVERSE       157  reverse
    SPIN_LEFT/SPIN_RIGHT     in-place U-turn, one wheel FORWARD one REVERSE
    OBSTACLE_STOP_SECONDS 0.40, TURN_BACK_SECONDS 1.40  -- same timing

Coverage strategy: sweep the room in straight lanes parallel to the longer
free-space axis. The first lane's near edge sits wall_margin from the wall;
each following lane shifts over by exactly one robot_width (edge-to-edge,
no gap/overlap). AMCL (/amcl_pose) gives the lane cross-track/heading
correction target; the saved static map is also checked a short distance
ahead of the robot as a second, independent boundary check -- since a
blind spot (or something the live scan just doesn't catch) shouldn't mean
driving through a wall the map already knows is there.

/scan (raw, NOT /scan_filtered) is used here on purpose: /scan_filtered
blanks anything closer than exclude_radius_m to hide the laptop that rides
behind the robot, but that blind spot is squarely in the robot's REAR --
this node only ever looks at front/front-left/front-right sectors, so the
raw scan is safe and avoids losing exactly the close-range front readings
the 25cm stop distance depends on. /scan_filtered stays the right choice
for AMCL, which does look all the way around.

A CoverageTracker (same footprint-marking logic as before) still records
which cells have been passed over, so mop-up of missed pockets still works
the same way.
"""

import math
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int16MultiArray, String

# ---------------------------------------------------------------------------
# Proven motor byte values, copied as-is from robotmaganglidar1.py -- do not
# guess new ones, these are the only values that have actually been tested
# on this robot.
# ---------------------------------------------------------------------------
MOTOR_NEUTRAL = 127
MOTOR_STEP = 30
MOTOR_FORWARD = MOTOR_NEUTRAL - MOTOR_STEP            # 97
MOTOR_REVERSE = MOTOR_NEUTRAL + MOTOR_STEP            # 157
MOTOR_FORWARD_SLOW = MOTOR_NEUTRAL - 20               # 107, gentle-turn slowed side
MOTOR_FORWARD_VERY_SLOW = MOTOR_NEUTRAL - 10          # 117, sharp-turn slowed side
SPIN_LEFT = (MOTOR_REVERSE, MOTOR_FORWARD)            # (kiri, kanan) -- spin left/CCW
SPIN_RIGHT = (MOTOR_FORWARD, MOTOR_REVERSE)           # (kiri, kanan) -- spin right/CW
OBSTACLE_STOP_SECONDS = 0.40
TURN_BACK_SECONDS = 1.40


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class CoverageState(Enum):
    WAITING_MAP = auto()
    WAITING_LOCALIZATION = auto()
    DRIVING = auto()
    STOPPING = auto()
    REVERSING = auto()
    TURNING = auto()
    FINISHED = auto()


class MapProcessor:
    """Turns a raw OccupancyGrid into a safe-free mask usable for lane math and boundary checks."""

    def __init__(self, grid: OccupancyGrid, wall_margin: float, robot_half_width: float):
        self.info = grid.info
        self.resolution = grid.info.resolution
        self.width = grid.info.width
        self.height = grid.info.height
        self.origin_x = grid.info.origin.position.x
        self.origin_y = grid.info.origin.position.y

        raw = np.array(grid.data, dtype=np.int16).reshape(self.height, self.width)
        free = raw == 0
        occupied = raw >= 65

        margin_m = wall_margin + robot_half_width
        margin_cells = int(math.ceil(margin_m / self.resolution)) if self.resolution > 0 else 0
        blocked = occupied | (raw < 0)
        blocked = self._dilate(blocked, margin_cells)

        self.safe_free = free & ~blocked

        rows, cols = np.where(self.safe_free)
        self.has_free_space = rows.size > 0
        if self.has_free_space:
            self.row_min, self.row_max = int(rows.min()), int(rows.max())
            self.col_min, self.col_max = int(cols.min()), int(cols.max())

    @staticmethod
    def _dilate(mask: np.ndarray, cells: int) -> np.ndarray:
        if cells <= 0:
            return mask
        out = mask.copy()
        for _ in range(cells):
            grown = out.copy()
            grown[1:, :] |= out[:-1, :]
            grown[:-1, :] |= out[1:, :]
            grown[:, 1:] |= out[:, :-1]
            grown[:, :-1] |= out[:, 1:]
            out = grown
        return out

    def cell_to_world(self, row: int, col: int) -> Tuple[float, float]:
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def is_safe(self, x: float, y: float) -> bool:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return False
        row, col = cell
        return bool(self.safe_free[row, col])


class CoverageTracker:
    """Marks the mop footprint over the safe-free mask as the robot moves."""

    def __init__(self, processor: MapProcessor, mop_width: float):
        self.processor = processor
        self.radius_cells = max(1, int(math.ceil((mop_width * 0.5) / processor.resolution)))
        self.covered = np.zeros_like(processor.safe_free, dtype=bool)
        self.total_free_cells = int(np.count_nonzero(processor.safe_free))

    def mark(self, x: float, y: float) -> None:
        cell = self.processor.world_to_cell(x, y)
        if cell is None:
            return
        row0, col0 = cell
        r = self.radius_cells
        row_lo, row_hi = max(0, row0 - r), min(self.processor.height, row0 + r + 1)
        col_lo, col_hi = max(0, col0 - r), min(self.processor.width, col0 + r + 1)

        rows = np.arange(row_lo, row_hi)[:, None]
        cols = np.arange(col_lo, col_hi)[None, :]
        disk = (rows - row0) ** 2 + (cols - col0) ** 2 <= r * r

        region = self.covered[row_lo:row_hi, col_lo:col_hi]
        region[disk] = True

    def percent(self) -> float:
        if self.total_free_cells == 0:
            return 0.0
        done = int(np.count_nonzero(self.covered & self.processor.safe_free))
        return 100.0 * done / self.total_free_cells

    def to_occupancy_grid(self, stamp) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = stamp
        grid.info = self.processor.info
        data = np.full(self.processor.safe_free.shape, -1, dtype=np.int8)
        data[self.processor.safe_free] = 0
        data[self.processor.safe_free & self.covered] = 100
        grid.data = data.flatten().tolist()
        return grid


@dataclass
class Lane:
    axis: str        # 'x' or 'y' -- the world axis the robot drives along
    sign: int         # +1 or -1 -- direction of travel along that axis this lane
    offset: float     # world coordinate on the OTHER axis this lane holds


def sector_min_range(scan: LaserScan, center_deg: float, half_width_deg: float) -> float:
    """Minimum valid range within a LiDAR-local angular window (degrees)."""
    center = math.radians(center_deg)
    half = math.radians(half_width_deg)
    best = float("inf")
    angle = scan.angle_min
    for r in scan.ranges:
        diff = abs(normalize_angle(angle - center))
        if diff <= half and scan.range_min <= r <= scan.range_max:
            if r < best:
                best = r
        angle += scan.angle_increment
    return best


def sector_wall_fraction(scan: LaserScan, center_deg: float, half_width_deg: float, within_m: float) -> float:
    """Fraction of valid rays in the sector reading closer than within_m.

    A real wall fills the whole sector at similar close range (fraction near
    1.0); a single thin object (chair leg, cable, sensor noise) only trips
    one or two rays (fraction near 0) -- used to tell an actual wall apart
    from a lone spike before committing to the stop/reverse/turn sequence.
    """
    center = math.radians(center_deg)
    half = math.radians(half_width_deg)
    total = 0
    close = 0
    angle = scan.angle_min
    for r in scan.ranges:
        diff = abs(normalize_angle(angle - center))
        if diff <= half and scan.range_min <= r <= scan.range_max:
            total += 1
            if r < within_m:
                close += 1
        angle += scan.angle_increment
    return (close / total) if total > 0 else 0.0


class CoveragePlannerNode(Node):

    def __init__(self):
        super().__init__("coverage_planner_node")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("global_frame", "map")

        self.declare_parameter("mop_width", 0.40)
        self.declare_parameter("wall_margin", 0.15)
        self.declare_parameter("robot_half_width", 0.25)
        self.declare_parameter("robot_width", 0.50)

        # Reactive driving thresholds -- matches the behavior spec directly:
        # far ahead clear -> straight (+ lane correction); something within
        # 1m to the front-left/front-right -> veer away from it; a WALL
        # (see wall_confirm_fraction) within 30cm dead ahead -> stop, back
        # up, U-turn.
        self.declare_parameter("front_stop_m", 0.30)
        self.declare_parameter("wall_confirm_fraction", 0.6)  # fraction of front-sector rays that must be close to call it a wall, not a spike
        self.declare_parameter("veer_warn_m", 1.0)
        self.declare_parameter("veer_sharp_m", 0.5)
        self.declare_parameter("front_sector_deg", 20.0)
        self.declare_parameter("front_diag_center_deg", 135.0)
        self.declare_parameter("front_diag_sector_deg", 25.0)
        self.declare_parameter("reverse_seconds", 0.6)
        self.declare_parameter("lookahead_map_check_m", 0.0)  # <=0 disables; see _lookahead_blocked docstring
        self.declare_parameter("lane_correct_deg", 8.0)
        self.declare_parameter("map_side_check_m", 1.5)  # how far to sample the saved map left/right when picking turn direction

        self.declare_parameter("completion_percent", 95.0)
        self.declare_parameter("require_localized", True)
        self.declare_parameter("max_pose_covariance", 0.5)
        self.declare_parameter("status_publish_period", 1.0)
        self.declare_parameter("control_period", 0.1)

        self.mop_width = float(self.get_parameter("mop_width").value)
        self.wall_margin = float(self.get_parameter("wall_margin").value)
        self.robot_half_width = float(self.get_parameter("robot_half_width").value)
        self.robot_width = float(self.get_parameter("robot_width").value)

        self.front_stop_m = float(self.get_parameter("front_stop_m").value)
        self.wall_confirm_fraction = float(self.get_parameter("wall_confirm_fraction").value)
        self.veer_warn_m = float(self.get_parameter("veer_warn_m").value)
        self.veer_sharp_m = float(self.get_parameter("veer_sharp_m").value)
        self.front_sector_deg = float(self.get_parameter("front_sector_deg").value)
        self.front_diag_center_deg = float(self.get_parameter("front_diag_center_deg").value)
        self.front_diag_sector_deg = float(self.get_parameter("front_diag_sector_deg").value)
        self.reverse_seconds = float(self.get_parameter("reverse_seconds").value)
        self.lookahead_map_check_m = float(self.get_parameter("lookahead_map_check_m").value)
        self.lane_correct_deg = float(self.get_parameter("lane_correct_deg").value)
        self.map_side_check_m = float(self.get_parameter("map_side_check_m").value)

        self.completion_percent = float(self.get_parameter("completion_percent").value)
        self.require_localized = bool(self.get_parameter("require_localized").value)
        self.max_pose_covariance = float(self.get_parameter("max_pose_covariance").value)

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self.on_map, map_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter("amcl_pose_topic").value, self.on_amcl_pose, 10
        )
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.on_scan, 10)

        self.status_pub = self.create_publisher(String, "/coverage/status", 10)
        self.percent_pub = self.create_publisher(Float32, "/coverage/percent", 10)
        self.complete_pub = self.create_publisher(Bool, "/coverage/complete", 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/coverage/grid", map_qos)
        self.motor_pub = self.create_publisher(Int16MultiArray, "/motor_rpm", 10)

        self.state = CoverageState.WAITING_MAP
        self.processor: Optional[MapProcessor] = None
        self.tracker: Optional[CoverageTracker] = None
        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.is_localized = not self.require_localized
        self.latest_scan: Optional[LaserScan] = None

        self.lane: Optional[Lane] = None
        self.lane_index = 0
        self._state_entered_at = 0.0
        self._mission_start_time = 0.0
        self._turn_direction = "right"  # chosen fresh at each stop by _pick_turn_direction

        self.create_timer(float(self.get_parameter("control_period").value), self.control_loop)
        self.create_timer(float(self.get_parameter("status_publish_period").value), self.publish_status)

        self.get_logger().info("coverage_planner_node (reactive) ready, waiting for /map and localization")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def on_map(self, msg: OccupancyGrid) -> None:
        if self.processor is not None:
            return
        self.processor = MapProcessor(msg, self.wall_margin, self.robot_half_width)
        self.tracker = CoverageTracker(self.processor, self.mop_width)
        self.get_logger().info(
            f"Map received: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m, "
            f"{self.tracker.total_free_cells} safe-free cells"
        )
        if self.state == CoverageState.WAITING_MAP:
            self.state = CoverageState.WAITING_LOCALIZATION

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)

        if self.tracker is not None:
            self.tracker.mark(p.x, p.y)

        if self.require_localized:
            cov = msg.pose.covariance
            self.is_localized = cov[0] < self.max_pose_covariance and cov[7] < self.max_pose_covariance

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    # ------------------------------------------------------------------
    # Lane setup
    # ------------------------------------------------------------------

    def _start_first_lane(self) -> None:
        # safe_free is already eroded by (wall_margin + robot_half_width) in
        # MapProcessor.__init__, so row_min/col_max etc. are already the
        # first/last safe cells -- their world coordinate already sits
        # wall_margin+robot_half_width from the true wall. Adding that
        # margin again here would double-count it.
        p = self.processor
        row_span = p.row_max - p.row_min
        col_span = p.col_max - p.col_min

        if col_span >= row_span:
            # Sweep along x, lanes stacked along y starting near the bottom wall.
            axis = "x"
            y0 = p.cell_to_world(p.row_min, p.col_min)[1]
            self.lane = Lane(axis="x", sign=+1, offset=y0)
        else:
            axis = "y"
            x0 = p.cell_to_world(p.row_min, p.col_min)[0]
            self.lane = Lane(axis="y", sign=+1, offset=x0)

        self.lane_index = 0
        self.get_logger().info(f"First lane: axis={axis}, offset={self.lane.offset:.2f} m")

    def _advance_lane(self) -> None:
        self.lane_index += 1
        self.lane.offset += self.robot_width
        self.lane.sign *= -1

    def _lane_target_yaw(self) -> float:
        forward = 0.0 if self.lane.sign > 0 else math.pi
        return forward if self.lane.axis == "x" else normalize_angle(forward + math.pi / 2.0)

    def _past_far_boundary(self) -> bool:
        # Same reasoning as _start_first_lane: row_max/col_max are already
        # the last safe cells, already margin-eroded from the far wall.
        p = self.processor
        if self.lane.axis == "x":
            limit = p.cell_to_world(p.row_max, p.col_max)[1]
        else:
            limit = p.cell_to_world(p.row_max, p.col_max)[0]
        return self.lane.offset > limit

    # ------------------------------------------------------------------
    # Main state machine
    # ------------------------------------------------------------------

    def control_loop(self) -> None:
        if self.state == CoverageState.WAITING_MAP:
            return

        if self.state == CoverageState.WAITING_LOCALIZATION:
            if self.current_pose is None or not self.is_localized:
                self.get_logger().warn(
                    "Waiting for a localized /amcl_pose (set 2D Pose Estimate in RViz if needed)",
                    throttle_duration_sec=5.0,
                )
                return
            self._start_first_lane()
            self._mission_start_time = time.monotonic()
            self._enter(CoverageState.DRIVING)
            return

        if self.current_pose is None or self.latest_scan is None:
            self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
            return

        if self.state == CoverageState.DRIVING:
            self._drive()
        elif self.state == CoverageState.STOPPING:
            self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
            if time.monotonic() - self._state_entered_at >= OBSTACLE_STOP_SECONDS:
                self._enter(CoverageState.REVERSING)
        elif self.state == CoverageState.REVERSING:
            self._publish_motor(MOTOR_REVERSE, MOTOR_REVERSE)
            if time.monotonic() - self._state_entered_at >= self.reverse_seconds:
                self._enter(CoverageState.TURNING)
        elif self.state == CoverageState.TURNING:
            kiri, kanan = SPIN_RIGHT if self._turn_direction == "right" else SPIN_LEFT
            self._publish_motor(kiri, kanan)
            if time.monotonic() - self._state_entered_at >= TURN_BACK_SECONDS:
                self._advance_lane()
                past_boundary = self._past_far_boundary()
                done = self.tracker.percent() >= self.completion_percent
                if past_boundary or done:
                    reason = "past far boundary" if past_boundary else "completion percent reached"
                    self.get_logger().info(f"Finishing ({reason}) at lane {self.lane_index}")
                    self.finish_mission()
                else:
                    self.get_logger().info(f"Starting lane {self.lane_index}, offset={self.lane.offset:.2f}m")
                    self._enter(CoverageState.DRIVING)

    def _enter(self, state: CoverageState) -> None:
        self.state = state
        self._state_entered_at = time.monotonic()

    def _drive(self) -> None:
        front = sector_min_range(self.latest_scan, 180.0, self.front_sector_deg)
        front_right = sector_min_range(self.latest_scan, self.front_diag_center_deg, self.front_diag_sector_deg)
        front_left = sector_min_range(self.latest_scan, -self.front_diag_center_deg, self.front_diag_sector_deg)

        map_blocked_ahead = self._lookahead_blocked()

        wall_ahead = False
        if front < self.front_stop_m:
            fraction = sector_wall_fraction(self.latest_scan, 180.0, self.front_sector_deg, self.front_stop_m)
            wall_ahead = fraction >= self.wall_confirm_fraction
            if not wall_ahead:
                self.get_logger().info(
                    f"Ignoring close spike at {front:.2f}m (wall_fraction={fraction:.2f}, "
                    f"< {self.wall_confirm_fraction}) -- looks like a small object, not a wall",
                    throttle_duration_sec=1.0,
                )

        if wall_ahead or map_blocked_ahead:
            reason = "wall ahead" if wall_ahead else "map lookahead"
            self.get_logger().info(
                f"Stopping ({reason}): front={front:.2f}m fr={front_right:.2f}m fl={front_left:.2f}m"
            )
            self._pick_turn_direction_from_map()
            self._enter(CoverageState.STOPPING)
            return

        if front_right < self.veer_sharp_m:
            self._publish_motor(MOTOR_FORWARD, MOTOR_FORWARD_VERY_SLOW)  # sharp left
            return
        if front_right < self.veer_warn_m:
            self._publish_motor(MOTOR_FORWARD, MOTOR_FORWARD_SLOW)  # gentle left
            return
        if front_left < self.veer_sharp_m:
            self._publish_motor(MOTOR_FORWARD_VERY_SLOW, MOTOR_FORWARD)  # sharp right
            return
        if front_left < self.veer_warn_m:
            self._publish_motor(MOTOR_FORWARD_SLOW, MOTOR_FORWARD)  # gentle right
            return

        self._drive_straight_with_lane_correction()

    def _drive_straight_with_lane_correction(self) -> None:
        x, y, yaw = self.current_pose
        target_yaw = self._lane_target_yaw()
        cross_track = (y - self.lane.offset) if self.lane.axis == "x" else (x - self.lane.offset)
        # Bias heading slightly against cross-track error, on top of pure
        # heading error -- keeps the robot converging back onto the lane
        # line rather than just holding whatever heading it started with.
        heading_error = normalize_angle(yaw - target_yaw) + normalize_angle(0.3 * cross_track)

        deg = math.degrees(heading_error)
        if abs(deg) < self.lane_correct_deg * 0.3:
            self._publish_motor(MOTOR_FORWARD, MOTOR_FORWARD)
        elif deg > 0:
            # heading rotated left of target -> nudge right to come back
            if deg > self.lane_correct_deg:
                self._publish_motor(MOTOR_FORWARD_VERY_SLOW, MOTOR_FORWARD)
            else:
                self._publish_motor(MOTOR_FORWARD_SLOW, MOTOR_FORWARD)
        else:
            if -deg > self.lane_correct_deg:
                self._publish_motor(MOTOR_FORWARD, MOTOR_FORWARD_VERY_SLOW)
            else:
                self._publish_motor(MOTOR_FORWARD, MOTOR_FORWARD_SLOW)

    def _map_side_free_fraction(self, x: float, y: float, yaw: float, side_sign: int) -> float:
        """Fraction of safe_free cells sampled in a patch to one side of (x,y).

        side_sign +1 = left (yaw+90deg), -1 = right (yaw-90deg). Sampling an
        area rather than a single point is what makes this ignore lone
        "bintik" (speckle) noise in the map -- one stray occupied pixel
        barely moves the fraction, a real wall/furniture block does.
        """
        side_yaw = yaw + side_sign * (math.pi / 2.0)
        total = 0
        free = 0
        for lateral in np.arange(0.2, self.map_side_check_m + 1e-6, 0.15):
            for forward in (-0.3, 0.0, 0.3, 0.6):
                px = x + forward * math.cos(yaw) + lateral * math.cos(side_yaw)
                py = y + forward * math.sin(yaw) + lateral * math.sin(side_yaw)
                total += 1
                if self.processor.is_safe(px, py):
                    free += 1
        return (free / total) if total > 0 else 0.0

    def _pick_turn_direction_from_map(self) -> None:
        # The behavior spec calls for turning toward whichever side the
        # SAVED MAP shows as more open -- not live LiDAR -- so the choice
        # stays correct even where the live scan's side sectors are short
        # range or noisy right at the moment the robot stops at a wall.
        x, y, yaw = self.current_pose
        left_frac = self._map_side_free_fraction(x, y, yaw, +1)
        right_frac = self._map_side_free_fraction(x, y, yaw, -1)
        self._turn_direction = "left" if left_frac >= right_frac else "right"
        self.get_logger().info(
            f"Turn direction pick (map): left_free={left_frac:.2f} right_free={right_frac:.2f} -> {self._turn_direction}"
        )

    def _lookahead_blocked(self) -> bool:
        """Fallback check against the saved static map for what the live scan might miss.

        Disabled by default (lookahead_map_check_m <= 0): this depends on
        AMCL's reported pose being accurate, and a localization drift makes
        it check the wrong point on the map -- looked blocked constantly
        even with 2.9m of clear space on live LiDAR, freezing the robot in
        place. Re-enable once AMCL's accuracy at this location is confirmed.
        """
        if self.lookahead_map_check_m <= 0.0 or self.current_pose is None:
            return False
        x, y, yaw = self.current_pose
        lx = x + self.lookahead_map_check_m * math.cos(yaw)
        ly = y + self.lookahead_map_check_m * math.sin(yaw)
        blocked = not self.processor.is_safe(lx, ly)
        if blocked:
            self.get_logger().info(
                f"Map lookahead blocked at ({lx:.2f}, {ly:.2f}) -- robot pose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)"
            )
        return blocked

    def _publish_motor(self, kiri: int, kanan: int) -> None:
        msg = Int16MultiArray()
        msg.data = [int(kiri), int(kanan)]
        self.motor_pub.publish(msg)

    def finish_mission(self) -> None:
        self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
        self.state = CoverageState.FINISHED
        elapsed = time.monotonic() - self._mission_start_time
        percent = self.tracker.percent() if self.tracker else 0.0
        self.get_logger().info(f"Coverage mission finished: {percent:.1f}% mopped in {elapsed:.0f}s")

    # ------------------------------------------------------------------
    # Status publishing
    # ------------------------------------------------------------------

    def publish_status(self) -> None:
        status_msg = String()
        status_msg.data = self.state.name
        self.status_pub.publish(status_msg)

        if self.tracker is not None:
            percent_msg = Float32()
            percent_msg.data = self.tracker.percent()
            self.percent_pub.publish(percent_msg)
            self.grid_pub.publish(self.tracker.to_occupancy_grid(self.get_clock().now().to_msg()))

        complete_msg = Bool()
        complete_msg.data = self.state == CoverageState.FINISHED
        self.complete_pub.publish(complete_msg)


def main() -> None:
    rclpy.init()
    node = CoveragePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
