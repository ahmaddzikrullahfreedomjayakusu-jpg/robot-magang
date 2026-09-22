"""Reactive straight-then-U-turn coverage driver for the mopping robot.

Replaces Nav2's DWB controller / BT navigator for actual driving -- Nav2
proved too unpredictable (jerky, went in circles) on this robot, and the
motor byte values it was driving through were never validated at anything
but full speed anyway. This node drives the hoverboard directly by
publishing the exact proven byte values from robotmaganglidar1.py straight
to /motor_rpm.

Deliberately minimal: drive straight, and only react to what's directly
ahead (via /scan's front sector). Left/right side-veer avoidance and
active lane-heading correction were both tried and both caused more
instability (zigzag, backwards corrections, flapping) than they were
worth -- ripped out so the core loop is trustworthy first:

    MOTOR_FORWARD       97   straight (the only forward speed used now)
    MOTOR_REVERSE       157  reverse
    SPIN_LEFT/SPIN_RIGHT     in-place U-turn, one wheel FORWARD one REVERSE
    OBSTACLE_STOP_SECONDS 0.35, TURN_BACK_SECONDS 3.25  -- same timing

Coverage strategy: sweep the room in straight lanes parallel to the longer
free-space axis. The first lane's near edge sits wall_margin from the wall;
each following lane shifts over by exactly one robot_width (edge-to-edge,
no gap/overlap). Pose + map come from one of two sources, picked by the
`localization_source` param:
  - "amcl" (default): AMCL's /amcl_pose against the pre-saved static map
    (map_server) -- needs a "2D Pose Estimate" click in RViz to start.
  - "tf": no saved map, no AMCL -- slam_toolbox builds /map live as the
    robot drives (and never saves it to disk), pose is read straight off
    the map->base_footprint TF it publishes. Everything downstream (lane
    math, coverage tracking, turn-direction picks) is unchanged either
    way; only where /map and current_pose come from differs.
The saved/live map (whichever is in play) is also checked a short distance
ahead of the robot as a second, independent boundary check -- since a
blind spot (or something the live scan just doesn't catch) shouldn't mean
driving through a wall the map already knows is there.

/scan_filtered (not raw /scan) is used here: besides the laptop riding
behind the robot, the LiDAR also sees the robot's own left/right wheels as
"obstacles" at close range, which could otherwise trip the front-wall
check for no real reason -- scan_blind_spot_filter's radius exclusion
(exclude_radius_m, ~0.32m) blanks both. This is safe now that front_stop_m
is nose-relative
(front_stop_from_lidar_m = nose_length_m + front_stop_m, ~0.65m): the
exclusion radius sits comfortably inside that, so real close obstacles
still trigger a stop well before the nose would touch anything.

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
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int16MultiArray, String
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker

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
# robotmaganglidar1.py actually declares OBSTACLE_STOP_SECONDS=0.40 /
# TURN_BACK_SECONDS=1.40, but those two constants are never referenced
# anywhere else in that file -- dead code. The values its live obstacle
# stop->turn control loop actually runs on are LIDAR_UTURN_STOP_SEC=0.35
# and LIDAR_UTURN_TURN_SEC=3.35 (then adjusted to 3.25 after testing).
OBSTACLE_STOP_SECONDS = 0.35
TURN_BACK_SECONDS = 3.25

# Instead of one continuous ~180deg spin in place, the turn-around is
# split into pivot-90 -> shift-sideways-by-shift_distance_m -> pivot-90
# (same direction both times). Net rotation is still 180deg, but the
# extra straight-line shift in between means the wheel on the turn's side
# ends up retracing that exact same wheel's previous pass -- turn LEFT
# and the LEFT wheel lands back on its own last track, turn RIGHT and the
# RIGHT wheel does (confirmed by the geometry: two same-direction 90deg
# pivots with a robot_width shift between them is exactly the classic
# boustrophedon corner turn -- that alignment is only exact when
# shift_distance_m == robot_width; shift_distance_m defaults smaller
# because at the full robot_width, this room's tight enough that the
# shift kept getting cut short by another wall anyway, which broke the
# alignment regardless -- a slight, reliably-achieved overlap beats an
# unreliable exact edge-to-edge). Odometry (current_pose), not a fixed
# timer, decides when each 90deg pivot / each shift is actually done -- a
# timer alone can't hit a precise angle or distance. The old
# *_BACK_SECONDS timers still act as a safety ceiling in case pose data
# stalls.
TURN_PIVOT_TOLERANCE_RAD = math.radians(3.0)
TURN_PIVOT_TIMEOUT_SECONDS = TURN_BACK_SECONDS / 2.0
SHIFT_TIMEOUT_SECONDS = 6.0


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
    TURNING = auto()      # first 90deg pivot
    SHIFTING = auto()     # drive robot_width forward on the new heading
    ALIGNING = auto()     # second 90deg pivot, same direction as TURNING
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
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("global_frame", "map")
        # "amcl" (default): pose comes from AMCL's /amcl_pose, localized
        # against the pre-saved static map (map_server). "tf": no AMCL, no
        # saved map -- pose is read straight off the map->base_footprint
        # TF that slam_toolbox publishes while it builds the map live, and
        # /map itself is also slam_toolbox's live (never-saved) map, not a
        # file on disk. Everything downstream (lane placement, coverage
        # tracking, turn-direction picks) still reads the exact same /map
        # + current_pose it always did -- only where those two come from
        # changes.
        self.declare_parameter("localization_source", "amcl")

        self.declare_parameter("mop_width", 0.40)
        self.declare_parameter("wall_margin", 0.15)
        self.declare_parameter("robot_half_width", 0.30)  # was 0.25 -- actual robot body is 60cm wide, not 50cm
        self.declare_parameter("robot_width", 0.60)  # was 0.50
        # SHIFTING's actual sideways drive target -- deliberately smaller
        # than robot_width (0.60m). At the full robot_width, "Shift
        # stopped early (wall ahead)" was firing on nearly every corner
        # turn (room's too tight for a full 60cm perpendicular shift in
        # a lot of spots), which broke the precise wheel-on-previous-track
        # alignment anyway since the shift kept getting cut short. 0.50m
        # trades a bit of lane overlap (rows land ~10cm closer together
        # than the robot is wide) for actually completing the shift
        # reliably.
        self.declare_parameter("shift_distance_m", 0.40)  # was 0.50 -- now just the ABSOLUTE ceiling for SHIFTING (see _shift_done()): live coverage-edge detection decides the real stopping point when there's a previous strip to detect
        # Minimum distance SHIFTING must move before it starts checking
        # for the covered-strip edge -- without this, a shift that starts
        # right at (or just barely past) the edge would immediately read
        # "not covered" and stop after a token few cm.
        self.declare_parameter("min_shift_for_edge_check_m", 0.10)

        # Reactive driving thresholds -- matches the behavior spec directly:
        # far ahead clear (per LIVE LiDAR) -> straight (+ lane correction);
        # something within 1m to the front-left/front-right -> veer away
        # from it; a WALL (see wall_confirm_fraction) within front_stop_m
        # of the robot's actual nose tip -> stop, back up, U-turn.
        #
        # The LiDAR sits nose_length_m behind the robot's front tip, so a
        # raw LiDAR reading of front_stop_m + nose_length_m is what actually
        # keeps front_stop_m of clearance in front of the physical robot --
        # using the raw LiDAR distance directly here would let the nose
        # touch the wall before "stopping" ever triggers.
        self.declare_parameter("nose_length_m", 0.30)  # LiDAR to the robot's front tip
        self.declare_parameter("front_stop_m", 0.35)  # desired clearance beyond the nose tip, not from the LiDAR (was 0.15 -> 0.25 -> 0.35, pushed further out each time so stop/reverse/turn kicks in earlier)
        self.declare_parameter("wall_confirm_fraction", 0.6)  # fraction of front-sector rays that must be close to call it a wall, not a spike
        self.declare_parameter("front_sector_deg", 20.0)
        self.declare_parameter("reverse_seconds", 1.0)  # was 0.6 -> 0.9 -> 1.5 -> 1.0
        # Disabled by default: the map is only used to place lanes and mark
        # covered cells now, not to block live driving -- an earlier version
        # of this check used the map to keep the robot off unmapped
        # territory, but it kept false-triggering near the lane start
        # (close to a wall by design) and, combined with the stuck-turn
        # safeguard, made the robot give up early instead of driving.
        # Obstacle avoidance is LIVE-LiDAR-only now (front/diag sectors
        # above). Set > 0 to bring the map check back if you need it.
        self.declare_parameter("lookahead_map_check_m", 0.0)
        # Unlike lookahead_map_check_m above (unmapped/unsafe area, stays
        # off by default), this checks already-MOPPED area ahead and is on
        # by default -- explicitly requested: treat ground the robot's
        # already covered like an obstacle, so it turns for fresh
        # territory instead of grinding back over its own trail.
        self.declare_parameter("coverage_lookahead_m", 0.6)
        self.declare_parameter("coverage_lookahead_confirm_fraction", 0.6)
        self.declare_parameter("map_side_check_m", 1.5)  # how far to sample the saved map left/right when picking turn direction
        # How strongly "this side is already mopped" outweighs "this side
        # is open" when picking a turn direction (_pick_turn_direction_from_map).
        # 1.0 = covered_fraction counts the same as free_fraction; higher
        # makes the robot noticeably more averse to turning back into its
        # own previous pass, even when that side is the more open one.
        self.declare_parameter("coverage_avoid_weight", 2.5)
        # Caps how many covered-cell cubes the green "already mopped"
        # marker draws per publish -- a big room fully covered at 5cm
        # resolution can have tens of thousands of True cells, way more
        # than useful to render individually. Downsamples (skips cells)
        # rather than truncating, so the whole covered area still shows,
        # just at coarser density once it's large.
        self.declare_parameter("covered_marker_max_points", 8000)

        # Collision/stall detection: odometry is dead-reckoned from wheel
        # encoders, so if the robot is physically wedged against something
        # (wheels still turning, not actually translating -- a low object
        # the LiDAR's scan plane misses, carpet snag, etc), odometry can
        # keep reporting forward progress that never really happened.
        # Cross-check against the live LiDAR: sample (position, front
        # distance) every stall_check_period_s: if odometry says the
        # robot moved at least stall_min_progress_m AND there's something
        # being tracked in front (front < stall_watch_range_m) but that
        # front distance barely closed the gap (less than
        # stall_min_front_decrease_ratio of the distance odometry claims
        # was covered), that mismatch means the robot isn't actually
        # approaching anything -- physically stuck, not driving. Handled
        # exactly like hitting a wall (stop -> reverse -> turn).
        self.declare_parameter("stall_check_period_s", 0.5)
        self.declare_parameter("stall_min_progress_m", 0.05)
        self.declare_parameter("stall_watch_range_m", 1.0)
        self.declare_parameter("stall_min_front_decrease_ratio", 0.3)

        # Stuck detection: if the robot U-turns this many times in a row
        # without making at least min_progress_m of real forward progress
        # each time (e.g. cornered in a tight pocket where every direction
        # re-triggers a stop almost immediately), stop the mission instead
        # of spinning stop/reverse/turn indefinitely.
        self.declare_parameter("min_progress_m", 0.2)
        self.declare_parameter("max_stuck_turns", 6)  # was 3 -- gave up too early in a genuinely cluttered corner; more retries in case a later direction still had room

        self.declare_parameter("completion_percent", 95.0)
        # Both AMCL-only: max_pose_covariance has no equivalent in "tf"
        # mode (no covariance in a raw TF lookup) -- there, "localized"
        # just means the map->base_footprint transform exists at all.
        self.declare_parameter("require_localized", True)
        # A fresh 2D Pose Estimate click alone can leave AMCL's covariance
        # well above a strict threshold -- it only narrows further with
        # actual robot motion, which creates a deadlock in "coverage" mode
        # (robot won't drive until localized, AMCL won't converge without
        # driving). Loosened from 0.5 so a reasonable click is accepted
        # right away; localization keeps refining once the robot starts
        # moving anyway, and nothing here still leans on tight AMCL
        # precision for safety (that's LiDAR-only now) -- only lane
        # bookkeeping/coverage-percent/turn-direction picks use pose, none
        # of them safety-critical.
        self.declare_parameter("max_pose_covariance", 3.0)
        self.declare_parameter("status_publish_period", 1.0)
        self.declare_parameter("control_period", 0.1)
        # When true, the full state machine still runs (map/localization
        # gating, lane tracking, the commit-hysteresis veer decision, the
        # RViz decision label) but _publish_motor() never actually sends
        # anything to /motor_rpm -- for pushing the robot by hand and
        # watching what it WOULD have decided, before trusting it to
        # actually drive.
        self.declare_parameter("advisory_only", False)

        self.mop_width = float(self.get_parameter("mop_width").value)
        self.wall_margin = float(self.get_parameter("wall_margin").value)
        self.robot_half_width = float(self.get_parameter("robot_half_width").value)
        self.robot_width = float(self.get_parameter("robot_width").value)
        self.shift_distance_m = float(self.get_parameter("shift_distance_m").value)
        self.min_shift_for_edge_check_m = float(self.get_parameter("min_shift_for_edge_check_m").value)

        self.nose_length_m = float(self.get_parameter("nose_length_m").value)
        self.front_stop_m = float(self.get_parameter("front_stop_m").value)
        # What the raw LiDAR range actually needs to read to keep
        # front_stop_m of clearance in front of the physical nose tip.
        self.front_stop_from_lidar_m = self.nose_length_m + self.front_stop_m
        self.wall_confirm_fraction = float(self.get_parameter("wall_confirm_fraction").value)
        self.front_sector_deg = float(self.get_parameter("front_sector_deg").value)
        self.reverse_seconds = float(self.get_parameter("reverse_seconds").value)
        self.lookahead_map_check_m = float(self.get_parameter("lookahead_map_check_m").value)
        self.coverage_lookahead_m = float(self.get_parameter("coverage_lookahead_m").value)
        self.coverage_lookahead_confirm_fraction = float(self.get_parameter("coverage_lookahead_confirm_fraction").value)
        self.map_side_check_m = float(self.get_parameter("map_side_check_m").value)
        self.coverage_avoid_weight = float(self.get_parameter("coverage_avoid_weight").value)
        self.covered_marker_max_points = int(self.get_parameter("covered_marker_max_points").value)
        self.stall_check_period_s = float(self.get_parameter("stall_check_period_s").value)
        self.stall_min_progress_m = float(self.get_parameter("stall_min_progress_m").value)
        self.stall_watch_range_m = float(self.get_parameter("stall_watch_range_m").value)
        self.stall_min_front_decrease_ratio = float(self.get_parameter("stall_min_front_decrease_ratio").value)
        self.min_progress_m = float(self.get_parameter("min_progress_m").value)
        self.max_stuck_turns = int(self.get_parameter("max_stuck_turns").value)

        self.completion_percent = float(self.get_parameter("completion_percent").value)
        self.require_localized = bool(self.get_parameter("require_localized").value)
        self.max_pose_covariance = float(self.get_parameter("max_pose_covariance").value)
        self.advisory_only = bool(self.get_parameter("advisory_only").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.localization_source = str(self.get_parameter("localization_source").value)

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self.on_map, map_qos)
        self.tf_buffer: Optional[Buffer] = None
        if self.localization_source == "tf":
            # slam_toolbox is the pose source -- no /amcl_pose to subscribe
            # to, read map->base_footprint straight off TF each control
            # tick instead (see _update_pose_from_tf()). A TransformListener
            # is only created here, not unconditionally: it processes the
            # entire /tf tree, and running one even in "amcl" mode (which
            # has no use for it) was enough added load on this machine to
            # stall the control loop and stop the robot from driving.
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.create_subscription(
                PoseWithCovarianceStamped, self.get_parameter("amcl_pose_topic").value, self.on_amcl_pose, 10
            )
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.on_scan, 10)

        self.status_pub = self.create_publisher(String, "/coverage/status", 10)
        self.percent_pub = self.create_publisher(Float32, "/coverage/percent", 10)
        self.complete_pub = self.create_publisher(Bool, "/coverage/complete", 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/coverage/grid", map_qos)
        self.motor_pub = self.create_publisher(Int16MultiArray, "/motor_rpm", 10)
        self.body_viz_pub = self.create_publisher(Marker, "/robot_body", 10)
        self.covered_viz_pub = self.create_publisher(Marker, "/coverage/covered_marker", 10)

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
        self._lane_start_pose: Optional[Tuple[float, float, float]] = None
        self._stuck_count = 0
        self._pivot_start_yaw: Optional[float] = None
        self._shift_start_xy: Optional[Tuple[float, float]] = None
        self._shift_start_was_covered = False
        self._stall_ref_time = 0.0
        self._stall_ref_pos: Optional[Tuple[float, float]] = None
        self._stall_ref_front: Optional[float] = None

        self.create_timer(float(self.get_parameter("control_period").value), self.control_loop)
        self.create_timer(float(self.get_parameter("status_publish_period").value), self.publish_status)
        self.create_timer(0.5, self._publish_zone_markers)  # 2Hz -- this laptop runs hot (RViz+AMCL+RPLidar all competing), keep it light

        if self.advisory_only:
            self.get_logger().info(
                "coverage_planner_node (ADVISORY ONLY -- push the robot by hand, "
                "no /motor_rpm will be sent) ready, waiting for /map and localization"
            )
        else:
            self.get_logger().info("coverage_planner_node (reactive) ready, waiting for /map and localization")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def on_map(self, msg: OccupancyGrid) -> None:
        if self.processor is not None:
            return
        processor = MapProcessor(msg, self.wall_margin, self.robot_half_width)
        if not processor.has_free_space:
            # In "tf" (live SLAM) mode the very first /map snapshot(s) can
            # be almost empty -- slam_toolbox hasn't had enough scans yet
            # to open up any area that survives the wall_margin +
            # robot_half_width erosion. Locking onto that would leave
            # row_min/row_max/col_min/col_max unset on the processor (only
            # set when has_free_space is True) and crash the first time
            # _start_first_lane()/_past_far_boundary() touch them. Just
            # wait for a later map that actually has safe-free area,
            # instead of latching onto this one.
            self.get_logger().warn(
                "Map received but has zero safe-free cells (too early, or nothing "
                "survives the wall_margin+robot_half_width erosion yet) -- waiting "
                "for a better one.",
                throttle_duration_sec=5.0,
            )
            return
        self.processor = processor
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

    def _update_pose_from_tf(self) -> None:
        """localization_source=='tf' counterpart to on_amcl_pose(): reads
        map->base_footprint straight off TF (published live by slam_toolbox,
        the same way AMCL publishes it) instead of a /amcl_pose message.
        There's no AMCL covariance here, so "localized" just means a valid
        transform exists yet -- slam_toolbox needs a few scans after
        startup before that's true, same as AMCL needing an initial pose.
        """
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, "base_footprint", Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        p = t.transform.translation
        yaw = yaw_from_quaternion(t.transform.rotation)
        self.current_pose = (p.x, p.y, yaw)
        self.is_localized = True

        if self.tracker is not None:
            self.tracker.mark(p.x, p.y)

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
        if self.localization_source == "tf":
            self._update_pose_from_tf()

        if self.state == CoverageState.WAITING_MAP:
            return

        if self.state == CoverageState.WAITING_LOCALIZATION:
            if self.current_pose is None or not self.is_localized:
                if self.localization_source == "tf":
                    self.get_logger().warn(
                        f"Waiting for the {self.global_frame}->base_footprint TF (slam_toolbox needs a "
                        "few scans after startup before this exists)",
                        throttle_duration_sec=5.0,
                    )
                else:
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
            if self._pivot_done():
                self._enter(CoverageState.SHIFTING)
        elif self.state == CoverageState.SHIFTING:
            front = sector_min_range(self.latest_scan, 180.0, self.front_sector_deg)
            blocked = False
            if front < self.front_stop_from_lidar_m:
                fraction = sector_wall_fraction(
                    self.latest_scan, 180.0, self.front_sector_deg, self.front_stop_from_lidar_m
                )
                blocked = fraction >= self.wall_confirm_fraction
            if blocked:
                self.get_logger().info(
                    f"Shift stopped early (wall ahead at {front:.2f}m) -- proceeding with partial offset",
                    throttle_duration_sec=1.0,
                )
                self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
                self._enter(CoverageState.ALIGNING)
            else:
                self._publish_motor(MOTOR_FORWARD, MOTOR_FORWARD)
                if self._shift_done():
                    self._enter(CoverageState.ALIGNING)
        elif self.state == CoverageState.ALIGNING:
            kiri, kanan = SPIN_RIGHT if self._turn_direction == "right" else SPIN_LEFT
            self._publish_motor(kiri, kanan)
            if self._pivot_done():
                if self._stuck_count >= self.max_stuck_turns:
                    self.get_logger().warn(
                        f"Stuck: {self._stuck_count} U-turns in a row with barely any forward "
                        f"progress (< {self.min_progress_m}m each) -- stopping here instead of "
                        "spinning indefinitely. Likely cornered in a tight pocket."
                    )
                    self.finish_mission()
                    return

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
        if state == CoverageState.DRIVING and self.current_pose is not None:
            self._lane_start_pose = self.current_pose
            # Fresh drive, fresh stall-check baseline -- a reference taken
            # before the turn (different heading entirely) would compare
            # apples to oranges.
            self._stall_ref_time = 0.0
            self._stall_ref_pos = None
            self._stall_ref_front = None
        if state in (CoverageState.TURNING, CoverageState.ALIGNING) and self.current_pose is not None:
            self._pivot_start_yaw = self.current_pose[2]
        if state == CoverageState.SHIFTING and self.current_pose is not None:
            self._shift_start_xy = (self.current_pose[0], self.current_pose[1])
            self._shift_start_was_covered = False
            if self.tracker is not None:
                cell = self.processor.world_to_cell(self.current_pose[0], self.current_pose[1])
                if cell is not None:
                    self._shift_start_was_covered = bool(self.tracker.covered[cell[0], cell[1]])

    def _pivot_done(self) -> bool:
        """True once the current TURNING/ALIGNING pivot has rotated ~90deg
        in self._turn_direction, judged from odometry -- a fixed timer
        can't reliably hit a precise angle. Falls back to a time ceiling
        (TURN_PIVOT_TIMEOUT_SECONDS) if pose isn't updating for some
        reason, so a stalled pose can't spin the robot forever.

        Uses self.current_pose (AMCL/SLAM-corrected), not a separate raw
        odom TF lookup -- a raw-odometry version was tried and reverted:
        keeping an always-on TransformListener running (needed even in
        "amcl" mode, which previously had none) processing the full /tf
        tree added enough load on this machine that the robot stopped
        driving at all. Not worth it for a precision gain on the turn
        maneuver alone.
        """
        elapsed = time.monotonic() - self._state_entered_at
        if elapsed >= TURN_PIVOT_TIMEOUT_SECONDS:
            return True
        if self._pivot_start_yaw is None or self.current_pose is None:
            return False
        delta = normalize_angle(self.current_pose[2] - self._pivot_start_yaw)
        target = math.pi / 2.0 if self._turn_direction == "left" else -math.pi / 2.0
        if target > 0:
            return delta >= target - TURN_PIVOT_TOLERANCE_RAD
        return delta <= target + TURN_PIVOT_TOLERANCE_RAD

    def _shift_done(self) -> bool:
        """True once SHIFTING has moved past the edge of the previously
        mopped strip -- checked live against the CoverageTracker (the
        same data the green "CoveredPath" marker draws), not just a fixed
        shift_distance_m. A fixed distance kept needing re-tuning (0.60 ->
        0.50 -> 0.40) because real execution slack meant it never quite
        landed exactly on the edge; checking the actual coverage data is
        self-correcting regardless of that slack -- it stops exactly when
        it gets there, whatever the real distance turns out to be.

        Falls back to plain shift_distance_m when there's nothing to
        detect an edge FROM (self._shift_start_was_covered is False --
        e.g. the very first lane, nothing's been mopped anywhere yet), so
        it doesn't stop after a token few cm of shift with no real strip
        to have exited. shift_distance_m and SHIFT_TIMEOUT_SECONDS still
        apply as absolute ceilings either way.
        """
        elapsed = time.monotonic() - self._state_entered_at
        if elapsed >= SHIFT_TIMEOUT_SECONDS:
            return True
        if self._shift_start_xy is None or self.current_pose is None:
            return False
        sx, sy = self._shift_start_xy
        cx, cy = self.current_pose[0], self.current_pose[1]
        dist_moved = math.hypot(cx - sx, cy - sy)
        if dist_moved >= self.shift_distance_m:
            return True

        if not self._shift_start_was_covered or self.tracker is None:
            return False
        if dist_moved < self.min_shift_for_edge_check_m:
            return False

        # Small patch around the current position, majority vote -- one
        # noisy uncovered cell right at the edge shouldn't be enough to
        # call it done a beat early.
        total = 0
        covered = 0
        for dx in (-0.05, 0.0, 0.05):
            for dy in (-0.05, 0.0, 0.05):
                cell = self.processor.world_to_cell(cx + dx, cy + dy)
                if cell is None:
                    continue
                total += 1
                if self.tracker.covered[cell[0], cell[1]]:
                    covered += 1
        if total == 0:
            return False
        return (covered / total) < 0.3

    def _drive(self) -> None:
        """Straight-until-wall, then U-turn -- and nothing else.

        All the side-veer/lane-heading-correction machinery (gentle/sharp
        left-right avoidance, commit-hysteresis, lane cross-track nudging)
        was ripped out on purpose: it was the single biggest source of
        instability this whole build (zigzag, backwards corrections,
        oscillation) despite round after round of tuning. Getting the core
        loop -- drive straight, detect a real wall ahead, stop/reverse/turn,
        drive straight again -- rock solid comes first; side-avoidance can
        come back later, deliberately, once this is trustworthy.
        """
        front = sector_min_range(self.latest_scan, 180.0, self.front_sector_deg)
        map_blocked_ahead = self._lookahead_blocked()
        coverage_blocked_ahead = self._coverage_lookahead_blocked()

        wall_ahead = False
        if front < self.front_stop_from_lidar_m:
            fraction = sector_wall_fraction(self.latest_scan, 180.0, self.front_sector_deg, self.front_stop_from_lidar_m)
            wall_ahead = fraction >= self.wall_confirm_fraction
            if not wall_ahead:
                self.get_logger().info(
                    f"Ignoring close spike at {front:.2f}m (wall_fraction={fraction:.2f}, "
                    f"< {self.wall_confirm_fraction}) -- looks like a small object, not a wall",
                    throttle_duration_sec=1.0,
                )

        stalled = self._check_stall(front)

        if wall_ahead or map_blocked_ahead or coverage_blocked_ahead or stalled:
            if wall_ahead:
                reason = "wall ahead"
            elif stalled:
                reason = "stalled -- odom moving but LiDAR front isn't closing (likely physically stuck)"
            elif coverage_blocked_ahead:
                reason = "already mopped ahead"
            else:
                reason = "map lookahead"

            progress = 0.0
            if self._lane_start_pose is not None:
                sx, sy, _ = self._lane_start_pose
                cx, cy, _ = self.current_pose
                progress = math.hypot(cx - sx, cy - sy)
            if progress < self.min_progress_m:
                self._stuck_count += 1
            else:
                self._stuck_count = 0

            self.get_logger().info(
                f"Stopping ({reason}): front={front:.2f}m "
                f"-- progress since last turn={progress:.2f}m, stuck_count={self._stuck_count}"
            )
            self._pick_turn_direction_from_map()
            self._enter(CoverageState.STOPPING)
            return

        self._publish_motor(MOTOR_FORWARD, MOTOR_FORWARD)

    def _check_stall(self, front: float) -> bool:
        """True if odometry says the robot moved but the live LiDAR front
        reading didn't close the gap to match -- see the
        stall_check_period_s param's declaration comment for the full
        rationale (wheel-encoder odometry can report progress that never
        physically happened if the robot is wedged against something).
        Samples a fresh (position, front) reference every
        stall_check_period_s and compares against the previous one.
        """
        now = time.monotonic()
        if self._stall_ref_pos is None:
            self._stall_ref_time = now
            self._stall_ref_pos = (self.current_pose[0], self.current_pose[1])
            self._stall_ref_front = front
            return False

        if (now - self._stall_ref_time) < self.stall_check_period_s:
            return False

        sx, sy = self._stall_ref_pos
        cx, cy = self.current_pose[0], self.current_pose[1]
        dist_moved = math.hypot(cx - sx, cy - sy)
        front_decrease = self._stall_ref_front - front

        # abs(), not a one-sided "< threshold": the front reading opening
        # up a lot (front_decrease strongly negative -- rounded a corner,
        # cleared whatever was there) satisfied the old one-sided check
        # just as easily as truly being stuck did, and got flagged as a
        # "stall" for successfully making progress. Stuck means the front
        # reading stayed close to flat despite the claimed movement,
        # whichever direction it drifted.
        stalled = (
            self._stall_ref_front < self.stall_watch_range_m
            and dist_moved >= self.stall_min_progress_m
            and abs(front_decrease) < dist_moved * self.stall_min_front_decrease_ratio
        )
        if stalled:
            self.get_logger().warn(
                f"Stall detected: odom moved {dist_moved:.2f}m but front LiDAR only closed "
                f"{front_decrease:.2f}m ({self._stall_ref_front:.2f}m -> {front:.2f}m)"
            )

        self._stall_ref_time = now
        self._stall_ref_pos = (cx, cy)
        self._stall_ref_front = front
        return stalled

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

    def _side_covered_fraction(self, x: float, y: float, yaw: float, side_sign: int) -> float:
        """Fraction of the CoverageTracker's already-mopped cells sampled
        in a patch to one side -- same sampling pattern as
        _map_side_free_fraction, but reads self.tracker.covered instead of
        self.processor.safe_free.
        """
        side_yaw = yaw + side_sign * (math.pi / 2.0)
        total = 0
        covered = 0
        for lateral in np.arange(0.2, self.map_side_check_m + 1e-6, 0.15):
            for forward in (-0.3, 0.0, 0.3, 0.6):
                px = x + forward * math.cos(yaw) + lateral * math.cos(side_yaw)
                py = y + forward * math.sin(yaw) + lateral * math.sin(side_yaw)
                total += 1
                cell = self.processor.world_to_cell(px, py)
                if cell is not None and self.tracker.covered[cell[0], cell[1]]:
                    covered += 1
        return (covered / total) if total > 0 else 0.0

    def _pick_turn_direction_from_map(self) -> None:
        # Picking purely on which side the SAVED MAP shows as more open
        # (the old rule) has no idea whether that "open" side is ground
        # the robot has already mopped -- it can easily turn back into its
        # own previous pass over and over. Bias toward whichever side is
        # BOTH safe to turn into (free_fraction) AND still mostly unswept
        # (low covered_fraction), using the CoverageTracker's own record
        # of where the robot has actually already been.
        x, y, yaw = self.current_pose
        left_free = self._map_side_free_fraction(x, y, yaw, +1)
        right_free = self._map_side_free_fraction(x, y, yaw, -1)
        left_covered = self._side_covered_fraction(x, y, yaw, +1)
        right_covered = self._side_covered_fraction(x, y, yaw, -1)
        left_score = left_free - self.coverage_avoid_weight * left_covered
        right_score = right_free - self.coverage_avoid_weight * right_covered
        self._turn_direction = "left" if left_score >= right_score else "right"
        self.get_logger().info(
            f"Turn direction pick: left(free={left_free:.2f} covered={left_covered:.2f} score={left_score:.2f}) "
            f"right(free={right_free:.2f} covered={right_covered:.2f} score={right_score:.2f}) -> {self._turn_direction}"
        )

    def _lookahead_blocked(self) -> bool:
        """Keeps the robot inside the saved map's known-free (white) area.

        Without this, the reactive driver only reacts to live LiDAR --
        which happily drives the robot through an opening into unmapped
        (grey/unknown) territory, since nothing in the saved map stops it
        if the live scan reads clear right there.

        An earlier version checked a single point straight ahead and
        disabled itself after AMCL pose drift made that one point land on
        the wrong map cell, freezing the robot even with clearly open live
        LiDAR. This version samples a small PATCH ahead (roughly the
        robot's own width) and only calls it blocked when most of the
        patch is unsafe -- a one-cell pose error no longer flips the
        result, the same robustness trick as _map_side_free_fraction.
        """
        if self.lookahead_map_check_m <= 0.0 or self.current_pose is None:
            return False

        x, y, yaw = self.current_pose
        total = 0
        unsafe = 0
        for forward in np.arange(0.15, self.lookahead_map_check_m + 1e-6, 0.1):
            for lateral in (-0.15, 0.0, 0.15):
                side_yaw = yaw + math.pi / 2.0
                px = x + forward * math.cos(yaw) + lateral * math.cos(side_yaw)
                py = y + forward * math.sin(yaw) + lateral * math.sin(side_yaw)
                total += 1
                if not self.processor.is_safe(px, py):
                    unsafe += 1

        fraction_unsafe = (unsafe / total) if total > 0 else 0.0
        blocked = fraction_unsafe > 0.5
        if blocked:
            self.get_logger().info(
                f"Map lookahead blocked ({fraction_unsafe:.0%} of patch unmapped/unsafe) -- "
                f"robot pose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)"
            )
        return blocked

    def _coverage_lookahead_blocked(self) -> bool:
        """Treats already-mopped ground ahead like a wall: if most of a
        patch a short distance ahead of the robot is already marked
        covered (CoverageTracker, the same data the green "CoveredPath"
        marker draws and _pick_turn_direction_from_map already leans on),
        stop/reverse/turn instead of grinding forward over ground that's
        already done. Same area-patch + majority-vote sampling as
        _lookahead_blocked() (one stray marked cell can't flip the
        result), and the same stuck-detection safety net downstream
        applies here too, so this can't loop forever if every direction
        looks covered.
        """
        if self.coverage_lookahead_m <= 0.0 or self.current_pose is None or self.tracker is None:
            return False

        x, y, yaw = self.current_pose
        total = 0
        covered = 0
        for forward in np.arange(0.15, self.coverage_lookahead_m + 1e-6, 0.1):
            for lateral in (-0.15, 0.0, 0.15):
                side_yaw = yaw + math.pi / 2.0
                px = x + forward * math.cos(yaw) + lateral * math.cos(side_yaw)
                py = y + forward * math.sin(yaw) + lateral * math.sin(side_yaw)
                cell = self.processor.world_to_cell(px, py)
                if cell is None:
                    continue
                total += 1
                if self.tracker.covered[cell[0], cell[1]]:
                    covered += 1

        fraction_covered = (covered / total) if total > 0 else 0.0
        blocked = fraction_covered >= self.coverage_lookahead_confirm_fraction
        if blocked:
            self.get_logger().info(
                f"Coverage lookahead blocked ({fraction_covered:.0%} of patch already mopped) -- "
                f"robot pose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)"
            )
        return blocked

    def _publish_motor(self, kiri: int, kanan: int) -> None:
        if self.advisory_only:
            return
        msg = Int16MultiArray()
        msg.data = [int(kiri), int(kanan)]
        self.motor_pub.publish(msg)

    # ------------------------------------------------------------------
    # Live RViz visualization: draws the robot body outline and the
    # front-stop zone _drive() actually checks (not just raw LaserScan
    # dots), plus a text label of the live state -- so what the robot is
    # "thinking" is visible, not just inferred from behavior or log lines.
    # Everything is in the base_footprint frame (robot center, x=forward,
    # y=left per REP-103), matching how sector_min_range already reasons
    # about the front sector -- RViz transforms it into the fixed frame
    # via TF.
    # ------------------------------------------------------------------

    def _arc_marker(
        self,
        marker_id: int,
        ns: str,
        radius: float,
        start_deg: float,
        end_deg: float,
        rgba: Tuple[float, float, float, float],
        width: float = 0.02,
        z: float = 0.05,
        segments: int = 48,
    ) -> Marker:
        m = Marker()
        m.header.frame_id = "base_footprint"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.pose.orientation.w = 1.0
        start = math.radians(start_deg)
        end = math.radians(end_deg)
        for i in range(segments + 1):
            ang = start + (end - start) * i / segments
            m.points.append(Point(x=radius * math.cos(ang), y=radius * math.sin(ang), z=z))
        return m

    def _decision_label(self) -> Tuple[str, Tuple[float, float, float, float]]:
        """Human-readable (Indonesian) decision label + color for the current state/level."""
        GREEN = (0.2, 0.9, 0.2, 1.0)
        YELLOW = (1.0, 0.9, 0.0, 1.0)
        RED = (1.0, 0.15, 0.15, 1.0)
        BLUE = (0.3, 0.6, 1.0, 1.0)
        GREY = (0.6, 0.6, 0.6, 1.0)

        if self.state == CoverageState.STOPPING:
            return "BERHENTI (tembok)", RED
        if self.state == CoverageState.REVERSING:
            return "MUNDUR", RED
        if self.state == CoverageState.TURNING:
            return f"PUTAR BALIK 1/2 ({self._turn_direction})", RED
        if self.state == CoverageState.SHIFTING:
            return f"GESER ({self._turn_direction})", RED
        if self.state == CoverageState.ALIGNING:
            return f"PUTAR BALIK 2/2 ({self._turn_direction})", RED
        if self.state == CoverageState.FINISHED:
            return "SELESAI", BLUE
        if self.state in (CoverageState.WAITING_MAP, CoverageState.WAITING_LOCALIZATION):
            return "MENUNGGU PETA/LOKALISASI", GREY

        # DRIVING: side-veer/lane-correction are gone, so this is always
        # just LURUS until the front-wall check in _drive() takes over
        # (STOPPING/REVERSING/TURNING above).
        return "LURUS", GREEN

    def _publish_zone_markers(self) -> None:
        front_half = self.front_sector_deg

        markers = [
            # Robot body outline (yellow, matches robotmaganglidar.py's convention).
            self._arc_marker(0, "body", self.robot_half_width, 0.0, 360.0, (1.0, 1.0, 0.0, 0.9), width=0.015),
            # front_stop_from_lidar_m: the actual stop/reverse/turn trigger distance.
            self._arc_marker(
                5,
                "front_stop",
                self.front_stop_from_lidar_m,
                -front_half,
                front_half,
                (1.0, 0.0, 0.0, 0.7),
                width=0.02,
            ),
        ]
        for m in markers:
            self.body_viz_pub.publish(m)

        label, rgba = self._decision_label()
        text = Marker()
        text.header.frame_id = "base_footprint"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "decision"
        text.id = 6
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.0
        text.pose.position.y = 0.0
        text.pose.position.z = 0.6
        text.pose.orientation.w = 1.0
        text.scale.z = 0.25
        text.color.r, text.color.g, text.color.b, text.color.a = rgba
        text.text = label
        self.body_viz_pub.publish(text)

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
            stamp = self.get_clock().now().to_msg()
            self.grid_pub.publish(self.tracker.to_occupancy_grid(stamp))
            self.covered_viz_pub.publish(self._build_covered_marker(stamp))

        complete_msg = Bool()
        complete_msg.data = self.state == CoverageState.FINISHED
        self.complete_pub.publish(complete_msg)

    def _build_covered_marker(self, stamp) -> Marker:
        """Solid GREEN cubes over every cell the CoverageTracker has
        marked as already mopped -- the "avoid this, already done" trail
        the robot's turn-direction picking (_pick_turn_direction_from_map,
        coverage_avoid_weight) is actually steering away from, made
        visible. Separate from the CoverageGrid OccupancyGrid display
        (which RViz tints via its own costmap color scheme, not a color
        this file controls) so it can just be plain green.
        """
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = stamp
        m.ns = "covered_path"
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        res = self.processor.resolution
        m.scale.x = res
        m.scale.y = res
        m.scale.z = 0.01
        m.color.r, m.color.g, m.color.b, m.color.a = (0.0, 0.85, 0.15, 0.65)
        m.pose.orientation.w = 1.0

        rows, cols = np.where(self.tracker.covered)
        count = rows.size
        stride = max(1, math.ceil(count / self.covered_marker_max_points)) if count > 0 else 1
        for row, col in zip(rows[::stride].tolist(), cols[::stride].tolist()):
            x, y = self.processor.cell_to_world(row, col)
            m.points.append(Point(x=x, y=y, z=0.0))
        return m


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
