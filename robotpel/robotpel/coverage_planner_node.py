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
"obstacles" at close range, confusing the veer/turn-direction logic --
scan_blind_spot_filter's radius exclusion (exclude_radius_m, ~0.32m) blanks
both. This is safe now that front_stop_m is nose-relative
(front_stop_from_lidar_m = nose_length_m + front_stop_m, ~0.55m): the
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


def nearest_in_side_zone(
    scan: LaserScan,
    side: str,
    radius: float,
    min_points: int = 1,
    cone_half_deg: float = 90.0,
) -> float:
    """Nearest range on `side` ('left' or 'right') of the robot within `radius`.

    Converts each ray to base_footprint-relative Cartesian (x forward, y
    lateral, +y=left / -y=right per REP-103) using the established
    lidar_angle -> base_footprint_angle = lidar_angle + 180deg relationship
    (matches the laser_yaw=pi static transform this robot uses), the same
    approach robotmaganglidar.py uses for its own left/center/right
    classification -- picking a side by SIGN OF Y is far less error-prone
    than picking a LiDAR-local angle range and hoping it lines up with the
    real physical side, which is what caused repeated "kebalik" (backwards)
    reports here.

    `cone_half_deg` bounds how far off dead-ahead (bf_angle = 0) a ray may
    be and still count, e.g. 90 = the full forward hemisphere ("something
    behind me can't be hit by driving forward"), 45 = a 90-degree
    quarter-circle cone straight ahead (user sketched this as two lines
    ~45deg either side of forward on a protractor) so a ray coming in from
    near the robot's flank, which driving straight ahead would never
    actually reach, doesn't count as something to veer away from.

    If left/right still comes out swapped after this, the fix is the
    `laser_inverted` launch arg (it flips the raw scan's own angle sign,
    i.e. an actual Y-axis mirror) -- NOT the wheel mapping below, which
    already matches robotmaganglidar1.py's tested motor convention.

    `min_points` mirrors robotmaganglidar.py's left_pts/right_pts approach
    (and this file's own sector_wall_fraction): a single stray ray -- a
    reflection, a spec of noise -- shouldn't be enough to swerve the robot.
    Below `min_points` confirming rays in the zone, this returns inf (i.e.
    "nothing real detected here") even if one lone ray was technically in
    range.
    """
    want_left = side == "left"
    cone_half = math.radians(cone_half_deg)
    best = float("inf")
    count = 0
    angle = scan.angle_min
    for r in scan.ranges:
        if scan.range_min <= r <= scan.range_max and r <= radius:
            bf_angle = normalize_angle(angle + math.pi)
            x = r * math.cos(bf_angle)
            y = r * math.sin(bf_angle)
            if abs(bf_angle) <= cone_half and (
                (want_left and y > 0.0) or (not want_left and y < 0.0)
            ):
                count += 1
                if r < best:
                    best = r
        angle += scan.angle_increment
    if count < min_points:
        return float("inf")
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
        self.declare_parameter("robot_half_width", 0.25)
        self.declare_parameter("robot_width", 0.50)

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
        self.declare_parameter("front_stop_m", 0.25)  # desired clearance beyond the nose tip, not from the LiDAR (was 0.15 -- pushed further out so stop/reverse/turn kicks in earlier)
        self.declare_parameter("wall_confirm_fraction", 0.6)  # fraction of front-sector rays that must be close to call it a wall, not a spike
        # Two-radius correction, both from the robot's body center (LiDAR
        # sits at laser_x=laser_y=0) and classified by real left/right via
        # nearest_in_side_zone (proper x,y, not a guessed angle) -- see
        # _drive(): side_close_m (~0.35m, robot is 50cm wide so this is
        # just past the body's own edge) is a tight close-range safety
        # net; veer_circle_radius_m (~0.7m) is the farther anticipatory
        # zone. side_close_m retuned repeatedly (0.40->0.35->0.30 (too
        # tight, brought back up)->0.35) chasing "too sensitive" vs "too
        # tight to be a real safety margin". veer_circle_radius_m went
        # 1.0->0.6->0.5 while it was still flapping/zigzagging, then back
        # up to 0.7 once side_confirm_points + front_cone_half_deg +
        # veer_commit_seconds below were added to actually fix the
        # instability -- the radius itself was never really the bug.
        self.declare_parameter("side_close_m", 0.35)
        self.declare_parameter("veer_circle_radius_m", 0.7)
        self.declare_parameter("veer_sharp_m", 0.38)
        # A lone noisy ray (reflection, dust, LiDAR spec) shouldn't be
        # enough to swerve the robot for no real reason -- require at
        # least this many confirming rays in the zone, same spirit as
        # robotmaganglidar.py's left_pts/right_pts point lists and this
        # file's own wall_confirm_fraction for the front sector.
        self.declare_parameter("side_confirm_points", 2)
        # Far-zone rays only count within this many degrees of dead-ahead
        # (90 = full forward hemisphere, 60 = a 120-degree cone straight
        # ahead) -- something out near the robot's flank that a straight
        # path would never actually reach shouldn't trigger a veer.
        self.declare_parameter("front_cone_half_deg", 60.0)
        # Once a veer correction commits to a side, de-escalating
        # (sharp->gentle->straight) on that same side is held off for this
        # long -- stops rapid gentle/sharp flapping when a real wall sits
        # right at a threshold. See _drive()'s commit-hysteresis comment.
        self.declare_parameter("veer_commit_seconds", 0.4)
        self.declare_parameter("front_sector_deg", 20.0)
        self.declare_parameter("reverse_seconds", 0.6)
        # Disabled by default: the map is only used to place lanes and mark
        # covered cells now, not to block live driving -- an earlier version
        # of this check used the map to keep the robot off unmapped
        # territory, but it kept false-triggering near the lane start
        # (close to a wall by design) and, combined with the stuck-turn
        # safeguard, made the robot give up early instead of driving.
        # Obstacle avoidance is LIVE-LiDAR-only now (front/diag sectors
        # above). Set > 0 to bring the map check back if you need it.
        self.declare_parameter("lookahead_map_check_m", 0.0)
        self.declare_parameter("lane_correct_deg", 8.0)
        self.declare_parameter("map_side_check_m", 1.5)  # how far to sample the saved map left/right when picking turn direction

        # Stuck detection: if the robot U-turns this many times in a row
        # without making at least min_progress_m of real forward progress
        # each time (e.g. cornered in a tight pocket where every direction
        # re-triggers a stop almost immediately), stop the mission instead
        # of spinning stop/reverse/turn indefinitely.
        self.declare_parameter("min_progress_m", 0.2)
        self.declare_parameter("max_stuck_turns", 3)

        self.declare_parameter("completion_percent", 95.0)
        # Both AMCL-only: max_pose_covariance has no equivalent in "tf"
        # mode (no covariance in a raw TF lookup) -- there, "localized"
        # just means the map->base_footprint transform exists at all.
        self.declare_parameter("require_localized", True)
        self.declare_parameter("max_pose_covariance", 0.5)
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

        self.nose_length_m = float(self.get_parameter("nose_length_m").value)
        self.front_stop_m = float(self.get_parameter("front_stop_m").value)
        # What the raw LiDAR range actually needs to read to keep
        # front_stop_m of clearance in front of the physical nose tip.
        self.front_stop_from_lidar_m = self.nose_length_m + self.front_stop_m
        self.wall_confirm_fraction = float(self.get_parameter("wall_confirm_fraction").value)
        self.side_close_m = float(self.get_parameter("side_close_m").value)
        self.veer_circle_radius_m = float(self.get_parameter("veer_circle_radius_m").value)
        self.veer_sharp_m = float(self.get_parameter("veer_sharp_m").value)
        self.side_confirm_points = int(self.get_parameter("side_confirm_points").value)
        self.front_cone_half_deg = float(self.get_parameter("front_cone_half_deg").value)
        self.veer_commit_seconds = float(self.get_parameter("veer_commit_seconds").value)
        self.front_sector_deg = float(self.get_parameter("front_sector_deg").value)
        self.reverse_seconds = float(self.get_parameter("reverse_seconds").value)
        self.lookahead_map_check_m = float(self.get_parameter("lookahead_map_check_m").value)
        self.lane_correct_deg = float(self.get_parameter("lane_correct_deg").value)
        self.map_side_check_m = float(self.get_parameter("map_side_check_m").value)
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
            # tick instead (see _update_pose_from_tf()).
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
        # Commit-hysteresis state, see _commit_level(): obstacle-veer and
        # lane-keeping each get their own so they never clobber each other.
        self._veer_commit = {"level": "STRAIGHT", "until": 0.0}
        self._lane_commit = {"level": "STRAIGHT", "until": 0.0}

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
            if time.monotonic() - self._state_entered_at >= TURN_BACK_SECONDS:
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
            # A stop/reverse/turn maneuver changes the robot's heading
            # completely -- whatever veer/lane commit was in effect before
            # it is no longer meaningful, so start the fresh DRIVING run
            # with a clean slate rather than possibly holding a stale
            # commit into the new heading.
            self._veer_commit = {"level": "STRAIGHT", "until": 0.0}
            self._lane_commit = {"level": "STRAIGHT", "until": 0.0}

    def _commit_level(self, commit_state: dict, desired_level: str, rank: dict, commit_seconds: float) -> str:
        """Shared commit-hysteresis: hold a committed level on its side for
        `commit_seconds` before allowing a de-escalation back toward
        STRAIGHT/a lower rank, so borderline sensor/heading readings don't
        make the decision flap every control tick. See _drive()'s comment
        for the full rationale. `commit_state` is a small {"level",
        "until"} dict owned by the caller (self._veer_commit for obstacle
        avoidance, self._lane_commit for lane-keeping) so the two decision
        layers don't share -- and corrupt -- each other's state.
        """

        def _side_of(level: str) -> Optional[str]:
            if level == "STRAIGHT":
                return None
            return "LEFT" if level.endswith("LEFT") else "RIGHT"

        now = time.monotonic()
        prev_level = commit_state["level"]
        desired_side = _side_of(desired_level)
        prev_side = _side_of(prev_level)
        still_committed = (
            now < commit_state["until"]
            and prev_level != "STRAIGHT"
            and (desired_side is None or desired_side == prev_side)
            and rank[desired_level] <= rank[prev_level]
        )
        driven_level = prev_level if still_committed else desired_level
        if driven_level != prev_level:
            commit_state["until"] = now + commit_seconds
        commit_state["level"] = driven_level
        return driven_level

    def _drive(self) -> None:
        front = sector_min_range(self.latest_scan, 180.0, self.front_sector_deg)
        # Two radii per side, both via nearest_in_side_zone's proper (x,y)
        # classification (real left/right, not a guessed LiDAR-local angle
        # range): side_close_m (~0.35m) is a tight close-range safety net,
        # veer_circle_radius_m (~0.7m) is the farther anticipatory zone.
        # Any point on that side within the radius counts, near or
        # diagonal -- what matters is total distance, not a specific angle.
        # side_close_m stays single-ray-sensitive and full-hemisphere (it's
        # the last line of defense against actually touching something);
        # the farther veer_circle_radius_m zone requires side_confirm_points
        # confirming rays AND narrows to front_cone_half_deg (default 45 =
        # a 90-degree cone dead ahead, not the full 180 hemisphere) --
        # something out near the flank that driving straight would never
        # reach shouldn't cause a sudden unprovoked swerve.
        side_right = nearest_in_side_zone(self.latest_scan, "right", self.side_close_m)
        side_left = nearest_in_side_zone(self.latest_scan, "left", self.side_close_m)
        diag_right = nearest_in_side_zone(
            self.latest_scan,
            "right",
            self.veer_circle_radius_m,
            self.side_confirm_points,
            self.front_cone_half_deg,
        )
        diag_left = nearest_in_side_zone(
            self.latest_scan,
            "left",
            self.veer_circle_radius_m,
            self.side_confirm_points,
            self.front_cone_half_deg,
        )
        front_right = diag_right  # kept for the stop-state debug log below
        front_left = diag_left

        map_blocked_ahead = self._lookahead_blocked()

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

        if wall_ahead or map_blocked_ahead:
            reason = "wall ahead" if wall_ahead else "map lookahead"

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
                f"Stopping ({reason}): front={front:.2f}m fr={front_right:.2f}m fl={front_left:.2f}m "
                f"-- progress since last turn={progress:.2f}m, stuck_count={self._stuck_count}"
            )
            self._pick_turn_direction_from_map()
            self._enter(CoverageState.STOPPING)
            return

        # Wheel convention (robotmaganglidar1.py, tested): slowing the RIGHT
        # wheel turns the robot RIGHT; slowing the LEFT wheel turns it LEFT.
        # Something on the right -> turn LEFT (away) -> slow the LEFT
        # wheel. This mapping was never the actual bug -- the earlier
        # "kebalik" reports were the side classification itself (raw LiDAR
        # angle guessing) being unreliable; nearest_in_side_zone's proper
        # (x,y) classification above should have fixed that.
        if side_right < self.side_close_m or diag_right < self.veer_sharp_m:
            desired_level = "SHARP_LEFT"
        elif diag_right < self.veer_circle_radius_m:
            desired_level = "GENTLE_LEFT"
        elif side_left < self.side_close_m or diag_left < self.veer_sharp_m:
            desired_level = "SHARP_RIGHT"
        elif diag_left < self.veer_circle_radius_m:
            desired_level = "GENTLE_RIGHT"
        else:
            desired_level = "STRAIGHT"

        # Commit hysteresis: once a real, persistently-close wall sits
        # right at a threshold, distance readings wobble a few cm cycle to
        # cycle (10Hz) and the raw decision above flaps between
        # gentle/sharp/straight every tick -- that flapping, not a false
        # trigger (already filtered above), is what zigzags the robot.
        # Mirrors robotmaganglidar.py's "commit turn agar tidak langsung
        # balik ke tengah" (LIDAR_TURN_COMMIT_SEC) idea, adapted to this
        # file's discrete gentle/sharp levels instead of a continuous
        # steering angle: once committed to a side, de-escalating (sharp
        # -> gentle -> straight) on that SAME side is held off for
        # veer_commit_seconds. Escalating, or a real obstacle appearing on
        # the OPPOSITE side, always breaks through immediately -- safety
        # is never delayed, only the "let's relax now" decision is.
        # See _commit_level() -- the same hysteresis also guards
        # _drive_straight_with_lane_correction()'s lane-keeping nudges
        # (self._lane_commit), which was flapping/oscillating on its own
        # even with no obstacle in sight.
        rank = {"STRAIGHT": 0, "GENTLE_LEFT": 1, "GENTLE_RIGHT": 1, "SHARP_LEFT": 2, "SHARP_RIGHT": 2}
        driven_level = self._commit_level(self._veer_commit, desired_level, rank, self.veer_commit_seconds)

        # SHARP stops the inside wheel dead (MOTOR_NEUTRAL) instead of just
        # slowing it -- a pivot turn on one wheel, much tighter than a
        # differential slow-down, for when the robot really needs to get
        # out of the way fast. GENTLE keeps the softer slow-down.
        level_bytes = {
            "SHARP_LEFT": (MOTOR_NEUTRAL, MOTOR_FORWARD),
            "GENTLE_LEFT": (MOTOR_FORWARD_SLOW, MOTOR_FORWARD),
            "SHARP_RIGHT": (MOTOR_FORWARD, MOTOR_NEUTRAL),
            "GENTLE_RIGHT": (MOTOR_FORWARD, MOTOR_FORWARD_SLOW),
        }
        if driven_level in level_bytes:
            kiri, kanan = level_bytes[driven_level]
            self.get_logger().info(
                f"Veer {driven_level.replace('_', ' ')} (raw={desired_level}): "
                f"side_right={side_right:.2f}m diag_right={diag_right:.2f}m "
                f"side_left={side_left:.2f}m diag_left={diag_left:.2f}m -> kiri={kiri} kanan={kanan}",
                throttle_duration_sec=1.0,
            )
            self._publish_motor(kiri, kanan)
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
            desired_level = "STRAIGHT"
        elif deg > 0:
            # yaw > target_yaw = heading rotated CCW/left of target (REP-103:
            # positive yaw is CCW) -> need to turn RIGHT to come back.
            desired_level = "LANE_HARD_RIGHT" if deg > self.lane_correct_deg else "LANE_SOFT_RIGHT"
        else:
            desired_level = "LANE_HARD_LEFT" if -deg > self.lane_correct_deg else "LANE_SOFT_LEFT"

        # Same commit-hysteresis as the obstacle-veer decision in _drive(),
        # but its own state (self._lane_commit) -- this was oscillating
        # (bang-bang correcting back and forth across the lane_correct_deg
        # boundary every ~0.1s tick) even with zero obstacles around,
        # which is why the robot zigzagged while the RViz label still said
        # "LURUS" (that label only reflected the obstacle-veer layer,
        # never this one). Also fixes a real direction bug: the wheel
        # slowed for each branch was backwards -- slowing the LEFT wheel
        # turns the robot LEFT (established, tested convention, see
        # _drive()'s comment), so a "need to turn right" correction must
        # slow the RIGHT wheel, not the left.
        rank = {"STRAIGHT": 0, "LANE_SOFT_LEFT": 1, "LANE_SOFT_RIGHT": 1, "LANE_HARD_LEFT": 2, "LANE_HARD_RIGHT": 2}
        driven_level = self._commit_level(self._lane_commit, desired_level, rank, self.veer_commit_seconds)

        lane_bytes = {
            "LANE_SOFT_RIGHT": (MOTOR_FORWARD, MOTOR_FORWARD_SLOW),
            "LANE_HARD_RIGHT": (MOTOR_FORWARD, MOTOR_FORWARD_VERY_SLOW),
            "LANE_SOFT_LEFT": (MOTOR_FORWARD_SLOW, MOTOR_FORWARD),
            "LANE_HARD_LEFT": (MOTOR_FORWARD_VERY_SLOW, MOTOR_FORWARD),
        }
        kiri, kanan = lane_bytes.get(driven_level, (MOTOR_FORWARD, MOTOR_FORWARD))
        self._publish_motor(kiri, kanan)

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

    def _publish_motor(self, kiri: int, kanan: int) -> None:
        if self.advisory_only:
            return
        msg = Int16MultiArray()
        msg.data = [int(kiri), int(kanan)]
        self.motor_pub.publish(msg)

    # ------------------------------------------------------------------
    # Live RViz visualization: draws the actual zones/angles _drive() uses
    # for its decisions (not just raw LaserScan dots), plus a text label
    # of the live decision (straight / veer left / veer right, and which
    # state) -- so what the robot is "thinking" is visible, not just
    # inferred from behavior or log lines. Everything is in the
    # base_footprint frame (robot center, x=forward, y=left per REP-103),
    # matching how nearest_in_side_zone/sector_min_range already reason
    # about these zones -- RViz transforms it into the fixed frame via TF.
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

    def _line_marker(
        self,
        marker_id: int,
        ns: str,
        p0: Tuple[float, float],
        p1: Tuple[float, float],
        rgba: Tuple[float, float, float, float],
        width: float = 0.02,
        z: float = 0.05,
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
        m.points = [Point(x=p0[0], y=p0[1], z=z), Point(x=p1[0], y=p1[1], z=z)]
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
            return f"PUTAR BALIK ({self._turn_direction})", RED
        if self.state == CoverageState.FINISHED:
            return "SELESAI", BLUE
        if self.state in (CoverageState.WAITING_MAP, CoverageState.WAITING_LOCALIZATION):
            return "MENUNGGU PETA/LOKALISASI", GREY

        # DRIVING: reflect whichever commit-hysteresis level actually
        # drove the wheels this tick. Obstacle-veer (self._veer_commit)
        # takes priority when active; _drive_straight_with_lane_correction()
        # only ever runs (and updates self._lane_commit) when the veer
        # layer itself says STRAIGHT, so falling back to it here means the
        # label always matches what _publish_motor() was actually just
        # called with -- no more "LURUS" on screen while the wheels are
        # actually nudging for lane-keeping.
        veer_level = self._veer_commit["level"]
        if veer_level != "STRAIGHT":
            return {
                "GENTLE_LEFT": ("BELOK KIRI (halus)", YELLOW),
                "SHARP_LEFT": ("BELOK KIRI (tajam)", RED),
                "GENTLE_RIGHT": ("BELOK KANAN (halus)", YELLOW),
                "SHARP_RIGHT": ("BELOK KANAN (tajam)", RED),
            }.get(veer_level, ("LURUS", GREEN))

        return {
            "STRAIGHT": ("LURUS", GREEN),
            "LANE_SOFT_LEFT": ("LURUS (koreksi lajur kiri)", YELLOW),
            "LANE_HARD_LEFT": ("LURUS (koreksi lajur kiri, kuat)", YELLOW),
            "LANE_SOFT_RIGHT": ("LURUS (koreksi lajur kanan)", YELLOW),
            "LANE_HARD_RIGHT": ("LURUS (koreksi lajur kanan, kuat)", YELLOW),
        }.get(self._lane_commit["level"], ("LURUS", GREEN))

    def _publish_zone_markers(self) -> None:
        half = self.front_cone_half_deg
        front_half = self.front_sector_deg

        markers = [
            # Robot body outline (yellow, matches robotmaganglidar.py's convention).
            self._arc_marker(0, "body", self.robot_half_width, 0.0, 360.0, (1.0, 1.0, 0.0, 0.9), width=0.015),
            # side_close_m: tight close-range safety net, full 180deg hemisphere.
            self._arc_marker(1, "side_close", self.side_close_m, -90.0, 90.0, (1.0, 0.5, 0.0, 0.5), width=0.015),
            # veer_circle_radius_m: farther anticipatory zone, only the
            # front_cone_half_deg cone actually used by nearest_in_side_zone.
            self._arc_marker(
                2, "veer_cone", self.veer_circle_radius_m, -half, half, (1.0, 1.0, 0.0, 0.6), width=0.02
            ),
            self._line_marker(
                3,
                "veer_cone",
                (0.0, 0.0),
                (self.veer_circle_radius_m * math.cos(math.radians(half)), self.veer_circle_radius_m * math.sin(math.radians(half))),
                (1.0, 1.0, 0.0, 0.6),
            ),
            self._line_marker(
                4,
                "veer_cone",
                (0.0, 0.0),
                (self.veer_circle_radius_m * math.cos(math.radians(-half)), self.veer_circle_radius_m * math.sin(math.radians(-half))),
                (1.0, 1.0, 0.0, 0.6),
            ),
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
