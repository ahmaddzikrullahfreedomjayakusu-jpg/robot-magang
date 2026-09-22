#!/usr/bin/env python3
"""Drive the robot through a path YOU mark by hand in RViz -- separate
project from robotpel's automatic coverage system (robotpel is untouched;
this is a standalone, additional option).

How to use it (see the matching launch file / start.sh for bringup):
  1. In RViz, click the "Publish Point" tool and click each point along
     the path you want the robot to drive, in order. Every click adds one
     waypoint -- a thick green corridor (robot_width wide) + blue spheres
     show the path building up live so you can see exactly what you've
     marked, and a yellow circle at the robot's live AMCL position shows
     where it actually is against that path.
  2. When you're done marking, click "2D Goal Pose" once anywhere (its
     actual position/orientation is ignored -- the click itself is just
     the "go" signal). The robot then drives the marked waypoints in
     order: turn to face the next point, drive straight to it, repeat.
  3. While marking is still in progress (before step 2), every new
     "Publish Point" click keeps extending the path. Once driving has
     started, new clicks are ignored until the run finishes or the node
     is restarted.

The marked path is auto-saved (to saved_waypoints_file) the moment you
click "2D Goal Pose", and auto-loaded back the next time the node starts
-- so most runs just need that one "2D Goal Pose" click, no re-marking.
Clicking "Publish Point" at least once after startup discards whatever
was loaded and starts a fresh path instead.

Driving itself deliberately reuses the exact same proven pieces as
robotpel's reactive coverage_planner_node -- the same tested motor byte
values, the same odometry-driven (not blind-timer) precise pivot, and the
same live-LiDAR front-wall safety stop -- rather than Nav2's
controller_server, which this whole project moved away from earlier for
being unpredictable on this robot. A user-marked path still deserves the
same safety net a wall gives the automatic system: if something's
actually in the way, this pauses instead of driving through it.
"""

import math
import os
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray
from visualization_msgs.msg import Marker

# Proven motor byte values -- copied as-is from robotmaganglidar1.py /
# robotpel's coverage_planner_node.py. Do not guess new ones.
MOTOR_NEUTRAL = 127
MOTOR_STEP = 30
MOTOR_FORWARD = MOTOR_NEUTRAL - MOTOR_STEP            # 97
MOTOR_REVERSE = MOTOR_NEUTRAL + MOTOR_STEP            # 157
MOTOR_FORWARD_SLOW = MOTOR_NEUTRAL - 20               # 107, gentle-turn slowed side
MOTOR_FORWARD_VERY_SLOW = MOTOR_NEUTRAL - 10          # 117, sharp-turn slowed side
MOTOR_REVERSE_SLOW = MOTOR_NEUTRAL + 15               # 142, slowed-side reverse-with-turn (same value robotmaganglidar1.py's own trapped-escape maneuver uses)
SPIN_LEFT = (MOTOR_REVERSE, MOTOR_FORWARD)    # (kiri, kanan) -- spin left/CCW
SPIN_RIGHT = (MOTOR_FORWARD, MOTOR_REVERSE)   # (kiri, kanan) -- spin right/CW
REVERSE_ARC_LEFT = (MOTOR_REVERSE, MOTOR_REVERSE_SLOW)   # (kiri, kanan) -- back off curving left
REVERSE_ARC_RIGHT = (MOTOR_REVERSE_SLOW, MOTOR_REVERSE)  # (kiri, kanan) -- back off curving right


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


def sector_min_range(scan: LaserScan, center_deg: float, half_width_deg: float) -> float:
    """Minimum valid range within a LiDAR-local angular window (degrees).
    Same helper as robotpel's coverage_planner_node.py."""
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
    """Nearest range on `side` ('left' or 'right') of the robot within
    `radius` -- same Cartesian (x,y) classification as robotpel's
    coverage_planner_node's final, bug-fixed version (the one that
    actually got left/right correct, after an angle-guessing version
    repeatedly came out backwards). Converts each ray to
    base_footprint-relative (x forward, y lateral, +y=left/-y=right per
    REP-103) using lidar_angle -> base_footprint_angle = lidar_angle +
    180deg (matches this robot's laser_yaw=pi mount), and only counts
    rays within `cone_half_deg` of dead-ahead so something out near the
    robot's flank -- which driving straight would never actually reach --
    doesn't trigger a swerve. `min_points` requires that many confirming
    rays (not just one stray reflection/speck) before counting as a real
    detection, same anti-noise reasoning as sector_wall_fraction.
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
    """Fraction of valid rays in the sector reading closer than within_m --
    tells a real wall (fills the whole sector) apart from a lone noise
    spike (trips one or two rays). Same helper as coverage_planner_node.py."""
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


def sector_valid_count(scan: LaserScan, center_deg: float, half_width_deg: float) -> int:
    """How many rays in the sector got any real return at all (within
    range_min/range_max). A glass wall/window often gives NO return for
    most of a sector -- light passes through instead of bouncing back --
    leaving just a couple of stray valid points (off a frame/edge/
    reflection). sector_wall_fraction alone can be fooled by that: a
    sector with only 2 valid rays that are both close reads as "100%
    wall", identical to a sector genuinely packed with wall. Requiring a
    minimum valid-ray count alongside the fraction catches that gap --
    this is what lets the front-wall pause tell "wall" apart from "mostly
    glass, one or two spurious reflections". Live testing repeatedly hit
    exactly this at one recurring spot: the robot pausing/resuming over
    and over for a minute at a time instead of ever completing the turn
    there, which just looked like "not turning" from the outside.
    """
    center = math.radians(center_deg)
    half = math.radians(half_width_deg)
    count = 0
    angle = scan.angle_min
    for r in scan.ranges:
        diff = abs(normalize_angle(angle - center))
        if diff <= half and scan.range_min <= r <= scan.range_max:
            count += 1
        angle += scan.angle_increment
    return count


class DriveState(Enum):
    WAITING_POINTS = auto()   # collecting clicked waypoints, not driving yet
    TURNING = auto()          # pivoting to face the next waypoint
    DRIVING = auto()          # driving straight toward it
    PAUSED_OBSTACLE = auto()  # something's in the way, holding until clear
    REVERSING = auto()        # genuinely stuck (no progress) -- backing off briefly
    FINISHED = auto()


@dataclass
class Waypoint:
    x: float
    y: float
    # True for a point that should be approached by fully stopping and
    # pivoting to face it (an original clicked point kept as a genuinely
    # sharp turn, or the very first/last point). False for a point
    # inserted by _smooth_path() to round a gentle bend into a curve --
    # reaching one of those should NOT stop the robot at all, just seamlessly
    # keep driving toward the next point; stopping at every one of those
    # (they can be a few cm apart) is exactly what made a smoothed curve
    # feel like stop-start stuttering instead of one continuous turn.
    hard_corner: bool = True


class ManualWaypointDriverNode(Node):
    """Collects hand-marked RViz waypoints, then drives them in order
    using the same proven motor bytes / pivot-by-odometry / LiDAR-safety
    approach as robotpel's reactive coverage_planner_node."""

    def __init__(self) -> None:
        super().__init__("manual_waypoint_driver_node")

        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("start_trigger_topic", "/goal_pose")
        self.declare_parameter("global_frame", "map")
        # Marking the same path by hand every single test run got old
        # fast. The raw clicked points (before smoothing) are saved here
        # every time driving starts ("2D Goal Pose"), and auto-loaded
        # back on startup -- so most runs just need that one click, no
        # re-marking. Clicking "Publish Point" at least once after
        # startup still starts a fresh path from scratch (the loaded one
        # is discarded on the first new click, not appended to).
        self.declare_parameter(
            "saved_waypoints_file",
            os.path.expanduser("~/Documents/Robot magang/robotpel_manual_waypoints/config/saved_waypoints.yaml"),
        )

        # How close counts as "reached" a waypoint.
        self.declare_parameter("waypoint_tolerance_m", 0.15)
        # Pivot precision + safety ceiling, mirrors coverage_planner_node's
        # TURN_PIVOT_TOLERANCE_RAD / TURN_PIVOT_TIMEOUT_SECONDS.
        self.declare_parameter("turn_pivot_tolerance_deg", 3.0)
        self.declare_parameter("turn_pivot_timeout_s", 5.0)
        # Per-waypoint drive safety ceiling (in case pose data stalls or a
        # waypoint is unreachable) -- not a normal stopping condition,
        # just a "don't spin forever" net.
        self.declare_parameter("drive_timeout_s", 30.0)
        # Genuine stall detection -- live testing found the robot can get
        # physically wedged somewhere (wheel slip, jammed against
        # something) while still being commanded to drive: motors keep
        # getting GENTLE/SHARP/SPIN bytes the whole time (never neutral),
        # but the AMCL position doesn't move at all for many seconds
        # straight, which just looks like the robot silently freezing in
        # RViz. This checks *actual displacement*, with a long window and
        # a tiny threshold so ordinary slow cornering never false-trips
        # it (only a real, near-total lack of movement does). Recovery is
        # a brief straight reverse -- not a skip, not a freeze -- so the
        # robot is always still doing something, per feedback that it
        # must always keep moving/correcting, never just sit still.
        self.declare_parameter("stall_check_period_s", 4.0)
        self.declare_parameter("stall_min_progress_m", 0.04)
        self.declare_parameter("reverse_escape_seconds", 1.2)

        # Right after the initial pivot, drive dead straight for a beat
        # before corridor correction kicks in -- feedback was that
        # starting to correct immediately (even a small GENTLE nudge from
        # residual pivot/cross-track error right at the start) felt like
        # the robot veering before it had even properly started moving.
        self.declare_parameter("start_straight_seconds", 2.0)

        # Live-LiDAR front safety stop -- same detection logic AND the
        # same tuned values as robotpel's coverage_planner_node
        # (nose_length_m + front_stop_m, wall_confirm_fraction,
        # front_sector_deg): that threshold went through a lot of live
        # re-tuning there (0.45 -> 0.55 -> 0.65m total), so this package
        # reuses the same final numbers instead of guessing its own.
        # Only the DRIVING behavior differs between the two projects
        # (waypoint-following here vs. straight+U-turn there) -- the
        # "is there really a wall in front of me" detection itself is
        # shared.
        self.declare_parameter("nose_length_m", 0.30)  # LiDAR to the robot's front tip
        self.declare_parameter("front_stop_m", 0.35)  # desired clearance beyond the nose tip, not from the LiDAR
        self.declare_parameter("front_sector_deg", 20.0)
        self.declare_parameter("wall_confirm_fraction", 0.6)
        # Live testing found a spot with a glass wall/window right at the
        # marked path: the LiDAR flickered between "nothing detected" and
        # "close (~0.5m)" tick to tick, and the few close returns alone
        # satisfied wall_confirm_fraction, pausing/resuming over and over
        # for a minute straight instead of driving through. This requires
        # a minimum number of actual valid returns in the sector too --
        # a couple of stray points off a window frame shouldn't carry the
        # same weight as a sector genuinely packed with real wall.
        self.declare_parameter("front_wall_min_valid_rays", 5)
        # Resuming used to re-check against the exact same threshold that
        # triggered the pause -- with a marked path that grazes close to a
        # wall/corner, the front reading sits right on that boundary and
        # flickers (noise, or the beam sweeping slightly), causing rapid
        # pause/resume chatter that stalls the robot in place instead of
        # ever reaching the real turn further down the path. Requiring a
        # bit more clearance to resume than what triggered the pause
        # breaks that chatter.
        self.declare_parameter("front_clear_margin_m", 0.15)

        # Robot body/path-corridor visualization -- same physical
        # dimensions as robotpel's coverage_planner_node (robot_width
        # 0.60m / robot_half_width 0.30m), kept as this package's own
        # parameters rather than importing from robotpel so the two
        # packages stay fully independent.
        self.declare_parameter("robot_half_width", 0.30)
        self.declare_parameter("robot_width", 0.80)  # was 0.60 -> 0.70 -> 0.80 -- widened per user request so the corridor gives more slack before correcting

        # --- Side-obstacle avoidance + stay-in-corridor correction ---
        # This exact kind of feature (side-veer avoidance + cross-track
        # lane correction) was tried in robotpel's coverage_planner_node
        # and caused real zigzag/oscillation until two things were added:
        # (1) proper Cartesian (x,y) left/right classification instead of
        # guessing a LiDAR-local angle range, and (2) commit-hysteresis
        # (once committed to a correction, hold it briefly before
        # relaxing) so borderline readings don't flip the decision every
        # 0.1s tick. Both are reused here as-is (_commit_level,
        # nearest_in_side_zone) -- same proven fix, not a fresh guess.
        self.declare_parameter("side_close_m", 0.35)  # tight, single-ray-sensitive safety net
        # Anticipatory GENTLE zone -- feedback was this made the robot too
        # sensitive in ordinary narrow corridors (both walls sit inside a
        # wide anticipatory radius just by the corridor being narrow, so
        # it kept nudging side to side there even with plenty of real
        # clearance). Narrowed so GENTLE only kicks in once actually
        # getting close, not just "somewhere in a narrow hallway";
        # veer_sharp_m/side_close_m (the real "body's about to touch the
        # wall" floor) are untouched.
        self.declare_parameter("side_avoid_radius_m", 0.45)
        self.declare_parameter("side_confirm_points", 2)  # far zone needs this many confirming rays, not just one
        self.declare_parameter("side_cone_half_deg", 60.0)  # far zone only looks within this cone of dead-ahead
        self.declare_parameter("veer_sharp_m", 0.38)  # inside this, either zone -> sharp (stop the inside wheel)
        self.declare_parameter("veer_commit_seconds", 0.4)  # hold a committed veer level this long before relaxing
        # Cross-track ("stay inside the green corridor") correction.
        # Bias term added to heading error, same formula
        # coverage_planner_node's lane-correction used.
        self.declare_parameter("cross_track_gain", 0.35)  # was 0.6 -> 0.4 -> 0.35 -- softer pull, jumping straight to a hard correction was overshooting past the corridor
        self.declare_parameter("lane_correct_deg", 18.0)  # was 8.0 -> 10.0 -> 18.0 -- much wider GENTLE-only zone, SHARP now reserved for genuinely large deviations instead of triggering early
        # SHARP only slows the inside wheel (still both wheels driving
        # forward) -- live testing found that's not enough turning
        # authority once the deviation is already huge (over a meter off
        # the line): the robot just cruises along failing to converge
        # until drive_timeout_s eventually gives up and skips ahead,
        # which looked like the robot "stopping" instead of correcting.
        # Past this much error, spin the inside wheel in REVERSE instead
        # (same SPIN_LEFT/SPIN_RIGHT bytes as the initial pivot) while the
        # outside wheel keeps driving forward -- a much tighter turn, and
        # still actively moving, never a dead stop.
        self.declare_parameter("spin_correct_deg", 45.0)

        # --- Path smoothing: round gentle bends into a curve, keep genuinely
        # sharp turns as hard corners -- applied once, right when driving
        # starts (on_start_trigger), replacing the raw clicked points with
        # a denser smoothed list. A quadratic-Bezier fillet at each corner
        # gentler than sharp_turn_threshold_deg, not a full spline: simple,
        # predictable, and leaves real sharp corners exactly where marked.
        self.declare_parameter("sharp_turn_threshold_deg", 60.0)  # turns sharper than this stay as a hard pivot corner
        self.declare_parameter("corner_round_fraction", 0.3)  # how far back from the corner (fraction of the shorter adjacent segment) rounding starts
        self.declare_parameter("corner_round_points", 6)  # samples along each rounded corner's curve

        self.amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.clicked_point_topic = str(self.get_parameter("clicked_point_topic").value)
        self.start_trigger_topic = str(self.get_parameter("start_trigger_topic").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.saved_waypoints_file = str(self.get_parameter("saved_waypoints_file").value)
        self.waypoint_tolerance_m = float(self.get_parameter("waypoint_tolerance_m").value)
        self.turn_pivot_tolerance_rad = math.radians(float(self.get_parameter("turn_pivot_tolerance_deg").value))
        self.turn_pivot_timeout_s = float(self.get_parameter("turn_pivot_timeout_s").value)
        self.drive_timeout_s = float(self.get_parameter("drive_timeout_s").value)
        self.stall_check_period_s = float(self.get_parameter("stall_check_period_s").value)
        self.stall_min_progress_m = float(self.get_parameter("stall_min_progress_m").value)
        self.reverse_escape_seconds = float(self.get_parameter("reverse_escape_seconds").value)
        self.start_straight_seconds = float(self.get_parameter("start_straight_seconds").value)
        self.nose_length_m = float(self.get_parameter("nose_length_m").value)
        self.front_stop_m = float(self.get_parameter("front_stop_m").value)
        # What the raw LiDAR range actually needs to read to keep
        # front_stop_m of clearance in front of the physical nose tip --
        # identical formula to coverage_planner_node's front_stop_from_lidar_m.
        self.front_stop_from_lidar_m = self.nose_length_m + self.front_stop_m
        self.front_sector_deg = float(self.get_parameter("front_sector_deg").value)
        self.wall_confirm_fraction = float(self.get_parameter("wall_confirm_fraction").value)
        self.front_wall_min_valid_rays = int(self.get_parameter("front_wall_min_valid_rays").value)
        self.front_clear_margin_m = float(self.get_parameter("front_clear_margin_m").value)
        self.robot_half_width = float(self.get_parameter("robot_half_width").value)
        self.robot_width = float(self.get_parameter("robot_width").value)
        self.side_close_m = float(self.get_parameter("side_close_m").value)
        self.side_avoid_radius_m = float(self.get_parameter("side_avoid_radius_m").value)
        self.side_confirm_points = int(self.get_parameter("side_confirm_points").value)
        self.side_cone_half_deg = float(self.get_parameter("side_cone_half_deg").value)
        self.veer_sharp_m = float(self.get_parameter("veer_sharp_m").value)
        self.veer_commit_seconds = float(self.get_parameter("veer_commit_seconds").value)
        self.cross_track_gain = float(self.get_parameter("cross_track_gain").value)
        self.lane_correct_deg = float(self.get_parameter("lane_correct_deg").value)
        self.spin_correct_deg = float(self.get_parameter("spin_correct_deg").value)
        self.sharp_turn_threshold_deg = float(self.get_parameter("sharp_turn_threshold_deg").value)
        self.corner_round_fraction = float(self.get_parameter("corner_round_fraction").value)
        self.corner_round_points = int(self.get_parameter("corner_round_points").value)

        self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose_topic, self.on_amcl_pose, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.create_subscription(PointStamped, self.clicked_point_topic, self.on_clicked_point, 10)
        self.create_subscription(PoseStamped, self.start_trigger_topic, self.on_start_trigger, 10)

        self.motor_pub = self.create_publisher(Int16MultiArray, "/motor_rpm", 10)
        self.path_viz_pub = self.create_publisher(Marker, "/manual_waypoints/path_marker", 10)
        self.robot_viz_pub = self.create_publisher(Marker, "/manual_waypoints/robot_marker", 10)

        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.latest_scan: Optional[LaserScan] = None
        self.waypoints: List[Waypoint] = []
        self.current_index = 0
        self.state = DriveState.WAITING_POINTS
        self._state_entered_at = time.monotonic()
        self._pivot_start_yaw: Optional[float] = None
        self._mission_start_xy: Optional[Tuple[float, float]] = None
        # Set once, the first time the end of the marked path is reached,
        # so the round trip happens exactly once: out to the last marked
        # point, then back to the start along the same corridor -- not a
        # repeating shuttle.
        self._returning = False
        # Commit-hysteresis state (see _commit_level()) -- one shared
        # state for the single merged obstacle+corridor decision.
        self._veer_commit = {"level": "STRAIGHT", "until": 0.0}
        self._last_progress_xy: Optional[Tuple[float, float]] = None
        self._last_progress_check_at = time.monotonic()
        self._start_straight_until: Optional[float] = None
        # Which way the next REVERSING escape should curve -- flips every
        # time reversing straight-back-arc'd one way doesn't free it, so
        # repeated attempts try both sides instead of grinding the same
        # direction against whatever it's wedged on.
        self._reverse_arc_left = True
        # True right after loading a saved path at startup, until the
        # first fresh "Publish Point" click either keeps it (by starting
        # to drive) or discards it (by marking a new path instead).
        self._loaded_from_file = False

        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.3, self._publish_path_marker)
        self.create_timer(0.2, self._publish_robot_marker)

        self._load_saved_waypoints()

        self.get_logger().info(
            "manual_waypoint_driver_node ready -- click 'Publish Point' in RViz to mark waypoints, "
            "then click '2D Goal Pose' once to start driving them in order."
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def on_clicked_point(self, msg: PointStamped) -> None:
        if self.state != DriveState.WAITING_POINTS:
            self.get_logger().warn(
                "Already driving (or finished) -- ignoring new clicked point. "
                "Restart the node to mark a fresh path.",
                throttle_duration_sec=2.0,
            )
            return
        if self._loaded_from_file:
            # First fresh click after a saved path was auto-loaded --
            # treat this as marking a brand new path, not appending to
            # the old one.
            self.get_logger().info("Marking a new path -- discarding the loaded one.")
            self.waypoints = []
            self._loaded_from_file = False
        self.waypoints.append(Waypoint(x=msg.point.x, y=msg.point.y))
        self.get_logger().info(
            f"Waypoint {len(self.waypoints)} marked: ({msg.point.x:.2f}, {msg.point.y:.2f})"
        )

    def on_start_trigger(self, msg: PoseStamped) -> None:
        # The "2D Goal Pose" click's own position/orientation is ignored
        # on purpose -- it's only ever used here as a "start now" signal,
        # not as an extra waypoint.
        if self.state != DriveState.WAITING_POINTS:
            return
        if not self.waypoints:
            self.get_logger().warn("Start triggered but no waypoints marked yet -- ignoring.")
            return
        self._save_waypoints(self.waypoints)
        if self.current_pose is not None:
            self._mission_start_xy = (self.current_pose[0], self.current_pose[1])
        raw_count = len(self.waypoints)
        self.waypoints = self._smooth_path(self.waypoints)
        self.current_index = 0
        self._returning = False
        self._start_straight_until = None
        self.get_logger().info(
            f"Starting: driving {raw_count} marked point(s), smoothed into {len(self.waypoints)} -- "
            "gentle bends rounded into a curve, sharp turns kept as-is."
        )
        self._enter(DriveState.TURNING)

    def _load_saved_waypoints(self) -> None:
        path = self.saved_waypoints_file
        if not os.path.isfile(path):
            return
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f) or {}
            points = data.get("waypoints", [])
            loaded = [Waypoint(x=float(p["x"]), y=float(p["y"])) for p in points]
        except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"Couldn't load saved waypoints from {path}: {exc}")
            return
        if not loaded:
            return
        self.waypoints = loaded
        self._loaded_from_file = True
        self.get_logger().info(
            f"Loaded {len(loaded)} saved waypoint(s) from {path} -- click '2D Goal Pose' to drive "
            "them as-is, or click 'Publish Point' to mark a new path instead."
        )

    def _save_waypoints(self, waypoints: List["Waypoint"]) -> None:
        path = self.saved_waypoints_file
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            data = {"waypoints": [{"x": w.x, "y": w.y} for w in waypoints]}
            with open(path, "w") as f:
                yaml.safe_dump(data, f, default_flow_style=False)
            self.get_logger().info(f"Saved {len(waypoints)} waypoint(s) to {path} for next time.")
        except OSError as exc:
            self.get_logger().warn(f"Couldn't save waypoints to {path}: {exc}")

    def _round_corner(
        self,
        smoothed: List[Waypoint],
        prev_xy: Tuple[float, float],
        this_wp: Waypoint,
        next_xy: Tuple[float, float],
    ) -> None:
        """Append this_wp to `smoothed`, rounded into a short
        quadratic-Bezier curve if the turn there is gentler than
        sharp_turn_threshold_deg (cutting back corner_round_fraction of
        the shorter adjacent segment on each side, sampling
        corner_round_points along the curve), else kept as a single hard
        point -- a genuinely sharp turn stays a real pivot, only the
        gentle bends get smoothed into something that drives and looks
        like a normal curved road instead of a boxy zig-zag of straight
        segments.
        """
        v_in = (this_wp.x - prev_xy[0], this_wp.y - prev_xy[1])
        v_out = (next_xy[0] - this_wp.x, next_xy[1] - this_wp.y)
        len_in = math.hypot(*v_in)
        len_out = math.hypot(*v_out)
        if len_in < 1e-6 or len_out < 1e-6:
            smoothed.append(this_wp)
            return

        angle_in = math.atan2(v_in[1], v_in[0])
        angle_out = math.atan2(v_out[1], v_out[0])
        turn_deg = abs(math.degrees(normalize_angle(angle_out - angle_in)))

        if turn_deg >= self.sharp_turn_threshold_deg:
            smoothed.append(this_wp)  # genuinely sharp -- keep as a hard corner
            return

        # Cut back from the corner along both adjacent segments, never
        # more than half of either (so rounding one corner can't eat
        # into whatever's happening at its neighbor).
        cut = min(self.corner_round_fraction, 0.5) * min(len_in, len_out)
        t_in = 1.0 - cut / len_in
        t_out = cut / len_out
        # Every point belonging to this rounded corner -- the two cut
        # points and every sampled curve point in between -- is
        # hard_corner=False: reaching any of them should never stop the
        # robot, just keep it flowing through the curve.
        a = Waypoint(x=prev_xy[0] + v_in[0] * t_in, y=prev_xy[1] + v_in[1] * t_in, hard_corner=False)
        b = Waypoint(x=this_wp.x + v_out[0] * t_out, y=this_wp.y + v_out[1] * t_out, hard_corner=False)

        smoothed.append(a)
        steps = max(2, self.corner_round_points)
        for j in range(1, steps):
            s = j / steps
            bx = (1 - s) ** 2 * a.x + 2 * (1 - s) * s * this_wp.x + s ** 2 * b.x
            by = (1 - s) ** 2 * a.y + 2 * (1 - s) * s * this_wp.y + s ** 2 * b.y
            smoothed.append(Waypoint(x=bx, y=by, hard_corner=False))
        smoothed.append(b)

    def _smooth_path(self, waypoints: List[Waypoint]) -> List[Waypoint]:
        """Round each corner gentler than sharp_turn_threshold_deg into a
        curve (see _round_corner), including the corner at the very
        first clicked point (measured against the robot's actual
        starting position) -- that one used to always stay a raw,
        unrounded point no matter how sharp, which was easy to miss since
        it only matters going INTO the first leg. But the return trip
        retraces this same smoothed list in reverse, which turns that
        exact corner into the LAST turn of the whole mission -- live
        testing found the robot failing to actually complete it there
        ("di tikungan terakhir malah lurus", not enough correction
        authority applied in time for an unrounded sharp-ish corner right
        before the final stop). Rounding it the same as every other
        corner fixes that without touching any driving/correction logic.
        """
        if len(waypoints) < 3:
            return list(waypoints)

        smoothed: List[Waypoint] = []
        if self._mission_start_xy is not None:
            self._round_corner(smoothed, self._mission_start_xy, waypoints[0], (waypoints[1].x, waypoints[1].y))
        else:
            smoothed.append(waypoints[0])

        for i in range(1, len(waypoints) - 1):
            prev_wp = waypoints[i - 1]
            this_wp = waypoints[i]
            next_wp = waypoints[i + 1]
            self._round_corner(smoothed, (prev_wp.x, prev_wp.y), this_wp, (next_wp.x, next_wp.y))

        smoothed.append(waypoints[-1])
        return smoothed

    def _segment_start_xy(self) -> Tuple[float, float]:
        """Start point of the corridor line for the CURRENT target
        waypoint: the previous waypoint, or the robot's pose when the
        mission started if this is the first one."""
        if self.current_index == 0:
            if self._mission_start_xy is not None:
                return self._mission_start_xy
            return (self.current_pose[0], self.current_pose[1])
        prev = self.waypoints[self.current_index - 1]
        return (prev.x, prev.y)

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _enter(self, state: DriveState) -> None:
        self.state = state
        self._state_entered_at = time.monotonic()
        if state == DriveState.TURNING and self.current_pose is not None:
            self._pivot_start_yaw = self.current_pose[2]
        if state == DriveState.DRIVING:
            # Fresh leg after a real pivot (hard corner) -- a stale
            # commit from before that turn shouldn't carry over. Smoothed
            # curve points never re-enter DRIVING (see control_loop), so
            # this does NOT reset mid-curve -- only after an actual stop.
            self._veer_commit = {"level": "STRAIGHT", "until": 0.0}
            # Fresh stall-detection checkpoint every time real driving
            # (re-)starts -- an obstacle pause/reverse-escape stopping
            # briefly for a legitimate reason shouldn't count against it.
            if self.current_pose is not None:
                self._last_progress_xy = (self.current_pose[0], self.current_pose[1])
            self._last_progress_check_at = time.monotonic()
            # Arm the "go straight first" window exactly once per mission
            # -- only the very first leg (still targeting the first
            # waypoint, outbound) gets it.
            if self.current_index == 0 and not self._returning and self._start_straight_until is None:
                self._start_straight_until = time.monotonic() + self.start_straight_seconds

    def _commit_level(self, commit_state: dict, desired_level: str, rank: dict, commit_seconds: float) -> str:
        """Shared commit-hysteresis: hold a committed level on its side
        for `commit_seconds` before allowing a de-escalation back toward
        STRAIGHT/a lower rank, so borderline sensor/heading readings
        don't make the decision flap every control tick. Identical logic
        to the one that fixed the exact same oscillation problem in
        robotpel's coverage_planner_node. `commit_state` is a small
        {"level", "until"} dict; this node only has one (self._veer_commit,
        the single merged obstacle+corridor decision), but the helper
        stays generic in case a caller ever needs a second, independent one.
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

    def _publish_motor(self, kiri: int, kanan: int) -> None:
        msg = Int16MultiArray()
        msg.data = [int(kiri), int(kanan)]
        self.motor_pub.publish(msg)

    def control_loop(self) -> None:
        if self.state in (DriveState.WAITING_POINTS, DriveState.FINISHED):
            return
        if self.current_pose is None:
            self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
            return

        # A smoothed curve can pack points a few cm apart -- advance
        # through as many as are already within tolerance in one control
        # tick (bounded, so a pose glitch can't skip the whole path) so a
        # slightly-too-coarse tolerance can't stall progress along a
        # dense curve either.
        for _ in range(10):
            target = self.waypoints[self.current_index]
            x, y, yaw = self.current_pose
            dx = target.x - x
            dy = target.y - y
            dist = math.hypot(dx, dy)

            if dist > self.waypoint_tolerance_m:
                break

            reached_hard_corner = target.hard_corner
            self.get_logger().info(
                f"Reached point {self.current_index + 1}/{len(self.waypoints)}"
                + (" (corner)" if reached_hard_corner else ""),
                throttle_duration_sec=0.5,
            )
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                if not self._returning and self._mission_start_xy is not None and len(self.waypoints) > 1:
                    # End of the outward leg -- turn around and retrace
                    # the exact same corridor back to where driving
                    # started, once (self._returning stops this from
                    # repeating into an endless shuttle).
                    final_xy = (target.x, target.y)
                    return_path = list(reversed(self.waypoints[:-1]))
                    return_path.append(
                        Waypoint(x=self._mission_start_xy[0], y=self._mission_start_xy[1], hard_corner=True)
                    )
                    self.waypoints = return_path
                    self.current_index = 0
                    self._mission_start_xy = final_xy
                    self._returning = True
                    self._publish_path_marker()
                    self.get_logger().info(
                        "End of the marked path reached -- turning around and retracing "
                        "the same corridor back to the start."
                    )
                    self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
                    self._enter(DriveState.TURNING)
                    return
                self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
                self._enter(DriveState.FINISHED)
                self.get_logger().info("All marked waypoints reached -- done.")
                return

            # Never stop mid-route, not even at a real sharp corner --
            # live testing showed stopping to pivot in place at every
            # sharp bend along the way looked like the robot randomly
            # freezing. Only the initial pivot (on_start_trigger, before
            # any driving starts) and the final point (end-of-path
            # turn-around above) still stop; every other point -- hard
            # corner or smoothing curve alike -- just keeps flowing
            # toward the next one, relying on corridor correction
            # (including the SPIN tier for genuinely sharp turns) to
            # actually make the turn while still moving. Re-loop to
            # re-check distance against the new target instead of waiting
            # a full 0.1s tick.

        if self.state == DriveState.TURNING:
            self._do_turning(dx, dy)
        elif self.state == DriveState.DRIVING:
            self._do_driving(dx, dy)
        elif self.state == DriveState.PAUSED_OBSTACLE:
            self._do_paused_obstacle()
        elif self.state == DriveState.REVERSING:
            self._do_reversing()

    def _do_turning(self, dx: float, dy: float) -> None:
        """Pivot in place to face the next waypoint -- odometry decides
        when the target bearing is reached (same approach as
        coverage_planner_node's _pivot_done: a fixed timer can't reliably
        hit a precise angle), with a time ceiling as a safety net.
        """
        elapsed = time.monotonic() - self._state_entered_at
        target_yaw = math.atan2(dy, dx)
        _, _, yaw = self.current_pose
        heading_error = normalize_angle(target_yaw - yaw)

        if abs(heading_error) <= self.turn_pivot_tolerance_rad or elapsed >= self.turn_pivot_timeout_s:
            self._enter(DriveState.DRIVING)
            return

        kiri, kanan = SPIN_LEFT if heading_error > 0 else SPIN_RIGHT
        self._publish_motor(kiri, kanan)

    def _do_driving(self, dx: float, dy: float) -> None:
        """Drive toward the current target with exactly ONE unified
        decision -- STRAIGHT / GENTLE left-right / SHARP left-right, the
        same three motor-byte levels robotmaganglidar1.py itself uses
        (97 straight, 107 gentle-slowed-side, 117 sharp-slowed-side) --
        not two separate systems (obstacle-veer and corridor-correction)
        each independently deciding and committing. Two overlapping
        commit-hysteresis states could still hand off roughly between
        each other and look jerky; merging them into one desired level,
        picked by whichever demands the stronger correction, resolved by
        ONE shared commit-hysteresis, is both simpler and steadier.

        Priority: each control tick, weigh (1) a close side obstacle
        against (2) drifting off the green corridor line, and act on
        whichever is more severe -- THEN (3) only pause for a wall dead
        ahead if the resulting decision is still STRAIGHT. A wall
        straight ahead only means "pause" if the path itself wants to go
        STRAIGHT into it -- genuinely unexpected. If the marked path
        already calls for a turn here (real objects right next to a
        marked turn are common -- that's often *why* the path turns
        there), trust that turn and keep driving instead of pausing: live
        testing found the pause firing and blocking the turn from ever
        completing, which looked like the robot just refusing to turn.
        """
        elapsed = time.monotonic() - self._state_entered_at
        if elapsed >= self.drive_timeout_s:
            self.get_logger().warn(
                f"Drive timeout ({self.drive_timeout_s:.0f}s) reaching waypoint "
                f"{self.current_index + 1} -- skipping to the next one.",
            )
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self._enter(DriveState.FINISHED)
            else:
                # Give up on this one and keep going toward the next --
                # no mid-route stop-and-pivot here either, same reasoning
                # as control_loop's waypoint-reached handling.
                self._enter(DriveState.DRIVING)
            return

        front_wall_confirmed = False
        front = float("inf")
        if self.latest_scan is not None:
            front = sector_min_range(self.latest_scan, 180.0, self.front_sector_deg)
            if front < self.front_stop_from_lidar_m:
                fraction = sector_wall_fraction(
                    self.latest_scan, 180.0, self.front_sector_deg, self.front_stop_from_lidar_m
                )
                valid_count = sector_valid_count(self.latest_scan, 180.0, self.front_sector_deg)
                front_wall_confirmed = fraction >= self.wall_confirm_fraction and valid_count >= self.front_wall_min_valid_rays

        x, y, yaw = self.current_pose

        # Genuine stall check -- a long window with a tiny threshold, so
        # normal driving (even a slow correction crawl) never trips this;
        # only near-total lack of movement over several seconds does.
        if self._last_progress_xy is None:
            self._last_progress_xy = (x, y)
            self._last_progress_check_at = time.monotonic()
        elif time.monotonic() - self._last_progress_check_at >= self.stall_check_period_s:
            moved = math.hypot(x - self._last_progress_xy[0], y - self._last_progress_xy[1])
            if moved < self.stall_min_progress_m:
                self.get_logger().warn(
                    f"Stuck -- only moved {moved:.2f}m in the last {self.stall_check_period_s:.0f}s "
                    "despite driving -- backing off briefly.",
                )
                self._enter(DriveState.REVERSING)
                return
            self._last_progress_xy = (x, y)
            self._last_progress_check_at = time.monotonic()

        rank = {
            "STRAIGHT": 0,
            "GENTLE_LEFT": 1, "GENTLE_RIGHT": 1,
            "SHARP_LEFT": 2, "SHARP_RIGHT": 2,
            "SPIN_LEFT": 3, "SPIN_RIGHT": 3,
        }

        # --- (2) side-obstacle desired level ---
        obstacle_level = "STRAIGHT"
        side_right = side_left = diag_right = diag_left = float("inf")
        if self.latest_scan is not None:
            side_right = nearest_in_side_zone(self.latest_scan, "right", self.side_close_m)
            side_left = nearest_in_side_zone(self.latest_scan, "left", self.side_close_m)
            diag_right = nearest_in_side_zone(
                self.latest_scan, "right", self.side_avoid_radius_m, self.side_confirm_points, self.side_cone_half_deg
            )
            diag_left = nearest_in_side_zone(
                self.latest_scan, "left", self.side_avoid_radius_m, self.side_confirm_points, self.side_cone_half_deg
            )
            if side_right < self.side_close_m or diag_right < self.veer_sharp_m:
                obstacle_level = "SHARP_LEFT"
            elif diag_right < self.side_avoid_radius_m:
                obstacle_level = "GENTLE_LEFT"
            elif side_left < self.side_close_m or diag_left < self.veer_sharp_m:
                obstacle_level = "SHARP_RIGHT"
            elif diag_left < self.side_avoid_radius_m:
                obstacle_level = "GENTLE_RIGHT"

        # --- (3) corridor cross-track desired level ---
        sx, sy = self._segment_start_xy()
        seg_dx = (x + dx) - sx  # target.x - sx
        seg_dy = (y + dy) - sy  # target.y - sy
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 1e-3:
            target_yaw = math.atan2(dy, dx)
            cross_track = 0.0
        else:
            target_yaw = math.atan2(seg_dy, seg_dx)
            # +cross_track = robot is to the LEFT of the line (REP-103).
            cross_track = (seg_dx * (y - sy) - seg_dy * (x - sx)) / seg_len
        # yaw > target_yaw means rotated CCW/left of the line -> need to
        # turn RIGHT to come back (established, tested wheel convention).
        heading_error = normalize_angle(yaw - target_yaw) + normalize_angle(self.cross_track_gain * cross_track)
        deg = math.degrees(heading_error)
        if abs(deg) < self.lane_correct_deg * 0.3:
            corridor_level = "STRAIGHT"
        elif deg > 0:
            if deg > self.spin_correct_deg:
                corridor_level = "SPIN_RIGHT"
            else:
                corridor_level = "SHARP_RIGHT" if deg > self.lane_correct_deg else "GENTLE_RIGHT"
        else:
            if -deg > self.spin_correct_deg:
                corridor_level = "SPIN_LEFT"
            else:
                corridor_level = "SHARP_LEFT" if -deg > self.lane_correct_deg else "GENTLE_LEFT"

        # Straight-only grace window right at mission start (see _enter)
        # -- don't let residual pivot/cross-track error nudge a
        # correction in before the robot's even properly moving yet.
        if self._start_straight_until is not None and time.monotonic() < self._start_straight_until:
            corridor_level = "STRAIGHT"

        # Direct cross-track-distance escalation -- the combined
        # heading_error above dilutes a large cross-track by
        # cross_track_gain (0.35), so if the robot's yaw happens to be
        # roughly parallel to the line (a common case when it's just
        # drifted sideways, not mis-angled), cross-track alone could
        # never reach lane_correct_deg/spin_correct_deg no matter how far
        # off it actually drifted. Live testing found the robot frozen at
        # cross_track=0.55m -- already past the edge of the 0.80m-wide
        # drawn corridor -- stuck offering only GENTLE forever. Escalate
        # directly off distance too, tied to the real corridor width so
        # "past the green edge" and "a full lane-width past" mean something
        # concrete regardless of what the heading-based check alone says.
        half_corridor_m = self.robot_width / 2.0
        if abs(cross_track) >= self.robot_width:
            distance_level = "SPIN_RIGHT" if cross_track > 0 else "SPIN_LEFT"
        elif abs(cross_track) >= half_corridor_m:
            distance_level = "SHARP_RIGHT" if cross_track > 0 else "SHARP_LEFT"
        else:
            distance_level = "STRAIGHT"
        if rank[distance_level] > rank[corridor_level]:
            corridor_level = distance_level

        # --- merge: whichever demands the stronger correction wins; ties
        # go to the obstacle (safety over precision) ---
        desired_level = obstacle_level if rank[obstacle_level] >= rank[corridor_level] else corridor_level
        driven_level = self._commit_level(self._veer_commit, desired_level, rank, self.veer_commit_seconds)

        if front_wall_confirmed and driven_level == "STRAIGHT":
            self.get_logger().warn(
                f"Obstacle ahead (front={front:.2f}m) on the way to waypoint "
                f"{self.current_index + 1} -- pausing.",
                throttle_duration_sec=1.0,
            )
            self._enter(DriveState.PAUSED_OBSTACLE)
            return

        # robotmaganglidar1.py's 3-level convention (straight=97 both
        # sides, gentle slows the inside wheel to 107, sharp to 117) plus
        # one level above SHARP for genuinely large deviations: SPIN
        # reverses the inside wheel (same bytes as the initial pivot)
        # while the outside wheel keeps driving forward -- still actively
        # moving, never a dead stop, just a much tighter turn than SHARP
        # (which only slows a wheel, never reverses it) can manage.
        level_bytes = {
            "SHARP_LEFT": (MOTOR_FORWARD_VERY_SLOW, MOTOR_FORWARD),
            "GENTLE_LEFT": (MOTOR_FORWARD_SLOW, MOTOR_FORWARD),
            "SHARP_RIGHT": (MOTOR_FORWARD, MOTOR_FORWARD_VERY_SLOW),
            "GENTLE_RIGHT": (MOTOR_FORWARD, MOTOR_FORWARD_SLOW),
            "SPIN_LEFT": SPIN_LEFT,
            "SPIN_RIGHT": SPIN_RIGHT,
        }
        kiri, kanan = level_bytes.get(driven_level, (MOTOR_FORWARD, MOTOR_FORWARD))
        self.get_logger().info(
            f"{driven_level.replace('_', ' ')} (obstacle={obstacle_level}, corridor={corridor_level}, "
            f"cross_track={cross_track:.2f}m) -> kiri={kiri} kanan={kanan}",
            throttle_duration_sec=1.0,
        )
        self._publish_motor(kiri, kanan)

    def _do_paused_obstacle(self) -> None:
        self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
        if self.latest_scan is None:
            return
        front = sector_min_range(self.latest_scan, 180.0, self.front_sector_deg)
        if front >= self.front_stop_from_lidar_m + self.front_clear_margin_m:
            self.get_logger().info("Path clear again -- resuming.")
            self._enter(DriveState.DRIVING)

    def _do_reversing(self) -> None:
        """Genuinely stuck (see the stall check in _do_driving) -- back
        off for reverse_escape_seconds, then resume driving with a fresh
        stall checkpoint. Not a stop-and-wait: the robot is always
        actively doing something, just backward for a moment.

        Backs off in a curve (one wheel full reverse, the other a slower
        reverse -- same idea as robotmaganglidar1.py's own trapped-escape
        maneuver), not straight back: live testing found a spot where
        reversing dead straight repeatedly failed to free the robot (many
        stall->reverse->stuck-again cycles in a row, zero net movement
        each time) -- backing straight out of a wedge only works if
        whatever it's caught on is directly behind, which isn't always
        true. Alternates which side it curves toward on each new
        REVERSING entry, so repeated attempts try both directions.
        """
        elapsed = time.monotonic() - self._state_entered_at
        if elapsed < self.reverse_escape_seconds:
            kiri, kanan = REVERSE_ARC_LEFT if self._reverse_arc_left else REVERSE_ARC_RIGHT
            self._publish_motor(kiri, kanan)
        else:
            self._reverse_arc_left = not self._reverse_arc_left
            self._enter(DriveState.DRIVING)

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------

    def _publish_path_marker(self) -> None:
        if not self.waypoints:
            return
        line = Marker()
        line.header.frame_id = self.global_frame
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = "manual_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        # Thick GREEN corridor, robot_width wide -- shows the lane the
        # robot is meant to stay inside while driving between waypoints,
        # not just a thin reference line.
        line.scale.x = self.robot_width
        line.color.r, line.color.g, line.color.b, line.color.a = (0.0, 0.85, 0.15, 0.45)
        line.pose.orientation.w = 1.0
        line.points = [Point(x=w.x, y=w.y, z=0.01) for w in self.waypoints]
        self.path_viz_pub.publish(line)

        points = Marker()
        points.header.frame_id = self.global_frame
        points.header.stamp = line.header.stamp
        points.ns = "manual_path"
        points.id = 1
        points.type = Marker.SPHERE_LIST
        points.action = Marker.ADD
        points.scale.x = points.scale.y = points.scale.z = 0.10
        points.color.r, points.color.g, points.color.b, points.color.a = (0.0, 0.6, 1.0, 0.95)
        points.pose.orientation.w = 1.0
        points.points = list(line.points)
        self.path_viz_pub.publish(points)

    def _publish_robot_marker(self) -> None:
        """Yellow circle at the robot's live AMCL position, radius
        robot_half_width -- same idea as robotpel's coverage_planner_node
        RobotBody marker, so the robot's actual footprint/position is
        visible in RViz against the marked path, not just inferred. Plus a
        dark arrow on top of it pointing along the robot's actual heading
        (yaw) -- without this, SHARP/GENTLE LEFT-RIGHT in the logs is hard
        to eyeball against the RViz view since the plain circle has no
        visible front; the arrow makes "which way is the robot's front
        pointing right now" immediate, so a correction direction can be
        checked at a glance while tuning live.
        """
        if self.current_pose is None:
            return
        x, y, yaw = self.current_pose

        body = Marker()
        body.header.frame_id = self.global_frame
        body.header.stamp = self.get_clock().now().to_msg()
        body.ns = "robot_body"
        body.id = 0
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x = x
        body.pose.position.y = y
        body.pose.position.z = 0.05
        body.pose.orientation.w = 1.0
        body.scale.x = self.robot_half_width * 2.0
        body.scale.y = self.robot_half_width * 2.0
        body.scale.z = 0.02
        body.color.r, body.color.g, body.color.b, body.color.a = (1.0, 1.0, 0.0, 0.9)
        self.robot_viz_pub.publish(body)

        heading = Marker()
        heading.header.frame_id = self.global_frame
        heading.header.stamp = body.header.stamp
        heading.ns = "robot_heading"
        heading.id = 0
        heading.type = Marker.ARROW
        heading.action = Marker.ADD
        heading.pose.position.x = x
        heading.pose.position.y = y
        heading.pose.position.z = 0.08
        heading.pose.orientation.z = math.sin(yaw / 2.0)
        heading.pose.orientation.w = math.cos(yaw / 2.0)
        heading.scale.x = self.robot_half_width * 2.2  # arrow length, pokes out past the body circle
        heading.scale.y = 0.06  # shaft width
        heading.scale.z = 0.08  # head width
        heading.color.r, heading.color.g, heading.color.b, heading.color.a = (0.1, 0.1, 0.1, 0.95)
        self.robot_viz_pub.publish(heading)


def main() -> None:
    rclpy.init()
    node = ManualWaypointDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
