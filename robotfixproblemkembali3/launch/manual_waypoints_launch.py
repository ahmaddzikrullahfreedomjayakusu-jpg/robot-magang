"""Bringup for manual-waypoint driving: RPLidar + TF + AMCL (against a
saved map, same as robotpel's coverage_launch.py) + manual_waypoint_driver_node.

Reuses robotpel's scan_blind_spot_filter executable (package="robotpel")
instead of duplicating it here -- that filter (blanks out the laptop/
wheels seen at close range) is shared, proven code; this launch file only
adds the new manual-waypoint-driving piece on top of it, robotpel itself
is never modified.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    own_share = get_package_share_directory("robotfixproblemkembali3")
    robotpel_share = get_package_share_directory("robotpel")
    default_map = os.path.join(robotpel_share, "maps", "room.yaml")
    default_nav2_params = os.path.join(robotpel_share, "config", "nav2_params.yaml")
    default_waypoint_params = os.path.join(own_share, "config", "manual_waypoints_params.yaml")

    map_yaml = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("params_file")
    waypoint_params_file = LaunchConfiguration("waypoint_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_map = DeclareLaunchArgument(
        "map", default_value=default_map, description="Full path to the saved map .yaml (same map as robotpel)"
    )
    declare_nav2_params = DeclareLaunchArgument(
        "params_file", default_value=default_nav2_params, description="AMCL/map_server parameter file (reuses robotpel's)"
    )
    declare_waypoint_params = DeclareLaunchArgument(
        "waypoint_params_file",
        default_value=default_waypoint_params,
        description="manual_waypoint_driver_node parameter file",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )
    declare_serial_port = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyUSB0", description="RPLidar serial port"
    )
    serial_port = LaunchConfiguration("serial_port")

    declare_exclude_radius = DeclareLaunchArgument(
        "exclude_radius_m", default_value="0.35",
        description="blank out /scan returns closer than this to the LiDAR (m) -- filters the robot's own laptop/wheels",
    )
    declare_exclude_min = DeclareLaunchArgument(
        "exclude_angle_min_deg", default_value="0.0",
        description="optional extra fixed blind-spot sector to blank out (deg, LiDAR-local angle); min==max disables it",
    )
    declare_exclude_max = DeclareLaunchArgument(
        "exclude_angle_max_deg", default_value="0.0",
        description="see exclude_angle_min_deg",
    )
    exclude_radius_m = LaunchConfiguration("exclude_radius_m")
    exclude_angle_min_deg = LaunchConfiguration("exclude_angle_min_deg")
    exclude_angle_max_deg = LaunchConfiguration("exclude_angle_max_deg")

    # Same laser mount description as robotpel's coverage_launch.py /
    # mapping_launch.py -- keep all in sync, they describe the same
    # physical mount.
    declare_laser_frame = DeclareLaunchArgument(
        "laser_frame", default_value="laser", description="frame_id published in /scan headers"
    )
    declare_laser_x = DeclareLaunchArgument("laser_x", default_value="0.0", description="lidar offset x (m)")
    declare_laser_y = DeclareLaunchArgument("laser_y", default_value="0.0", description="lidar offset y (m)")
    declare_laser_z = DeclareLaunchArgument("laser_z", default_value="0.15", description="lidar offset z (m)")
    declare_laser_roll = DeclareLaunchArgument("laser_roll", default_value="0.0", description="lidar roll (rad)")
    declare_laser_pitch = DeclareLaunchArgument("laser_pitch", default_value="0.0", description="lidar pitch (rad)")
    declare_laser_yaw = DeclareLaunchArgument("laser_yaw", default_value="3.14159", description="lidar yaw (rad)")
    declare_laser_inverted = DeclareLaunchArgument(
        "laser_inverted", default_value="false", description="true if the LiDAR is mounted upside-down"
    )

    laser_frame = LaunchConfiguration("laser_frame")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_roll = LaunchConfiguration("laser_roll")
    laser_pitch = LaunchConfiguration("laser_pitch")
    laser_yaw = LaunchConfiguration("laser_yaw")
    laser_inverted = LaunchConfiguration("laser_inverted")

    lifecycle_nodes_localization = ["map_server", "amcl"]

    return LaunchDescription(
        [
            declare_map,
            declare_nav2_params,
            declare_waypoint_params,
            declare_use_sim_time,
            declare_serial_port,
            declare_exclude_radius,
            declare_exclude_min,
            declare_exclude_max,
            declare_laser_frame,
            declare_laser_x,
            declare_laser_y,
            declare_laser_z,
            declare_laser_roll,
            declare_laser_pitch,
            declare_laser_yaw,
            declare_laser_inverted,

            Node(
                name="rplidar_composition",
                package="rplidar_ros",
                executable="rplidar_composition",
                output="screen",
                parameters=[{
                    "serial_port": serial_port,
                    "serial_baudrate": 115200,
                    "frame_id": laser_frame,
                    "inverted": laser_inverted,
                    "angle_compensate": True,
                }],
            ),

            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint_to_laser",
                output="screen",
                arguments=[
                    "--x", laser_x,
                    "--y", laser_y,
                    "--z", laser_z,
                    "--roll", laser_roll,
                    "--pitch", laser_pitch,
                    "--yaw", laser_yaw,
                    "--frame-id", "base_footprint",
                    "--child-frame-id", laser_frame,
                ],
            ),

            # Reuses robotpel's own filter node -- not duplicated here.
            Node(
                package="robotpel",
                executable="scan_blind_spot_filter",
                name="scan_blind_spot_filter",
                output="screen",
                parameters=[{
                    "input_topic": "/scan",
                    "output_topic": "/scan_filtered",
                    "exclude_radius_m": exclude_radius_m,
                    "exclude_angle_min_deg": exclude_angle_min_deg,
                    "exclude_angle_max_deg": exclude_angle_max_deg,
                }],
            ),

            # --- Localization: map + AMCL (identical setup to robotpel's coverage_launch.py) ---
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[nav2_params_file, {"yaml_filename": map_yaml, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                # nav2_params_file (robotpel's own, reused as-is -- never
                # edited here) has update_min_a/update_min_d at 0.2
                # (~11.5deg / 20cm): AMCL only re-corrects map->odom after
                # that much motion since its last update. During a fast
                # in-place pivot (this package does one right at the start
                # of every drive toward a new waypoint) that's coarse
                # enough to visibly lag -- the LiDAR points RViz draws use
                # a map->odom offset from before the spin, dragging behind
                # the true wall position until AMCL catches up. Overriding
                # both tighter here (not editing robotpel's file) fixes
                # that lag without touching the shared source file.
                parameters=[
                    nav2_params_file,
                    {"use_sim_time": use_sim_time, "update_min_a": 0.05, "update_min_d": 0.05},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[{"autostart": True, "node_names": lifecycle_nodes_localization, "use_sim_time": use_sim_time}],
            ),

            # --- Manual waypoint driver ---
            Node(
                package="robotfixproblemkembali3",
                executable="manual_waypoint_driver_node",
                name="manual_waypoint_driver_node",
                output="screen",
                parameters=[waypoint_params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
