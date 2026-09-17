"""Bringup for the Nav2/FollowWaypoints coverage approach.

Separate from coverage_launch.py (the reactive driver) -- this brings the
full Nav2 stack back up (controller_server/planner_server/behavior_server/
bt_navigator/waypoint_follower) plus coverage_waypoint_node, which sends a
rectangular lawnmower path to Nav2's /follow_waypoints action.

Known risk: this is the same controller_server/DWB layer that drove
unpredictably (jerky, circled) on this robot before the reactive driver
replaced it for coverage_launch.py. cmd_vel_to_motor_bridge's wheel_base/
max_linear_speed calibration matters a lot more here than it did for the
reactive driver, which only ever used fixed proven byte values.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("robotpel")
    default_map = os.path.join(share_dir, "maps", "room.yaml")
    default_nav2_params = os.path.join(share_dir, "config", "nav2_params.yaml")
    default_waypoint_params = os.path.join(share_dir, "config", "waypoint_coverage_params.yaml")

    map_yaml = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("params_file")
    waypoint_params_file = LaunchConfiguration("waypoint_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")

    declare_map = DeclareLaunchArgument(
        "map", default_value=default_map, description="Full path to the saved map .yaml"
    )
    declare_nav2_params = DeclareLaunchArgument(
        "params_file", default_value=default_nav2_params, description="Nav2 parameter file"
    )
    declare_waypoint_params = DeclareLaunchArgument(
        "waypoint_params_file",
        default_value=default_waypoint_params,
        description="coverage_waypoint_node parameter file (swath_width, room_bounds)",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )
    declare_serial_port = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyUSB0", description="RPLidar serial port"
    )

    declare_exclude_radius = DeclareLaunchArgument(
        "exclude_radius_m", default_value="0.25",
        description="blank out /scan returns closer than this to the LiDAR (m) -- filters the robot's own laptop/cables",
    )
    declare_exclude_min = DeclareLaunchArgument("exclude_angle_min_deg", default_value="0.0", description="see coverage_launch.py")
    declare_exclude_max = DeclareLaunchArgument("exclude_angle_max_deg", default_value="0.0", description="see coverage_launch.py")
    exclude_radius_m = LaunchConfiguration("exclude_radius_m")
    exclude_angle_min_deg = LaunchConfiguration("exclude_angle_min_deg")
    exclude_angle_max_deg = LaunchConfiguration("exclude_angle_max_deg")

    declare_laser_frame = DeclareLaunchArgument("laser_frame", default_value="laser", description="frame_id published in /scan headers")
    declare_laser_x = DeclareLaunchArgument("laser_x", default_value="0.0", description="lidar offset x (m)")
    declare_laser_y = DeclareLaunchArgument("laser_y", default_value="0.0", description="lidar offset y (m)")
    declare_laser_z = DeclareLaunchArgument("laser_z", default_value="0.15", description="lidar offset z (m)")
    declare_laser_roll = DeclareLaunchArgument("laser_roll", default_value="0.0", description="lidar roll (rad)")
    declare_laser_pitch = DeclareLaunchArgument("laser_pitch", default_value="0.0", description="lidar pitch (rad)")
    declare_laser_yaw = DeclareLaunchArgument("laser_yaw", default_value="3.14159", description="lidar yaw (rad)")
    declare_laser_inverted = DeclareLaunchArgument("laser_inverted", default_value="false", description="true if the LiDAR is mounted upside-down")

    laser_frame = LaunchConfiguration("laser_frame")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_roll = LaunchConfiguration("laser_roll")
    laser_pitch = LaunchConfiguration("laser_pitch")
    laser_yaw = LaunchConfiguration("laser_yaw")
    laser_inverted = LaunchConfiguration("laser_inverted")

    lifecycle_nodes_localization = ["map_server", "amcl"]
    lifecycle_nodes_navigation = ["controller_server", "planner_server", "behavior_server", "bt_navigator", "waypoint_follower"]

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
                    "--x", laser_x, "--y", laser_y, "--z", laser_z,
                    "--roll", laser_roll, "--pitch", laser_pitch, "--yaw", laser_yaw,
                    "--frame-id", "base_footprint", "--child-frame-id", laser_frame,
                ],
            ),
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

            # --- Localization ---
            Node(
                package="nav2_map_server", executable="map_server", name="map_server",
                output="screen",
                parameters=[nav2_params_file, {"yaml_filename": map_yaml, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_amcl", executable="amcl", name="amcl",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
                output="screen",
                parameters=[{"autostart": True, "node_names": lifecycle_nodes_localization, "use_sim_time": use_sim_time}],
            ),

            # --- Full Nav2 stack ---
            Node(
                package="nav2_controller", executable="controller_server", name="controller_server",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_planner", executable="planner_server", name="planner_server",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_behaviors", executable="behavior_server", name="behavior_server",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{"autostart": True, "node_names": lifecycle_nodes_navigation, "use_sim_time": use_sim_time}],
            ),

            # --- Coverage path generator + hardware bridge ---
            Node(
                package="robotpel",
                executable="waypoint_coverage_node",
                name="coverage_waypoint_node",
                output="screen",
                parameters=[waypoint_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="robotpel",
                executable="cmd_vel_to_motor_bridge",
                name="cmd_vel_to_motor_bridge",
                output="screen",
                parameters=[waypoint_params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
