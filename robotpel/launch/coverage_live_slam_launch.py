"""Coverage bringup that skips the saved map entirely: slam_toolbox builds
/map live as the robot drives (exactly like mapping_launch.py), and
coverage_planner_node reads its pose straight off the map->base_footprint
TF slam_toolbox publishes instead of AMCL's /amcl_pose -- no map_server,
no AMCL, no "2D Pose Estimate" click needed, and the map is never saved to
disk (slam_toolbox only persists a map if something explicitly calls its
save_map service, which nothing here does).

Use coverage_launch.py instead if you want to reuse a previously saved
map (faster to start, no localization "warm-up" while slam_toolbox builds
its first few scans of map from scratch).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robotpel_share = get_package_share_directory("robotpel")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")
    slam_params_default = os.path.join(robotpel_share, "config", "slam_toolbox_params.yaml")
    default_coverage_params = os.path.join(robotpel_share, "config", "coverage_params.yaml")

    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file", default_value=slam_params_default, description="slam_toolbox parameter file"
    )
    declare_coverage_params = DeclareLaunchArgument(
        "coverage_params_file",
        default_value=default_coverage_params,
        description="coverage_planner_node parameter file",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )
    declare_advisory_only = DeclareLaunchArgument(
        "advisory_only",
        default_value="false",
        description="true = compute/show the veer decision (RViz label) but never publish /motor_rpm -- for pushing the robot by hand",
    )

    slam_params_file = LaunchConfiguration("slam_params_file")
    coverage_params_file = LaunchConfiguration("coverage_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    advisory_only = LaunchConfiguration("advisory_only")

    declare_serial_port = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyUSB0", description="RPLidar serial port"
    )
    declare_exclude_radius = DeclareLaunchArgument(
        "exclude_radius_m", default_value="0.25",
        description="blank out /scan returns closer than this to the LiDAR (m) -- filters the robot's own laptop/cables",
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

    # Same laser mount description as coverage_launch.py / mapping_launch.py
    # -- keep all three in sync, they describe the same physical mount.
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
    serial_port = LaunchConfiguration("serial_port")

    return LaunchDescription(
        [
            declare_slam_params,
            declare_coverage_params,
            declare_use_sim_time,
            declare_advisory_only,
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

            # Builds /map live and publishes map->odom TF -- no map_server,
            # no AMCL, and (since nothing here calls its save_map service)
            # nothing ever gets written to disk.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "slam_params_file": slam_params_file,
                }.items(),
            ),

            # --- Reactive coverage driver, reading pose off live TF ---
            Node(
                package="robotpel",
                executable="coverage_planner_node",
                name="coverage_planner_node",
                output="screen",
                parameters=[
                    coverage_params_file,
                    {"use_sim_time": use_sim_time, "advisory_only": advisory_only, "localization_source": "tf"},
                ],
            ),
        ]
    )
