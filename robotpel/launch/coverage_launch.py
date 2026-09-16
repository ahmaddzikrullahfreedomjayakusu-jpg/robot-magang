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
    default_coverage_params = os.path.join(share_dir, "config", "coverage_params.yaml")

    map_yaml = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("params_file")
    coverage_params_file = LaunchConfiguration("coverage_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_map = DeclareLaunchArgument(
        "map", default_value=default_map, description="Full path to the saved map .yaml"
    )
    declare_nav2_params = DeclareLaunchArgument(
        "params_file", default_value=default_nav2_params, description="Nav2 parameter file"
    )
    declare_coverage_params = DeclareLaunchArgument(
        "coverage_params_file",
        default_value=default_coverage_params,
        description="coverage_planner_node parameter file",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )
    declare_serial_port = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyUSB0", description="RPLidar serial port"
    )
    serial_port = LaunchConfiguration("serial_port")

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

    # robot1.py only publishes odom -> base_footprint. Nothing in the existing
    # hardware stack publishes base_footprint -> <lidar frame>, so without this
    # static transform Nav2's costmaps cannot place /scan data and effectively
    # see no obstacles at all. Verify the real frame_id with
    # `ros2 topic echo /scan --field header.frame_id` once the RPLidar driver
    # is running, and measure the actual mount offset on the robot -- the
    # defaults below are placeholders and must be corrected before relying on
    # obstacle avoidance.
    #
    # laser_yaw defaults to pi (180 deg): the robot drives with the LiDAR's
    # physical "front" facing the robot's rear, so the LiDAR's own forward
    # direction must be declared as pointing along -x of base_footprint, not
    # +x. Getting this backwards makes Nav2 think obstacles ahead of the
    # robot are behind it (and vice versa) -- verify this still matches your
    # actual mounting before trusting obstacle avoidance.
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
            declare_coverage_params,
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
                    "serial_baudrate": 115200,  # A1 / A2. A3 uses 256000.
                    "frame_id": laser_frame,
                    "inverted": laser_inverted,
                    "angle_compensate": True,
                }],
            ),

            # base_footprint -> lidar: without this, /scan can never be
            # transformed into the robot/costmap frames (see comment above).
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

            # Blanks out the fixed rear sector where the laptop powering the
            # LiDAR always sits, so Nav2 never treats it as a real obstacle.
            # amcl/local_costmap/global_costmap in nav2_params.yaml all read
            # /scan_filtered (not raw /scan) -- keep those in sync if this
            # topic name ever changes.
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

            # --- Localization: map + AMCL ---
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
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[{"autostart": True, "node_names": lifecycle_nodes_localization, "use_sim_time": use_sim_time}],
            ),

            # --- Reactive coverage driver ---
            # Nav2's controller/planner/behavior/bt_navigator are NOT used
            # for driving anymore -- coverage_planner_node drives the robot
            # directly (see its own module docstring for why). Only
            # map_server + AMCL above are still needed, purely for
            # localization (this node's own lane/boundary logic).
            Node(
                package="robotpel",
                executable="coverage_planner_node",
                name="coverage_planner_node",
                output="screen",
                parameters=[coverage_params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
