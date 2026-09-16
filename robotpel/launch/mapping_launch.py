"""Bringup for the mapping phase: RPLidar + base_footprint->laser TF + slam_toolbox.

Run this instead of launching rplidar_ros and slam_toolbox separately -- it
fixes two easy-to-miss traps: slam_toolbox's online_async_launch.py defaults
to use_sim_time=true (it will silently hang waiting for /clock on a real
robot), and nothing else in this stack publishes the base_footprint->laser
transform slam_toolbox needs to place /scan data.

robot1.py (odom -> base_footprint) must already be running separately.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    robotpel_share = get_package_share_directory("robotpel")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")
    slam_params_default = os.path.join(robotpel_share, "config", "slam_toolbox_params.yaml")

    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file", default_value=slam_params_default, description="slam_toolbox parameter file"
    )
    slam_params_file = LaunchConfiguration("slam_params_file")

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

    declare_serial_port = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyUSB0", description="RPLidar serial port"
    )
    declare_laser_frame = DeclareLaunchArgument(
        "laser_frame", default_value="laser", description="frame_id published in /scan headers"
    )
    declare_laser_x = DeclareLaunchArgument("laser_x", default_value="0.0", description="lidar offset x (m)")
    declare_laser_y = DeclareLaunchArgument("laser_y", default_value="0.0", description="lidar offset y (m)")
    declare_laser_z = DeclareLaunchArgument("laser_z", default_value="0.15", description="lidar offset z (m)")
    declare_laser_roll = DeclareLaunchArgument("laser_roll", default_value="0.0", description="lidar roll (rad)")
    declare_laser_pitch = DeclareLaunchArgument("laser_pitch", default_value="0.0", description="lidar pitch (rad)")
    # Robot drives with the LiDAR's physical "front" facing the robot's rear,
    # so its forward direction points along -x of base_footprint (see the
    # matching note in coverage_launch.py). Keep this in sync between both
    # launch files -- it's the same physical mount.
    declare_laser_yaw = DeclareLaunchArgument("laser_yaw", default_value="3.14159", description="lidar yaw (rad)")
    # RPLidar mounted upside-down reports its sweep direction reversed, which
    # shows up as a left/right mirror once the points are transformed into
    # base_footprint. This flag is rplidar_ros's own compensation for that --
    # flip it if left/right still looks mirrored in RViz after this default.
    declare_laser_inverted = DeclareLaunchArgument(
        "laser_inverted", default_value="false", description="true if the LiDAR is mounted upside-down"
    )

    serial_port = LaunchConfiguration("serial_port")
    laser_frame = LaunchConfiguration("laser_frame")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_roll = LaunchConfiguration("laser_roll")
    laser_pitch = LaunchConfiguration("laser_pitch")
    laser_yaw = LaunchConfiguration("laser_yaw")
    laser_inverted = LaunchConfiguration("laser_inverted")

    return LaunchDescription(
        [
            declare_slam_params,
            declare_exclude_radius,
            declare_exclude_min,
            declare_exclude_max,
            declare_serial_port,
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

            # Same base_footprint -> lidar transform used by coverage_launch.py.
            # Keep the laser_x/y/z/roll/pitch/yaw values identical between the
            # two launch files -- they describe the same physical mount.
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
            # LiDAR always sits, so it never gets baked into the saved map.
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

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")
                ),
                launch_arguments={
                    # slam_toolbox's own launch file defaults use_sim_time to
                    # true, which hangs forever on a real robot with no
                    # /clock publisher.
                    "use_sim_time": "false",
                    "slam_params_file": slam_params_file,
                }.items(),
            ),
        ]
    )
