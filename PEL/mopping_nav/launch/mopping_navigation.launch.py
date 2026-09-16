from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory("mopping_nav")
    params = os.path.join(share_dir, "config", "mopping_params.yaml")

    return LaunchDescription(
        [
            Node(
                package="mopping_nav",
                executable="coverage_grid_tracker",
                name="coverage_grid_tracker",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="mopping_nav",
                executable="corridor_shuttle_bug2",
                name="corridor_shuttle_bug2",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="mopping_nav",
                executable="missed_area_dispatcher",
                name="missed_area_dispatcher",
                output="screen",
                parameters=[params],
            ),
        ]
    )
