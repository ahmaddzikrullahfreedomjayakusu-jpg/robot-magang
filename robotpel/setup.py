from glob import glob

from setuptools import setup

package_name = "robotpel"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="ahmaddzikrullahfreedomjayakusu@gmail.com",
    description="Boustrophedon coverage navigation for the hoverboard mopping robot, built on SLAM + AMCL + Nav2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "coverage_planner_node = robotpel.coverage_planner_node:main",
            "cmd_vel_to_motor_bridge = robotpel.cmd_vel_to_motor_bridge:main",
            "scan_blind_spot_filter = robotpel.scan_blind_spot_filter:main",
            "waypoint_coverage_node = robotpel.waypoint_coverage_node:main",
        ],
    },
)
