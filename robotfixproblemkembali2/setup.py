from glob import glob

from setuptools import setup

package_name = "robotfixproblemkembali2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="ahmaddzikrullahfreedomjayakusu@gmail.com",
    description="Frozen snapshot of robotpel_manual_waypoints -- safe rollback copy, saved once outbound+return waypoint driving worked reliably (incl. reverse-arc stuck-escape).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "manual_waypoint_driver_node = robotfixproblemkembali2.manual_waypoint_driver_node:main",
        ],
    },
)
