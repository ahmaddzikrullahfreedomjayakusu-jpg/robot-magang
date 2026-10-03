from glob import glob

from setuptools import setup

package_name = "pelcdx"

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
    description="Drive the mopping robot through a path marked by hand in RViz -- separate, additional option alongside robotpel's automatic coverage system.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "manual_waypoint_driver_node = pelcdx.manual_waypoint_driver_node:main",
        ],
    },
)
