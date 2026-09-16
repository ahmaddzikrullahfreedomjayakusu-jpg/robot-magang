from glob import glob
from setuptools import setup

package_name = "mopping_nav"

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
    maintainer_email="robot@example.com",
    description="Coverage navigation nodes for an autonomous mopping robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "coverage_grid_tracker = mopping_nav.coverage_grid_tracker:main",
            "corridor_shuttle_bug2 = mopping_nav.corridor_shuttle_bug2:main",
            "missed_area_dispatcher = mopping_nav.missed_area_dispatcher:main",
        ],
    },
)
