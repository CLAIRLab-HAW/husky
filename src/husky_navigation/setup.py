"""Package setup of the ``husky_navigation`` ROS 2 package (ament_python)."""

from glob import glob

from setuptools import find_packages, setup

package_name = "husky_navigation"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config/behavior_trees", glob("config/behavior_trees/*")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hannes Voss",
    maintainer_email="hannes.voss@haw-hamburg.de",
    description="The a200-0553's navigation layer: the lidar's 2D scan and Nav2.",
    license="MIT",
)
