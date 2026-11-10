"""Package setup of the ``husky_lidar`` ROS 2 package (ament_python)."""

from glob import glob

from setuptools import find_packages, setup

package_name = "husky_lidar"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hannes Voss",
    maintainer_email="hannes.voss@haw-hamburg.de",
    description="The a200-0553's lidar: its models and drivers, sensor names, self filter.",
    license="MIT",
    entry_points={"console_scripts": ["self_filter = husky_lidar.self_filter:main"]},
)
