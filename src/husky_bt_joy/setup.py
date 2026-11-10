"""Package setup of the ``husky_bt_joy`` ROS 2 package (ament_python)."""

from glob import glob

from setuptools import find_packages, setup

package_name = "husky_bt_joy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hannes Voss",
    maintainer_email="hannes.voss@haw-hamburg.de",
    description="The phone as the robot's Bluetooth joystick: a BlueZ pairing agent and the phone's teleop chain.",
    license="MIT",
    entry_points={"console_scripts": ["bt_joy_pairing = husky_bt_joy.pairing:main"]},
)
