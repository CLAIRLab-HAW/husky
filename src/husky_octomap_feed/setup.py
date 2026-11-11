"""Package setup of the ``husky_octomap_feed`` ROS 2 package (ament_python)."""

from glob import glob

from setuptools import find_packages, setup

package_name = "husky_octomap_feed"

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
    description="The wrist D435's depth as a throttled PointCloud2 for move_group's occupancy map monitor.",
    license="MIT",
    entry_points={"console_scripts": ["octomap_feed = husky_octomap_feed.feed:main"]},
)
