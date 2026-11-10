"""Package setup of the ``husky_manipulator_diagnostics`` ROS 2 package (ament_python)."""

from setuptools import find_packages, setup

package_name = "husky_manipulator_diagnostics"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hannes Voss",
    maintainer_email="hannes.voss@haw-hamburg.de",
    description="UR5 + OnRobot RG6 as diagnostic_msgs for the Clearpath diagnostic_aggregator.",
    license="MIT",
    entry_points={"console_scripts": ["manipulator_diagnostics = husky_manipulator_diagnostics.node:main"]},
)
