"""The RoboSense RS-LiDAR-16 of the a200-0553: driver configuration, the sensor's names, its self filter.

The sensor path and nothing that consumes it.  Navigation derives its 2D scan from these points, perception reads them
as they are; both import from here, and neither from the other.  The modules hold the decisions ROS-free, so that they
can be checked on the Mac; ``self_filter`` imports rclpy inside its ``main`` only.
"""

__version__ = "0.1.0"
