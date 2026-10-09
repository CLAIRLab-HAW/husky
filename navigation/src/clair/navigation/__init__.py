"""Navigation layer of the a200-0553.

ROS-FREE.  This package holds the decisions (topic and frame names, the scan's parameters, the synthetic map), not the
nodes -- so that they can be checked on the Mac without ROS.  The launch files in the repo's ``launch/`` import from
here instead of stating the values a second time.  The RS16's sensor path the scan builds on is ``clair.rs16``'s.
"""

__version__ = "0.1.0"
