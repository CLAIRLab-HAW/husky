"""The verdict every status builder returns: a ``diagnostic_msgs`` level, a message, and whether it is out of
service rather than broken."""

from collections import namedtuple

OK, WARN, ERROR, STALE = 0, 1, 2, 3

# "INACTIVE" (out of service, grey in Cockpit) is NOT a diagnostic_msgs level -- the standard only knows
# OK/WARN/ERROR/STALE.  An own byte value would confuse the max() rollups of the aggregator and every foreign consumer
# (rqt_robot_monitor).  Hence the convention: the level stays OK (nothing is broken, after all) plus the value
# 'display=inactive'.  Consumers that do not know the convention see "OK" with a plain-text message ("arm switched off")
# -- so nothing wrong; Cockpit paints it grey.
DISPLAY_KEY = "display"
DISPLAY_INACTIVE = "inactive"

Verdict = namedtuple("Verdict", "level message inactive", defaults=(False,))


def inactive(message):
    """Out of service, not an error -- see DISPLAY_INACTIVE."""
    return Verdict(OK, message, True)
