"""The UM7's gyro bias, learned while the wheels stand still, taken off before the EKF reads the yaw rate.

The EKF weighs the UM7's yaw rate against the wheels' about 9000 : 1 (R66), so at rest it integrates the gyro's bias:
on the a200-0553 AMCL's heading drifted 3 deg in about ten minutes while the robot stood (2026-11-23), and AMCL does
not correct a robot that does not move.  The driver zeroes the gyros once at start; the bias wanders with the
temperature afterwards.  This node learns it whenever the wheel odometry has reported standstill for ``STILL_S`` --
an exponential mean with time constant ``TAU_S`` -- and publishes the UM7's message with it subtracted, on
``unbiased_imu_topic()``, which the EKF reads instead of the driver's.

Checked and not taken: ``imu_complementary_filter`` (imu_tools, jazzy) learns and subtracts a gyro bias too, but
decides "at rest" by ``|a| = 9.81 +- 0.1 m/s^2`` with the threshold hard-coded (kAccelerationThreshold); this UM7
reads 10.06 at rest, so it would never learn (2026-11-23).  The wheels say standstill exactly -- platform/odom
reports 0.0 -- and do not depend on the accelerometer's scale.

The core (:class:`GyroBias`) is ROS-free; :func:`main` is the node, started from robot.yaml's platform.extras.launch
by ``launch/gyro_bias.launch.py``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

#: How long the wheels must have reported standstill before the gyro counts as at rest [s]: the base still rocks
#: on its tires right after it stops.
STILL_S = 1.0
#: Time constant of the bias mean [s]: long against the UM7's noise (2.8e-4 rad/s/sqrt(Hz), R75), short against its
#: drift with the temperature.
TAU_S = 20.0
#: Wheel speeds below these count as standing [m/s, rad/s]: platform/odom reports exactly 0.0 at rest (2026-11-23).
STILL_LINEAR_M_S = 1e-3
STILL_ANGULAR_RAD_S = 1e-3
#: The most bias it will learn per axis [rad/s]: the UM7's measured 1.5e-3 (-y) and 5.7e-3 (x) (2026-11-23) with
#: room; a base turned by hand while its wheels stand would otherwise be learned as bias.
MAX_BIAS_RAD_S = 0.02
#: Wheel odometry older than this says nothing about standstill [s]: the MCU reports at 10 Hz.
WHEEL_STALE_S = 0.5


@dataclass
class GyroBias:
    """Learns the bias while the wheels stand still; :meth:`correct` takes it off a rate."""

    bias: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    still_since: float | None = None
    wheel_stamp: float | None = None
    last_gyro_stamp: float | None = None

    def wheels(self, stamp: float, linear: float, angular: float) -> None:
        """One wheel odometry report: when standstill began, or that the base moves."""
        self.wheel_stamp = stamp
        if abs(linear) < STILL_LINEAR_M_S and abs(angular) < STILL_ANGULAR_RAD_S:
            if self.still_since is None:
                self.still_since = stamp
        else:
            self.still_since = None

    def at_rest(self, stamp: float) -> bool:
        """Whether the base has stood for ``STILL_S`` by fresh wheel odometry."""
        return (
            self.still_since is not None
            and self.wheel_stamp is not None
            and stamp - self.wheel_stamp < WHEEL_STALE_S
            and stamp - self.still_since >= STILL_S
        )

    def correct(self, stamp: float, rate: tuple[float, float, float]) -> tuple[float, float, float]:
        """Learn from ``rate`` when at rest, then return it with the bias taken off."""
        dt = 0.0 if self.last_gyro_stamp is None else max(0.0, stamp - self.last_gyro_stamp)
        self.last_gyro_stamp = stamp
        if self.at_rest(stamp) and dt > 0.0:
            alpha = min(1.0, dt / TAU_S)
            for i, w in enumerate(rate):
                learned = self.bias[i] + alpha * (w - self.bias[i])
                self.bias[i] = max(-MAX_BIAS_RAD_S, min(MAX_BIAS_RAD_S, learned))
        return tuple(w - b for w, b in zip(rate, self.bias, strict=True))


def main(argv: list[str] | None = None) -> None:  # pragma: no cover - needs a running robot
    """The node: the driver's Imu in, the same Imu with the bias off out."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    from husky_navigation.wiring import imu_topic, unbiased_imu_topic, wheel_odom_topic

    rclpy.init(args=argv)
    node = rclpy.create_node("gyro_bias")
    state = GyroBias()
    # Reliable, depth 10: a reliable publisher serves a reliable and a best-effort subscriber alike, whichever the
    # EKF's is.
    publisher = node.create_publisher(Imu, unbiased_imu_topic(), 10)
    stamp_s = lambda header: header.stamp.sec + header.stamp.nanosec * 1e-9  # noqa: E731

    def on_wheels(msg: Odometry) -> None:
        state.wheels(stamp_s(msg.header), msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_imu(msg: Imu) -> None:
        w = msg.angular_velocity
        w.x, w.y, w.z = state.correct(stamp_s(msg.header), (w.x, w.y, w.z))
        publisher.publish(msg)

    node.create_subscription(Odometry, wheel_odom_topic(), on_wheels, qos_profile_sensor_data)
    node.create_subscription(Imu, imu_topic(), on_imu, qos_profile_sensor_data)

    def say() -> None:
        x, y, z = state.bias
        node.get_logger().info(f"gyro bias (rad/s): {x:+.2e} {y:+.2e} {z:+.2e}")

    node.create_timer(60.0, say)
    from rclpy.executors import ExternalShutdownException

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C, or the SIGTERM systemd stops it with
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
