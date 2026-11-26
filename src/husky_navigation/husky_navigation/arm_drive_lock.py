"""Lock autonomous driving while the arm is out of a drive pose.

Nav2's footprint covers the base and the arm in ``forward`` -- the gripper 0.82 m ahead of base_link, the front edge
at 0.87 m (nav2_params.yaml) -- or folded in ``packed`` (x 0.01..0.34 m); every other pose puts the arm where Nav2
does not know it is, and the base would carry it into whatever stands there.  This node compares the arm's joints
with those two poses, read from robot.yaml (their source, which wakeup.sh and the robot profile read as well), and
publishes ``arm_drive_lock_topic()`` -- ``true`` while the arm stands outside both.

twist_mux holds it as the lock ``arm_out`` (robot.yaml, priority 5): above Nav2's ``external`` input (1), below the
interactive marker (8), the joystick (10) and the RC (12) -- a person can still drive the base, the autonomy cannot.
The lock is fail-safe twice: the node says ``true`` when the joint states fall silent, and twist_mux counts a lock
that stops arriving (timeout 0.5 s) as locked.  The plan server reads the same topic and refuses a drive on
/motion/nav_cmd with ``precondition_failed``, naming the reason.

The core (:class:`ArmDriveLock`) is ROS-free; :func:`main` is the node, started from robot.yaml's
platform.extras.launch by ``launch/arm_drive_lock.launch.py``.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from math import isfinite
from pathlib import Path

#: The poses the base may drive in: the ones inside Nav2's footprint (UR5 FK against TF, 2026-11-23).  ``tuck`` puts
#: the gripper 0.71 m ahead and fits too, but the owner keeps it out of driving.
DRIVE_POSES = ("forward", "packed")
#: How far a joint may stand from a drive pose and still count as in it [rad]: a MoveIt goal ends within ~0.01.
JOINT_TOLERANCE_RAD = 0.05
#: Joint states older than this say nothing about the arm [s]: the relay publishes at ~50 Hz.
JOINT_STALE_S = 0.5
#: The UR5's joints in the order robot.yaml's ``joints`` lists them, after the arm's name prefix.
UR_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
             "wrist_3_joint")  # fmt: skip
#: robot.yaml's path on the robot (a symlink into the husky clone).
ROBOT_YAML = Path("/etc/clearpath/robot.yaml")


def drive_poses_from_robot_yaml(text: str, names: Sequence[str] = DRIVE_POSES) -> dict[str, dict[str, float]]:
    """``{pose: {joint: rad}}`` for ``names`` from robot.yaml's first arm; Clearpath names an unnamed arm ``arm_0``.

    :raises KeyError: when a pose is missing -- a lock without its poses would hold the base for good.
    """
    import yaml

    arm = yaml.safe_load(text)["manipulators"]["arms"][0]
    prefix = arm.get("name") or "arm_0"
    poses = {p["name"]: p["joints"] for p in arm.get("poses", [])}
    missing = [n for n in names if n not in poses]
    if missing:
        raise KeyError(f"robot.yaml names no pose {missing} for {prefix}")
    return {n: {f"{prefix}_{j}": float(v) for j, v in zip(UR_JOINTS, poses[n], strict=True)} for n in names}


@dataclass
class ArmDriveLock:
    """Whether the arm stands in one of ``poses``, by the last joint state."""

    poses: Mapping[str, Mapping[str, float]]
    positions: dict[str, float] = field(default_factory=dict)
    stamp: float | None = None
    _stamps_s: dict[str, float] = field(default_factory=dict, init=False, repr=False)

    def joints(self, stamp: float, names: Sequence[str], values: Sequence[float]) -> None:
        """One joint state: the arm's joints among ``names``."""
        wanted = {joint for pose in self.poses.values() for joint in pose}
        for index, name in enumerate(names):
            if name not in wanted:
                continue
            try:
                value = float(values[index])
            except (IndexError, TypeError, ValueError):
                value = float("nan")
            if not isfinite(value) or not isfinite(stamp):
                self.positions.pop(name, None)
                self._stamps_s.pop(name, None)
                continue
            self.positions[name] = value
            self._stamps_s[name] = stamp
            self.stamp = stamp

    def verdict(self, now: float) -> tuple[bool, str]:
        """``(locked, why)``: unlocked in a drive pose, locked out of one or without fresh joint states."""
        if self.stamp is None or not isfinite(now) or not 0.0 <= now - self.stamp <= JOINT_STALE_S:
            return True, "no fresh joint states of the arm"
        wanted = {joint for pose in self.poses.values() for joint in pose}
        missing = wanted - self.positions.keys()
        if missing:
            return True, f"joint states lack {sorted(missing)}"
        stale = sorted(j for j in wanted if not 0.0 <= now - self._stamps_s.get(j, float("nan")) <= JOINT_STALE_S)
        if stale:
            return True, f"no fresh joint states of the arm: {stale}"
        nearest, worst = "", (None, float("inf"))
        for name, pose in self.poses.items():
            off = max(((j, abs(self.positions[j] - v)) for j, v in pose.items()), key=lambda jv: jv[1])
            if off[1] <= JOINT_TOLERANCE_RAD:
                return False, f"arm in {name}"
            if off[1] < worst[1]:
                nearest, worst = name, off
        return True, f"arm out of the drive poses: nearest {nearest}, {worst[0]} {worst[1]:.2f} rad off"


def main(argv: list[str] | None = None) -> None:  # pragma: no cover - needs a running robot
    """The node: joint states in, the lock out -- on every joint state and on a timer that catches their silence."""
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool

    from husky_navigation.wiring import arm_drive_lock_topic, joint_states_topic

    rclpy.init(args=argv)
    node = rclpy.create_node("arm_drive_lock")
    lock = ArmDriveLock(drive_poses_from_robot_yaml(ROBOT_YAML.read_text(encoding="utf-8")))
    publisher = node.create_publisher(Bool, arm_drive_lock_topic(), 10)
    was = {"locked": None}
    now_s = lambda: node.get_clock().now().nanoseconds * 1e-9  # noqa: E731

    def publish() -> None:
        locked, why = lock.verdict(now_s())
        publisher.publish(Bool(data=locked))
        if locked != was["locked"]:  # the flips, not every tick of a moving joint
            node.get_logger().info(f"{'LOCKED' if locked else 'free'}: {why}")
            was["locked"] = locked

    def on_joints(msg: JointState) -> None:
        lock.joints(now_s(), msg.name, msg.position)

    node.create_subscription(JointState, joint_states_topic(), on_joints, 50)
    node.create_timer(0.05, publish)
    node.get_logger().info(f"drive poses {list(lock.poses)} from {ROBOT_YAML}, tolerance {JOINT_TOLERANCE_RAD} rad")
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
