#!/usr/bin/env bash
# nav.sh -- Nav2 on the robot: start, stop, status, and saving the SLAM map.  Run over ssh by scripts/on-robot.sh
# (task robot:nav, robot:nav:stop, robot:map:save); the unit is clearpath-custom-navigation@<localization>.service,
# which the installer sets up together with the sudoers drop-in that lets start and stop run without a password.
#
#   nav.sh start slam|amcl     # the other localization stops (Conflicts=); waits for navigate_to_pose
#   nav.sh stop
#   nav.sh relocalize          # scan-match AMCL's pose against the map; corrects it when the match is clear
#   nav.sh status
#   nav.sh save-map NAME       # the running SLAM's map -> /tmp/NAME.{pgm,yaml}, for task robot:map:save to fetch
set -euo pipefail

UNIT=clearpath-custom-navigation
# Outside the husky checkout: a map saved into it left the robot's clone dirty, and its next pull refused
# (2026-11-23).  The map reaches the robot as a commit, like every other file.
MAPS="/tmp"
# How long Nav2 may take to offer navigate_to_pose [s]: slam_toolbox and the lifecycle bring-up took ~10 s in the
# container (2026-11-23).
READY_TIMEOUT_S=90

ros() {
    set +u
    # shellcheck disable=SC1091  # the robot's own setup, sourced at run time
    source /etc/clearpath/setup.bash
    set -u
    "$@"
}

case "${1:-status}" in
    start)
        loc="${2:-slam}"
        case "$loc" in slam | amcl) ;; *) echo "[nav] localization is slam or amcl, got: ${loc}" >&2; exit 2 ;; esac
        # AMCL starts at the map's origin -- where the mapping run began.  A robot standing elsewhere would be
        # localized wrong, so a running SLAM hands over where it has the robot: the stored map is its map.
        pose=""
        if [ "$loc" = amcl ] && systemctl is-active -q "${UNIT}@slam.service"; then
            pose="$(ros python3 - <<'PY'
import math, rclpy, tf2_ros
from rclpy.time import Time
rclpy.init(args=["--ros-args", "-r", "/tf:=/a200_0553/tf", "-r", "/tf_static:=/a200_0553/tf_static"])
node = rclpy.create_node("nav_handover")
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, node)
end = node.get_clock().now().nanoseconds + 5e9
while node.get_clock().now().nanoseconds < end:
    rclpy.spin_once(node, timeout_sec=0.1)
    if buf.can_transform("map", "base_link", Time()):
        t = buf.lookup_transform("map", "base_link", Time()).transform
        print(t.translation.x, t.translation.y, 2 * math.atan2(t.rotation.z, t.rotation.w))
        break
PY
)"
            echo "[nav] SLAM has the robot at (x y yaw): ${pose:-unknown}"
        fi
        sudo -n systemctl start "${UNIT}@${loc}.service"
        echo "[nav] ${UNIT}@${loc} started -- waiting for navigate_to_pose ..."
        deadline=$((SECONDS + READY_TIMEOUT_S))
        until ros ros2 action list 2>/dev/null | grep -q '/navigate_to_pose$'; do
            if ! systemctl is-active -q "${UNIT}@${loc}.service"; then
                echo "[nav] the unit stopped:" >&2
                journalctl -u "${UNIT}@${loc}.service" -n 20 --no-pager >&2 || true
                exit 1
            fi
            if [ "$SECONDS" -ge "$deadline" ]; then
                echo "[nav] no navigate_to_pose after ${READY_TIMEOUT_S} s" >&2
                exit 1
            fi
            sleep 2
        done
        if [ -n "$pose" ]; then
            read -r x y yaw <<<"$pose"
            qz="$(awk -v a="$yaw" 'BEGIN { print sin(a / 2) }')"
            qw="$(awk -v a="$yaw" 'BEGIN { print cos(a / 2) }')"
            # 0.25 m and 0.25 rad of doubt: a hand-over, not a guess -- AMCL's particles start around it.
            ros ros2 topic pub --once /a200_0553/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
                "{header: {frame_id: map},
                  pose: {pose: {position: {x: ${x}, y: ${y}}, orientation: {z: ${qz}, w: ${qw}}},
                covariance: [0.0625, 0, 0, 0, 0, 0, 0, 0.0625, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0625]}}" >/dev/null
            echo "[nav] AMCL starts at the pose SLAM handed over."
        elif [ "$loc" = amcl ]; then
            # No SLAM to hand over (a stack restart stops it): find the pose from the scan instead.
            if ! ros python3 -m husky_navigation.relocalize; then
                echo "[nav] no clear match -- set the pose in Echo (2D Pose Estimate)."
            fi
        fi
        echo "[nav] up: Nav2 on the robot, localization ${loc}."
        ;;
    stop)
        for loc in slam amcl; do
            if systemctl is-active -q "${UNIT}@${loc}.service"; then
                sudo -n systemctl stop "${UNIT}@${loc}.service"
                echo "[nav] ${UNIT}@${loc} stopped."
            fi
        done
        ;;
    relocalize)
        ros python3 -m husky_navigation.relocalize
        ;;
    status)
        for loc in slam amcl; do
            printf '%-36s %s\n' "${UNIT}@${loc}" "$(systemctl is-active "${UNIT}@${loc}.service" || true)"
        done
        ;;
    save-map)
        name="${2:?usage: nav.sh save-map NAME}"
        ros ros2 run nav2_map_server map_saver_cli -f "${MAPS}/${name}" --ros-args -r map:=/a200_0553/map
        ls -la "${MAPS}/${name}".*
        ;;
    *)
        echo "usage: nav.sh start slam|amcl | stop | relocalize | status | save-map NAME" >&2
        exit 2
        ;;
esac
