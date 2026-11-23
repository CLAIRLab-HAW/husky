#!/usr/bin/env bash
# nav.sh -- Nav2 on the robot: start, stop, relocalize, status, and saving the map.  Run over ssh by
# scripts/on-robot.sh (task robot:nav, robot:nav:stop, robot:nav:relocalize, robot:map:save); the unit is
# clearpath-custom-navigation@<localization>.service, which the installer sets up together with the sudoers drop-in
# that lets start and stop run without a password.
#
#   nav.sh start slam [fresh]  # maps; by default CONTINUES the stored map at the pose AMCL has the robot at --
#                              # `fresh` starts an empty one
#   nav.sh start amcl          # drives against the stored map; a running SLAM hands its pose over, else relocalize
#   nav.sh stop
#   nav.sh relocalize          # scan-match AMCL's pose against the map; corrects it when the match is clear
#   nav.sh status
#   nav.sh save-map [NAME]     # SLAM's map and pose graph -> ~/maps/NAME.{pgm,yaml,posegraph,data}
#
# The stored map lives in ~/maps, outside the husky checkout: a map saved into the clone left it dirty and its next
# pull refused (2026-11-23).  The image (pgm/yaml, AMCL's) and the pose graph (posegraph/data, slam_toolbox's) are
# saved together, so the two share one frame -- the pose AMCL reports is where SLAM continues the graph.
set -euo pipefail

UNIT=clearpath-custom-navigation
MAPS="${HOME}/maps"
MAP="${NAV_MAP_NAME:-labor}"
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

# Where the running localization has the robot: "x y yaw" in the map frame, or nothing.
pose_in_map() {
    ros python3 - <<'PY'
import math, rclpy, tf2_ros
from rclpy.time import Time
rclpy.init(args=["--ros-args", "-r", "/tf:=/a200_0553/tf", "-r", "/tf_static:=/a200_0553/tf_static"])
node = rclpy.create_node("nav_pose")
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, node)
end = node.get_clock().now().nanoseconds + 5e9
while node.get_clock().now().nanoseconds < end:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:  # apart: AMCL stamps map->odom ahead, and a chained lookup misses it
        a = buf.lookup_transform("map", "odom", Time()).transform
        b = buf.lookup_transform("odom", "base_link", Time()).transform
    except tf2_ros.TransformException:
        continue
    ya, yb = 2 * math.atan2(a.rotation.z, a.rotation.w), 2 * math.atan2(b.rotation.z, b.rotation.w)
    x = a.translation.x + math.cos(ya) * b.translation.x - math.sin(ya) * b.translation.y
    y = a.translation.y + math.sin(ya) * b.translation.x + math.cos(ya) * b.translation.y
    print(f"{x:.4f} {y:.4f} {math.remainder(ya + yb, math.tau):.5f}")
    break
PY
}

# Start one instance and wait until Nav2 offers navigate_to_pose.
start_unit() {
    local loc="$1" deadline
    sudo -n systemctl start "${UNIT}@${loc}.service"
    echo "[nav] ${UNIT}@${loc} started -- waiting for navigate_to_pose ..."
    deadline=$((SECONDS + READY_TIMEOUT_S))
    # The list first, then the search: grep -q/-m1 closes the pipe at the first match, ros2 dies of the broken pipe
    # (exit 120), and pipefail makes that the pipeline's -- the 2026-11-23 save-map stopped there.
    until grep -q '/navigate_to_pose$' <<<"$(ros ros2 action list 2>/dev/null || true)"; do
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
}

# The name a slam_toolbox service has in the graph (its namespace is the launch's, not this script's).
slam_service() {
    grep -m1 "/$1\$" <<<"$(ros ros2 service list 2>/dev/null || true)"
}

publish_initialpose() {
    local x="$1" y="$2" yaw="$3" qz qw
    qz="$(awk -v a="$yaw" 'BEGIN { print sin(a / 2) }')"
    qw="$(awk -v a="$yaw" 'BEGIN { print cos(a / 2) }')"
    # 0.25 m and 0.25 rad of doubt: a hand-over, not a guess -- AMCL's particles start around it.
    ros ros2 topic pub --once /a200_0553/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: map},
          pose: {pose: {position: {x: ${x}, y: ${y}}, orientation: {z: ${qz}, w: ${qw}}},
        covariance: [0.0625, 0, 0, 0, 0, 0, 0, 0.0625, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0625]}}" >/dev/null
}

start_slam() {
    local mode="${1:-continue}" pose="" since x y yaw service log
    case "$mode" in
        continue | fresh) ;;
        *) echo "[nav] slam mode is continue or fresh, got: ${mode}" >&2; exit 2 ;;
    esac
    if [ "$mode" = continue ] && [ ! -f "${MAPS}/${MAP}.posegraph" ]; then
        echo "[nav] no stored pose graph ${MAPS}/${MAP}.posegraph -- starting a fresh map."
        mode=fresh
    fi
    if [ "$mode" = continue ]; then
        # The stored graph's frame is the stored image's, and AMCL localizes in that image: its pose is where the
        # robot stands in the graph.  Without a running AMCL, one is started -- it relocalizes itself.
        if ! systemctl is-active -q "${UNIT}@amcl.service"; then
            echo "[nav] localizing in ${MAP} first (AMCL), to continue the map from where the robot stands ..."
            start_amcl
        fi
        pose="$(pose_in_map)"
        if [ -z "$pose" ]; then
            echo "[nav] AMCL gave no pose -- not continuing blind.  Relocalize, or: nav.sh start slam fresh" >&2
            exit 1
        fi
        echo "[nav] AMCL has the robot at (x y yaw): ${pose}"
    fi
    since="$(date '+%Y-%m-%d %H:%M:%S')"
    start_unit slam
    if [ "$mode" = fresh ]; then
        echo "[nav] up: SLAM on an empty map -- save-map writes it over ${MAP}, the old one kept beside it."
        return
    fi
    read -r x y yaw <<<"$pose"
    service="$(slam_service deserialize_map)"
    ros ros2 service call "$service" slam_toolbox/srv/DeserializePoseGraph \
        "{filename: '${MAPS}/${MAP}', match_type: 2, initial_pose: {x: ${x}, y: ${y}, theta: ${yaw}}}" >/dev/null
    # The service answers empty even when the file did not load -- slam_toolbox's own log says.
    sleep 2
    log="$(journalctl -u "${UNIT}@slam.service" --since "$since" --no-pager)"
    if grep -qiE 'deserializ.*(fail|error|could not)' <<<"$log"; then
        echo "[nav] the stored pose graph did not load:" >&2
        grep -iE 'deserializ' <<<"$log" | tail -5 >&2
        exit 1
    fi
    echo "[nav] up: SLAM continues ${MAP} from (${x}, ${y}, ${yaw})."
}

start_amcl() {
    local pose="" x y yaw
    # A running SLAM hands over where it has the robot: the map AMCL loads is SLAM's.
    if systemctl is-active -q "${UNIT}@slam.service"; then
        pose="$(pose_in_map)"
        echo "[nav] SLAM has the robot at (x y yaw): ${pose:-unknown}"
    fi
    start_unit amcl
    if [ -n "$pose" ]; then
        read -r x y yaw <<<"$pose"
        publish_initialpose "$x" "$y" "$yaw"
        echo "[nav] AMCL starts at the pose SLAM handed over."
    elif ! ros python3 -m husky_navigation.relocalize; then
        # No SLAM to hand over (a stack restart stops it): the pose comes from the scan, or from a person.
        echo "[nav] no clear match -- set the pose in Echo (2D Pose Estimate)."
    fi
    echo "[nav] up: Nav2 on the robot, AMCL against ${MAP}."
}

save_map() {
    local name="${1:-$MAP}" stamp ext service
    mkdir -p "$MAPS"
    if ! systemctl is-active -q "${UNIT}@slam.service"; then
        echo "[nav] no SLAM running -- there is no map to save." >&2
        exit 1
    fi
    # The previous map stays beside the new one: a fresh run saved over the stored map would otherwise be final.
    if [ -f "${MAPS}/${name}.yaml" ]; then
        stamp="$(date +%Y%m%d-%H%M%S)"
        for ext in pgm yaml posegraph data; do
            [ -f "${MAPS}/${name}.${ext}" ] && mv "${MAPS}/${name}.${ext}" "${MAPS}/${name}.${stamp}.${ext}"
        done
        echo "[nav] the previous ${name} is kept as ${MAPS}/${name}.${stamp}.*"
    fi
    ros ros2 run nav2_map_server map_saver_cli -f "${MAPS}/${name}" --ros-args -r map:=/a200_0553/map
    # The yaml names its image relatively, so AMCL finds it beside the yaml wherever ~/maps is.
    sed -i "s|^image: .*|image: ${name}.pgm|" "${MAPS}/${name}.yaml"
    service="$(slam_service serialize_map)"
    ros ros2 service call "$service" slam_toolbox/srv/SerializePoseGraph "{filename: '${MAPS}/${name}'}" >/dev/null
    ls -la "${MAPS}/${name}".{pgm,yaml,posegraph,data}
}

case "${1:-status}" in
    start)
        case "${2:-slam}" in
            slam) start_slam "${3:-continue}" ;;
            amcl) start_amcl ;;
            *) echo "[nav] localization is slam or amcl, got: ${2}" >&2; exit 2 ;;
        esac
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
        ls -la "${MAPS}/${MAP}".* 2>/dev/null || echo "[nav] no stored map ${MAPS}/${MAP}"
        ;;
    save-map)
        save_map "${2:-}"
        ;;
    *)
        echo "usage: nav.sh start slam [fresh] | start amcl | stop | relocalize | status | save-map [NAME]" >&2
        exit 2
        ;;
esac
