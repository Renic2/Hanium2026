#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/config/udp_only.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-/config/udp_only.xml}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

child_pids=()

shutdown_children() {
    trap - EXIT INT TERM
    if ((${#child_pids[@]})); then
        kill -TERM "${child_pids[@]}" 2>/dev/null || true
        wait "${child_pids[@]}" 2>/dev/null || true
    fi
}

trap shutdown_children EXIT INT TERM

ros2 launch rplidar_ros rplidar_a1_launch.py \
    serial_port:=/dev/rplidar \
    serial_baudrate:=115200 \
    frame_id:=laser_frame \
    inverted:=false \
    angle_compensate:=true \
    scan_mode:=Standard &
child_pids+=("$!")

# There is no wheel odometry in this sensor-only stack. A fixed odom-to-laser
# prior lets slam_toolbox use its own scan matcher to estimate map-to-odom.
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id odom --child-frame-id laser_frame &
child_pids+=("$!")

ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=/config/slam_params.yaml \
    use_sim_time:=false &
child_pids+=("$!")

# The WebSocket is deliberately loopback-only. The host connects through the
# SSH tunnel documented in host_view/README.md, so sensor data is not exposed
# to other machines on the Wi-Fi network.
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    address:=127.0.0.1 \
    port:=8765 \
    capabilities:='[services,connectionGraph,assets]' &
child_pids+=("$!")

# Keep telemetry on rosbridge. Camera/depth use a purpose-built binary stream
# below so Python JSON/base64 serialization cannot consume multiple CPU cores.
ros2 run rosbridge_server rosbridge_websocket --ros-args \
    -r __node:=rosbridge_telemetry \
    -p address:=127.0.0.1 \
    -p port:=9092 \
    -p unregister_timeout:=2.0 \
    -p websocket_ping_interval:=10.0 \
    -p websocket_ping_timeout:=5.0 &
rosbridge_telemetry_pid="$!"
child_pids+=("${rosbridge_telemetry_pid}")

python3 /image_stream_server.py &
image_stream_pid="$!"
child_pids+=("${image_stream_pid}")

python3 /dashboard_server.py &
child_pids+=("$!")

# Keep the telemetry bridge bounded. Image transport is raw binary and does not
# allocate rosbridge JSON/base64 buffers.
watch_rosbridge_memory() {
    while kill -0 "${rosbridge_telemetry_pid}" 2>/dev/null && kill -0 "${image_stream_pid}" 2>/dev/null; do
        bridge_pids="$(pgrep -d, -f 'rosbridge_websocket.*__node:=rosbridge_telemetry' || true)"
        if [[ -n "${bridge_pids}" ]]; then
            bridge_rss_kib="$(ps -o rss= -p "${bridge_pids}" 2>/dev/null | awk '{ total += $1 } END { print total + 0 }')"
            if [[ "${bridge_rss_kib}" =~ ^[0-9]+$ ]] && (( bridge_rss_kib > 262144 )); then
                echo "[mapping] telemetry rosbridge RSS ${bridge_rss_kib} KiB exceeded 256 MiB; restarting container" >&2
                kill "${rosbridge_telemetry_pid}" 2>/dev/null || true
                return 1
            fi
        fi
        sleep 5
    done
}

watch_rosbridge_memory &
child_pids+=("$!")

set +e
wait -n "${child_pids[@]}"
child_status=$?
set -e

echo "[mapping] a managed process exited with status ${child_status}" >&2
exit "${child_status}"
