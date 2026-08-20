#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

timeout 8s ros2 topic echo \
    --no-daemon \
    --spin-time 2 \
    --qos-durability transient_local \
    --qos-reliability reliable \
    --once \
    /map \
    nav_msgs/msg/OccupancyGrid \
    --field header >/dev/null

pgrep -f '/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge' >/dev/null
pgrep -f 'rosbridge_websocket.*__node:=rosbridge_telemetry' >/dev/null
pgrep -f '/image_stream_server.py' >/dev/null

python3 - <<'PY'
import socket
import urllib.request

for port in (9092, 9093):
    with socket.create_connection(("127.0.0.1", port), timeout=2):
        pass

with urllib.request.urlopen("http://127.0.0.1:8080/health", timeout=2) as response:
    if response.read() != b"ok":
        raise SystemExit("dashboard health response mismatch")
PY
