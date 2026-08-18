# RDK-X5 ROS 2 sensor stack

Docker Compose stack for the RDK-X5 running Ubuntu 22.04 and ROS 2 Humble.
The merged Compose project contains exactly four services: `lidar`, `camera`,
`imu`, and `depth`.

## Hardware mapping

- RPLIDAR: the CP2102 `/dev/serial/by-id/...0001-if00-port0` device
- Stereo camera: dual SC230AI MIPI cameras, channel 2 (left) and channel 0
  (right), 640x352 NV12 at 10 Hz
- Left EBIMU-9DOFV6: `/dev/ttyS2` -> `/dev/imu_left`

UART3/right IMU is intentionally disabled and is not a health requirement.

The EBIMU node publishes each sensor's native axes. It does not guess a
mounting transform; add the measured static transforms in the robot TF tree.

## Start and verify

```bash
cd /home/hanium/robot_sensors
docker compose config --quiet
docker compose up -d
docker compose ps
```

`depth` waits for the camera healthcheck to receive a real combined image.
Every service uses `restart: unless-stopped`, a fixed local image tag, and
rotating Docker logs (3 x 10 MB).

Expected live topics include:

- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/image_left_raw`, `/image_right_raw`, `/image_combine_raw`
- `/image_left_raw/camera_info`, `/image_right_raw/camera_info`
- `/imu/left/data_raw`, `/imu/left/data`
- `/StereoNetNode/stereonet_depth` (`mono16`, millimetres)
- `/map` (`nav_msgs/msg/OccupancyGrid`, 0.05 m resolution)

Inspect health, resource use, restart counts, and recent logs with:

```bash
docker compose ps
docker stats --no-stream
docker inspect -f '{{.Name}} restarts={{.RestartCount}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}n/a{{end}}' \
  rplidar stereo-camera ebimu stereo-depth
docker compose logs --tail=100
```

Run the simultaneous payload/rate check from the camera container:

```bash
docker cp tools/verify_sensor_streams.py stereo-camera:/tmp/verify_sensor_streams.py
docker exec stereo-camera bash -lc \
  'source /opt/tros/humble/setup.bash; python3 /tmp/verify_sensor_streams.py --duration 12'
docker exec stereo-camera rm -f /tmp/verify_sensor_streams.py
```

The verifier exits nonzero if any required stream is missing. It also reports
camera luma samples, valid/invalid depth samples in millimetres, and finite
LiDAR ranges, plus 2D map dimensions and occupied cells so payload content can
be checked without a GUI.

## View from a Windows host

The `lidar` service also runs `slam_toolbox` and a loopback-only Foxglove
Bridge. Follow [host_view/README.md](host_view/README.md) to open an SSH tunnel,
connect to `ws://localhost:8765`, and view the camera, depth, left IMU, LiDAR,
and 2D occupancy map without installing ROS 2 on the host.

The map is LiDAR scan-matching based. IMU data is viewable but is intentionally
not fused until its physical mounting transform has been measured.

For a no-configuration browser view, run
`host_view/start_dashboard.ps1 -IdentityFile R:\.ssh\id_ed25519` on Windows,
then open `http://localhost:8080`. The fixed dashboard shows camera, colorized
depth, UART2 IMU charts, and the LiDAR occupancy map without Foxglove panels.

Check the stereo ordering from feature-match disparity:

```bash
docker cp tools/check_stereo_order.py stereo-camera:/tmp/check_stereo_order.py
docker exec stereo-camera bash -lc \
  'source /opt/tros/humble/setup.bash; python3 /tmp/check_stereo_order.py'
docker exec stereo-camera rm -f /tmp/check_stereo_order.py
```

The deployed channel order must report `"interpretation": "top_is_left"`.

If a USB LiDAR is unplugged and reappears with a different `/dev/ttyUSB*`
number, recreate only its container so Docker resolves the stable by-id path
again:

```bash
docker compose up -d --no-deps --force-recreate lidar
```

## EBIMU stream contract

The node configures ASCII quaternion output at 100 Hz and accepts only complete
10-field packets. Quaternion order is converted from the device's Z,Y,X,W to
ROS X,Y,Z,W; angular velocity is converted from degrees/s to radians/s and
acceleration from g to m/s^2. `/data_raw` declares orientation unavailable,
while `/data` contains the normalized quaternion.
