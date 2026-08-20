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

The EBIMU node preserves the native sensor topics and also performs a startup
stationary calibration. The calibrated topics rotate the vertically mounted
sensor into `base_link`, remove the measured gyro zero bias, and retain gravity.

## Start and verify

```bash
cd /home/hanium/robot_sensors
docker compose config --quiet
docker compose up -d
docker compose ps
```

Place the robot on level ground and do not move it for at least 3 seconds after
the `imu` service starts. Calibration deliberately waits and retries while the
robot is moving.

`depth` waits for the camera healthcheck to receive a real combined image.
Every service uses `restart: unless-stopped`, a fixed local image tag, and
rotating Docker logs (3 x 10 MB).

Expected live topics include:

- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/image_left_raw`, `/image_right_raw`, `/image_combine_raw`
- `/image_left_raw/camera_info`, `/image_right_raw/camera_info`
- `/imu/left/data_raw`, `/imu/left/data` (native sensor axes)
- `/imu/left/data_raw_calibrated`, `/imu/left/data_calibrated` (`base_link` axes)
- `/StereoNetNode/stereonet_depth` (`mono16`, millimetres)
- `/map` (`nav_msgs/msg/OccupancyGrid`, 0.05 m resolution)

## 센서 토픽 가져오기와 처리 방법

### 1. RDK-X5에서 ROS 2 토픽 확인하기

모든 컨테이너가 host network와 `ROS_DOMAIN_ID=0`을 사용하므로, ROS 2가
설치된 `rplidar` 컨테이너 하나에서 카메라, Depth, UART2 IMU, LiDAR와 맵
토픽을 모두 확인할 수 있습니다.

```bash
cd /home/hanium/robot_sensors
docker exec -it rplidar bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# 토픽 이름과 메시지 타입
ros2 topic list -t

# 특정 토픽의 publisher, subscriber, QoS 확인
ros2 topic info -v /scan

# 발행 주기와 전송 대역폭 확인
ros2 topic hz /scan
ros2 topic bw /scan
```

카메라, Depth, IMU와 LiDAR는 sensor-data QoS(`best_effort`, `volatile`)를
사용합니다. `ros2 topic echo`에서 데이터가 보이지 않으면 아래처럼 QoS를
명시합니다. `/map`은 마지막 맵을 새 subscriber도 받을 수 있도록
`reliable`, `transient_local`을 사용합니다.

```bash
# sensor-data QoS 예시
ros2 topic echo --once \
  --qos-reliability best_effort --qos-durability volatile \
  /imu/left/data_calibrated sensor_msgs/msg/Imu

# map QoS 예시
ros2 topic echo --once \
  --qos-reliability reliable --qos-durability transient_local \
  /map nav_msgs/msg/OccupancyGrid --field info
```

주요 토픽의 계약은 다음과 같습니다.

| 토픽 | 메시지 타입 / 예상 주기 | 데이터 형태 | 주 처리 방법 |
| --- | --- | --- | --- |
| `/image_left_raw` | `sensor_msgs/msg/Image`, 약 10 Hz | 640x352, `nv12`, 337,920 bytes | NV12를 BGR/RGB로 변환 후 영상 처리 |
| `/image_right_raw` | `sensor_msgs/msg/Image`, 약 10 Hz | 640x352, `nv12`, 337,920 bytes | 왼쪽 영상과 timestamp를 맞춰 stereo 처리 |
| `/image_combine_raw` | `sensor_msgs/msg/Image`, 약 10 Hz | 640x704, `nv12`, 위쪽 left/아래쪽 right | NV12 변환 후 높이 352 기준으로 분리 |
| `/image_left_raw/camera_info` | `sensor_msgs/msg/CameraInfo` | `K`, `D`, `R`, `P`, distortion model | 보정, rectification, 3D 투영에 사용 |
| `/image_right_raw/camera_info` | `sensor_msgs/msg/CameraInfo` | 오른쪽 카메라 내부/외부 파라미터 | stereo calibration과 disparity에 사용 |
| `/StereoNetNode/stereonet_depth` | `sensor_msgs/msg/Image`, 약 10 Hz | 640x352, little-endian `mono16`, 단위 mm | `uint16`로 해석하고 m로 변환 |
| `/StereoNetNode/stereonet_depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth 영상의 투영 파라미터 | Depth 픽셀을 3D 점으로 역투영 |
| `/imu/left/data_raw` | `sensor_msgs/msg/Imu`, 약 100 Hz | `imu_left_link`, quaternion 없음, 센서 native 축 | 원본 측정값 점검과 재보정용 |
| `/imu/left/data` | `sensor_msgs/msg/Imu`, 약 100 Hz | `imu_left_link`, native quaternion + rad/s + m/s² | EBIMU 원본 자세와 축 점검용 |
| `/imu/left/data_raw_calibrated` | `sensor_msgs/msg/Imu`, 약 100 Hz | `base_link`, quaternion 없음 | 장착축 회전·gyro bias가 보정된 필터 입력 |
| `/imu/left/data_calibrated` | `sensor_msgs/msg/Imu`, 약 100 Hz | `base_link`, 보정 quaternion + rad/s + m/s² | 대시보드, 자세 계산, EKF 입력에 권장 |
| `/scan` | `sensor_msgs/msg/LaserScan`, 약 7.5 Hz | 각도 rad, 거리 m, 보통 1,080 ranges | 유효 range를 XY 점으로 변환 |
| `/map` | `nav_msgs/msg/OccupancyGrid`, 약 0.5 Hz | 0.05 m/cell, `-1/0..100` | 2D grid, 경로 계획, 장애물 판정 |
| `/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `map` 좌표계의 SLAM pose | 로봇 위치/방향과 공분산 사용 |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 좌표계 사이 translation/quaternion | `tf2_ros`로 센서 데이터를 같은 frame에 변환 |

다음은 보정·시각화·상태 확인에 사용하는 보조 토픽입니다.

| 보조 토픽 | 메시지 형태 | 처리 방법 |
| --- | --- | --- |
| `/StereoNetNode/rectify_left_image`, `/StereoNetNode/rectify_right_image` | `sensor_msgs/msg/Image`, 640x352 `nv12` | 렌즈 왜곡이 보정된 stereo 입력으로 사용하며 NV12 변환 방법은 raw 영상과 동일 |
| `/StereoNetNode/rectify_left_image/camera_info`, `/StereoNetNode/rectify_right_image/camera_info` | `sensor_msgs/msg/CameraInfo` | rectified 영상과 동일 timestamp/frame으로 묶어 투영 파라미터 사용 |
| `/map_metadata` | `nav_msgs/msg/MapMetaData` | 맵의 resolution, width, height, origin만 필요할 때 사용 |
| `/slam_toolbox/scan_visualization` | `sensor_msgs/msg/LaserScan` | SLAM이 처리한 scan을 RViz/Foxglove에서 진단할 때 사용 |
| `/slam_toolbox/graph_visualization` | `visualization_msgs/msg/MarkerArray` | pose graph node/edge 시각화용이며 경로 계획 입력으로 직접 사용하지 않음 |
| `/slam_toolbox/update`, `/slam_toolbox/feedback` | `InteractiveMarkerUpdate`, `InteractiveMarkerFeedback` | 대화형 graph 수정용 내부 토픽; 일반 센서 처리에서는 구독하지 않음 |
| `/connected_clients`, `/client_count` | `rosbridge_msgs/msg/ConnectedClients`, `std_msgs/msg/Int32` | ROSBridge 연결 수 모니터링과 과부하 진단에 사용 |
| `/parameter_events`, `/rosout` | ROS 2 표준 parameter/log 메시지 | 설정 변경과 노드 로그 진단용; 센서 데이터셋에는 보통 기록하지 않음 |

현재 raw 카메라의 `header.frame_id`는 비어 있을 수 있습니다. Depth는
`camera_optical_frame`, IMU 원본은 `imu_left_link`, 보정 IMU는 `base_link`,
LiDAR는 `laser_frame`, 맵은 `map` frame을 사용합니다. 여러 센서를 합칠 때는
각 메시지의 `header.frame_id`를 확인하고, 카메라·LiDAR의 실제 장착 위치에
맞는 static TF를 사용해야 합니다.

카메라와 Depth의 **ROS 원본 토픽 데이터 자체는 회전하지 않습니다**. 웹
대시보드와 아래 예제 코드가 현재 장착 방향에 맞춰 표시 단계에서 180도
회전합니다. 원본 좌표가 필요한 알고리즘에서는 회전 전 배열을 사용하십시오.

### 2. 카메라 토픽

큰 `data` 배열 전체를 터미널에 출력하면 터미널과 DDS가 느려질 수 있으므로
CLI에서는 인코딩, 크기, 주기만 확인하고 실제 프레임은 subscriber나
rosbag으로 받는 것을 권장합니다.

```bash
ros2 topic type /image_left_raw
ros2 topic hz /image_left_raw
ros2 topic echo --once --qos-reliability best_effort \
  /image_left_raw sensor_msgs/msg/Image --field encoding
ros2 topic echo --once --qos-reliability best_effort \
  /image_left_raw/camera_info sensor_msgs/msg/CameraInfo
```

`sensor_msgs/msg/Image`의 핵심 필드는 `height`, `width`, `encoding`, `step`,
`is_bigendian`, `data`입니다. 이 프로젝트의 NV12는 먼저 Y plane 전체가 오고
그 뒤에 UV가 교차 배치됩니다. Python/OpenCV에서는 다음처럼 변환합니다.

아래 예제를 실행하려면 처리 환경에 NumPy와 OpenCV가 필요합니다.

```python
import cv2
import numpy as np

def nv12_to_bgr(msg, rotate_180=True):
    expected = msg.width * msg.height * 3 // 2
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=expected)
    nv12 = raw.reshape(msg.height * 3 // 2, msg.width)
    bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    if rotate_180:
        bgr = cv2.rotate(bgr, cv2.ROTATE_180)  # 현재 장착 방향 보정
    return bgr
```

`/image_combine_raw`을 변환하면 결과 높이는 704입니다. 현재 배치는 위쪽
352줄이 left, 아래쪽 352줄이 right입니다. 결합 영상 전체를 먼저 회전하면
위아래 순서도 바뀌므로, 다음처럼 먼저 분리하고 각각 회전합니다.

```python
combined = nv12_to_bgr(msg, rotate_180=False)
left = cv2.rotate(combined[:352], cv2.ROTATE_180)
right = cv2.rotate(combined[352:], cv2.ROTATE_180)
```

Stereo 연산 시에는 두 영상의 `header.stamp`를 기준으로 동기화하고
`CameraInfo`의 `K/D/R/P`를 함께 사용합니다.

### 3. Depth 토픽

```bash
ros2 topic hz /StereoNetNode/stereonet_depth
ros2 topic echo --once --qos-reliability best_effort \
  /StereoNetNode/stereonet_depth sensor_msgs/msg/Image --field encoding
ros2 topic echo --once --qos-reliability best_effort \
  /StereoNetNode/stereonet_depth/camera_info sensor_msgs/msg/CameraInfo
```

Depth는 픽셀마다 16-bit unsigned 정수 한 개가 들어 있는 `mono16` 영상이며
값의 단위는 mm입니다. `0`과 `65535`는 무효값으로 취급합니다.

```python
import cv2
import numpy as np

def depth_to_metres(msg):
    dtype = ">u2" if msg.is_bigendian else "<u2"
    depth_mm = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
    depth_mm = cv2.rotate(depth_mm, cv2.ROTATE_180)  # 카메라와 같은 방향
    valid = (depth_mm > 0) & (depth_mm < 65535)
    depth_m = depth_mm.astype(np.float32) * 0.001
    depth_m[~valid] = np.nan
    return depth_m, valid

def colorize_depth(depth_m):
    normalized = np.nan_to_num((depth_m - 0.15) / (4.0 - 0.15), nan=0.0)
    image8 = (np.clip(normalized, 0.0, 1.0) * 255).astype(np.uint8)
    return cv2.applyColorMap(image8, cv2.COLORMAP_TURBO)
```

Depth 픽셀 `(u, v)`를 카메라 좌표의 3D 점으로 바꾸려면 CameraInfo의
`fx=K[0]`, `fy=K[4]`, `cx=K[2]`, `cy=K[5]`와 깊이 `z`(m)를 사용합니다.

```text
x = (u - cx) * z / fx
y = (v - cy) * z / fy
z = depth_m[v, u]
```

### 4. UART2 IMU 토픽

```bash
# 권장: 장착축과 gyro zero bias가 보정된 base_link 데이터
ros2 topic hz /imu/left/data_calibrated
ros2 topic echo --once --qos-reliability best_effort \
  /imu/left/data_calibrated sensor_msgs/msg/Imu

# 비교/진단: 센서 native 축 원본
ros2 topic echo --once --qos-reliability best_effort \
  /imu/left/data sensor_msgs/msg/Imu
```

`orientation`은 ROS 순서 `(x, y, z, w)`의 quaternion이고,
`angular_velocity`는 rad/s, `linear_acceleration`은 m/s²입니다.
`/imu/left/data_raw`은 `orientation_covariance[0] == -1`로 자세가 없음을
표시하고 `/imu/left/data`에는 정규화 quaternion이 들어갑니다. 같은 규칙이
두 calibrated 토픽에도 적용됩니다.

`imu` 컨테이너를 시작하거나 재시작할 때 로봇을 **평평한 바닥에 두고 3초
이상 완전히 정지**시킵니다. 노드는 연속 300개 정지 샘플에서 다음 값을
계산합니다.

- 평균 가속도 방향을 `base_link`의 `+Z`로 맞추는 장착 회전
- 정지 상태의 평균 각속도를 이용한 x/y/z gyro zero bias
- 가속도 norm을 표준 중력 `9.80665 m/s²`에 맞추는 scale
- 정지 샘플 분산을 회전한 각속도·가속도 covariance

현재처럼 중력이 센서의 `-Y`에 보이는 수직 장착에서는 결과가 대략 X축
`-90°` 회전이지만, 실제 기울어짐까지 정지 샘플로 계산하므로 각도를 코드에
고정하지 않습니다. 중력만으로는 yaw를 알 수 없기 때문에
`left.calibration.yaw_deg` 기본값은 `0.0`입니다. 로봇 정면과 IMU 정면이
다르면 실측한 yaw 장착각을 `compose.yaml`에 입력해야 합니다.

정지·수평 상태의 calibrated 출력은 다음 범위인지 확인합니다.

```text
angular_velocity.x/y/z  ≈ 0 rad/s
linear_acceleration.x/y ≈ 0 m/s²
linear_acceleration.z   ≈ +9.80665 m/s²
header.frame_id         = base_link
```

ROS `sensor_msgs/Imu.linear_acceleration`은 정지해도 중력을 포함하므로 Z가
0이 아니라 약 `+9.81`인 것이 정상입니다. 이동 가속도만 필요하면 보정
quaternion으로 중력 벡터를 같은 frame에 표현한 뒤 빼거나, `robot_localization`
같은 상태 추정기의 중력 제거 옵션을 사용하십시오. 단순히 항상 Z에서 9.81을
빼면 로봇이 기울었을 때 잘못된 값이 됩니다.

```python
import math

def quaternion_to_rpy(q):
    roll = math.atan2(2 * (q.w*q.x + q.y*q.z),
                      1 - 2 * (q.x*q.x + q.y*q.y))
    s = max(-1.0, min(1.0, 2 * (q.w*q.y - q.z*q.x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y),
                     1 - 2 * (q.y*q.y + q.z*q.z))
    return roll, pitch, yaw
```

원본 토픽은 센서 native axis를 그대로 유지합니다. 새 알고리즘에는
`/imu/left/data_calibrated`를 사용하고, 원본은 센서 고장·축 방향·재보정을
진단할 때 사용하십시오. 두 토픽 모두 정지 상태에서 가속도 norm이 약
9.81 m/s², quaternion norm이 약 1인지 확인하는 것이 기본 점검입니다.

### 5. LiDAR `/scan` 토픽

```bash
ros2 topic hz /scan
ros2 topic echo --once --qos-reliability best_effort \
  /scan sensor_msgs/msg/LaserScan
```

각 `ranges[i]`의 각도와 XY 좌표는 다음과 같습니다. `NaN`, `inf`,
`range_min` 미만, `range_max` 초과 값은 제거합니다.

```python
import numpy as np

def scan_to_xy(msg):
    ranges = np.asarray(msg.ranges, dtype=np.float32)
    angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
    valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
    x = ranges[valid] * np.cos(angles[valid])
    y = ranges[valid] * np.sin(angles[valid])
    return np.column_stack((x, y))  # laser_frame 기준, 단위 m
```

다른 센서나 맵과 합칠 때는 수동으로 좌표를 더하지 말고 메시지 timestamp의
`laser_frame -> map` 또는 `laser_frame -> odom` TF를 조회하여 변환합니다.

### 6. 2D 맵, pose와 TF

```bash
ros2 topic echo --once \
  --qos-reliability reliable --qos-durability transient_local \
  /map nav_msgs/msg/OccupancyGrid
ros2 topic echo --once /pose geometry_msgs/msg/PoseWithCovarianceStamped
ros2 run tf2_ros tf2_echo map laser_frame
```

`OccupancyGrid.data`는 행 우선(row-major) 1차원 배열입니다. `-1`은 unknown,
`0`은 free, `100`은 occupied이며 중간 값은 점유 확률입니다.

```python
import numpy as np

def occupancy_layers(msg):
    grid = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    unknown = grid < 0
    free = grid == 0
    occupied = grid >= 50
    return grid, unknown, free, occupied

# origin 회전이 0일 때 cell 중심의 map 좌표
world_x = msg.info.origin.position.x + (column + 0.5) * msg.info.resolution
world_y = msg.info.origin.position.y + (row + 0.5) * msg.info.resolution
```

origin quaternion에 회전이 있으면 위 XY에 그 회전을 추가 적용해야 합니다.
`/pose`와 센서 측정값을 결합할 때는 반드시 `header.stamp` 시각의 TF를
사용합니다. 수신 시각이나 가장 최근 TF를 무조건 사용하면 로봇이 움직일 때
지도 위 점이 밀립니다.

### 7. rclpy subscriber에서 QoS 설정하기

직접 Python 노드를 만들 때 sensor 토픽과 map 토픽의 QoS를 구분합니다.

```python
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, Imu, LaserScan

# Camera, Depth, IMU, LiDAR
image_sub = node.create_subscription(
    Image, "/image_left_raw", image_callback, qos_profile_sensor_data)
depth_sub = node.create_subscription(
    Image, "/StereoNetNode/stereonet_depth", depth_callback, qos_profile_sensor_data)
imu_sub = node.create_subscription(
    Imu, "/imu/left/data_calibrated", imu_callback, qos_profile_sensor_data)
scan_sub = node.create_subscription(
    LaserScan, "/scan", scan_callback, qos_profile_sensor_data)

# 마지막 맵을 즉시 받기 위한 transient-local QoS
map_qos = QoSProfile(depth=1)
map_qos.reliability = ReliabilityPolicy.RELIABLE
map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
map_sub = node.create_subscription(
    OccupancyGrid, "/map", map_callback, map_qos)
```

카메라와 Depth를 한 callback에서 사용해야 한다면
`message_filters.ApproximateTimeSynchronizer`를 사용하고 `header.stamp` 차이가
허용 범위 안인 프레임만 묶습니다. callback 안에서 오래 걸리는 추론이나 파일
저장을 직접 수행하지 말고 bounded queue와 worker를 사용해 DDS 수신을 막지
않도록 합니다.

### 8. Windows 호스트에서 WebSocket으로 받기

먼저 `host_view/start_dashboard.ps1`을 실행해 SSH 터널을 엽니다. 대역폭이 큰
영상이 다른 센서 연결을 막지 않도록 포트가 분리되어 있습니다.

- `ws://localhost:19092`: 표준 ROSBridge JSON으로 IMU, `/scan`, `/map`,
  `/pose`, `/tf`, `/tf_static`
- `ws://localhost:19093`: 카메라와 Depth 전용 raw binary WebSocket

```javascript
const telemetry = new WebSocket("ws://localhost:19092");

telemetry.addEventListener("open", () => {
  telemetry.send(JSON.stringify({
    op: "subscribe",
    id: "imu",
    topic: "/imu/left/data_calibrated",
    throttle_rate: 50,
    queue_length: 1,
  }));
});

telemetry.addEventListener("message", (event) => {
  const envelope = JSON.parse(event.data);
  if (envelope.op === "publish" && envelope.topic === "/imu/left/data_calibrated") {
    const imu = envelope.msg;
    console.log(imu.orientation, imu.angular_velocity, imu.linear_acceleration);
  }
});
```

영상 포트는 ROSBridge의 Image JSON/base64 변환 부하를 제거하기 위해 전용
binary protocol을 사용합니다. 기본 대시보드는 ROS 원본 주기를 측정하면서
최신 카메라와 Depth 프레임을 각각 정확한 목표 `2 Hz` 간격으로 보냅니다.
따라서 카드의 `ROS 9.8 · 표시 2.0 Hz`에서 앞 숫자는 RDK-X5의 실제 토픽
주기이고 뒤 숫자는 호스트 화면 전송 주기입니다.

```javascript
const images = new WebSocket("ws://localhost:19093");
images.binaryType = "arraybuffer";

images.addEventListener("open", () => {
  images.send(JSON.stringify({
    op: "subscribe",
    id: "left-camera",
    topic: "/image_left_raw",
    rate_hz: 2,
  }));
});

images.addEventListener("message", (event) => {
  if (typeof event.data === "string") return; // ready 메시지
  const view = new DataView(event.data);
  const headerLength = view.getUint32(0, true);
  const headerBytes = new Uint8Array(event.data, 4, headerLength);
  const header = JSON.parse(new TextDecoder().decode(headerBytes));
  const imageBytes = new Uint8Array(event.data, 4 + headerLength);
  console.log(header.topic, header.source_hz, header.encoding, imageBytes);
});
```

binary frame은 `little-endian uint32 header_length`, UTF-8 JSON header,
원본 Image payload 순서입니다. 허용 topic은 왼쪽/오른쪽 카메라와 Depth이며
`rate_hz`는 `0.5..4.0` 범위로 제한됩니다. 페이지나 프로그램을 종료하기
전에는 같은 `id`와 `topic`으로 `op: "unsubscribe"`를 보내고 WebSocket을
정상 종료합니다. 일반 ROSBridge Image 형식이 필요한 프로그램은 Foxglove
Bridge 또는 RDK-X5 내부의 ROS 2 subscriber를 사용하십시오.

대시보드 상단의 RDK-X5 CPU, RAM, 1분 load와 최대 온도는 같은 HTTP 터널의
`http://localhost:8080/api/resources`에서 2초마다 읽습니다. 이 endpoint는
JSON을 반환하며 브라우저 밖의 상태 모니터에서도 사용할 수 있습니다.

### 9. 토픽을 rosbag으로 저장하기

분석이나 재현이 필요하면 터미널 출력 대신 rosbag으로 원본을 저장합니다.

```bash
docker exec -it rplidar bash -lc '
  source /opt/ros/humble/setup.bash
  ros2 bag record -o /tmp/sensor_bag \
    /image_left_raw /image_right_raw \
    /StereoNetNode/stereonet_depth \
    /imu/left/data /imu/left/data_calibrated \
    /scan /map /pose /tf /tf_static
'
```

필요한 시간만 기록한 뒤 `Ctrl+C`로 정상 종료하고 호스트로 복사합니다.

```bash
docker cp rplidar:/tmp/sensor_bag ./sensor_bag
```

전체 토픽의 수신 주기, 카메라 payload 변화, 유효 Depth, IMU norm, 유효
LiDAR range와 맵 cell을 한 번에 점검하려면 아래의 기존 verifier를 사용합니다.

```bash
docker cp tools/verify_sensor_streams.py rplidar:/tmp/verify_sensor_streams.py
docker exec rplidar bash -lc \
  'source /opt/ros/humble/setup.bash; python3 /tmp/verify_sensor_streams.py --duration 12'
docker exec rplidar rm -f /tmp/verify_sensor_streams.py
```

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

호스트에서 바로 실행하는 최소 절차와 연결 오류 해결 방법은
[Windows 호스트 빠른 시작](README_HOST_WINDOWS.md)을 먼저 참고하십시오.

The `lidar` service also runs `slam_toolbox` and a loopback-only Foxglove
Bridge. Follow [host_view/README.md](host_view/README.md) to open an SSH tunnel,
connect to `ws://localhost:8765`, and view the camera, depth, left IMU, LiDAR,
and 2D occupancy map without installing ROS 2 on the host.

The map remains LiDAR scan-matching based. The host dashboard uses the calibrated
`base_link` IMU stream; SLAM fusion is still disabled until its filter parameters
and the LiDAR-to-base transform are validated together.

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
while `/data` contains the normalized quaternion. The original two topics retain
the native EBIMU axes. After 300 accepted stationary samples, the two
`*_calibrated` topics publish the measured mounting rotation, gyro-bias removal,
acceleration scale, and covariance in `base_link`. Calibration is repeated on
every IMU container start and never consumes UART3.
