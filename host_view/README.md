# 호스트 PC에서 센서와 2D 맵 보기

고정 웹 대시보드만 바로 실행하려면 별도 문서인
[Windows 호스트 빠른 시작](../README_HOST_WINDOWS.md)을 참고하십시오.

RDK-X5의 Foxglove Bridge는 보안을 위해 로봇 내부의
`127.0.0.1:8765`에만 열립니다. Windows 호스트에서 SSH 터널을 연 뒤
Foxglove WebSocket으로 접속하면 ROS 2를 호스트에 설치하지 않고도 볼 수
있습니다.

## 가장 쉬운 방법: 고정 웹 대시보드

Foxglove 패널을 직접 만들 필요가 없습니다. Windows PowerShell에서 다음
명령을 실행하고 창을 계속 열어 둡니다.

```powershell
Set-Location R:\robot_sensors
Set-ExecutionPolicy -Scope Process Bypass
.\host_view\start_dashboard.ps1 -IdentityFile R:\.ssh\id_ed25519
```

그다음 Chrome에서 <http://localhost:8080>을 엽니다. 카메라, 컬러 Depth,
UART2 IMU 그래프, LiDAR 2D 맵이 한 화면에 자동으로 나타납니다.
스크립트는 텔레메트리용 `19092`와 영상용 `19093`을 서로 분리해 전달합니다.
카메라/Depth 직렬화가 IMU·LiDAR·맵 연결을 막지 않으며, 다른 개발 도구가
흔히 사용하는 호스트 `9090`과도 충돌하지 않습니다.

각 ROS 2 토픽의 메시지 타입, QoS, 필드 단위, Python/OpenCV 처리 예제와
ROSBridge 구독 JSON은 루트 [README의 센서 토픽 가져오기와 처리 방법](../README.md#센서-토픽-가져오기와-처리-방법)을
참고하십시오.

아래 Foxglove 절차는 패널을 직접 조정하고 싶을 때만 사용합니다.

## 1. 로봇의 네 서비스를 시작

RDK-X5에서 실행합니다.

```bash
cd /home/hanium/robot_sensors
docker compose up -d
docker compose ps
```

`lidar`, `camera`, `imu`, `depth` 네 서비스가 모두 `healthy`여야 합니다.

## 2. Windows 호스트에서 SSH 터널 열기

PowerShell에서 실행하고 이 창을 계속 열어 둡니다.

```powershell
Set-Location R:\robot_sensors
.\host_view\start_tunnel.ps1
```

로봇의 Wi-Fi 주소로 직접 접속할 때는 다음처럼 지정할 수 있습니다.

```powershell
.\host_view\start_tunnel.ps1 -RobotHost 192.168.0.8
```

SSH 개인 키를 명시해야 하는 환경이라면 `-IdentityFile`을 추가합니다.
네트워크 드라이브의 키도 스크립트가 소유자 전용 임시 사본으로 안전하게
처리하고 터널 종료 시 삭제합니다.

```powershell
.\host_view\start_tunnel.ps1 -IdentityFile C:\path\to\id_ed25519
```

실행 정책 때문에 `.ps1`이 차단된 경우 현재 PowerShell 프로세스에만
허용한 뒤 다시 실행합니다.

```powershell
Set-ExecutionPolicy -Scope Process Bypass
```

## 3. Foxglove에 연결

1. Foxglove Desktop 또는 <https://app.foxglove.dev>를 엽니다.
2. 시작 화면의 `Open connection`을 누릅니다.
3. 연결 형식에서 `Foxglove WebSocket`을 선택합니다.
4. URL에 `ws://localhost:8765`를 입력하고 `Open`을 누릅니다.
5. 왼쪽 `Topics` 목록에 `/image_left_raw`, `/imu/left/data_calibrated`, `/scan`,
   `/map`이 보이면 연결된 것입니다.

이제 아래 순서대로 패널을 하나씩 추가합니다. 패널 설정은 패널 오른쪽 위의
톱니바퀴를 누르거나 패널을 선택한 뒤 `,` 키를 누르면 열립니다.

## 4. 왼쪽 카메라 패널

1. 화면 상단의 `Add panel`을 누릅니다.
2. 패널 목록에서 `Image`를 누릅니다.
3. 새 Image 패널의 톱니바퀴를 누릅니다.
4. 왼쪽 설정의 `General` -> `Topic`에서 `/image_left_raw`를 선택합니다.
5. 영상이 너무 확대되었으면 Image 패널 위에 마우스를 놓고 `1` 키를 눌러
   화면 맞춤 상태로 되돌립니다.

오른쪽 카메라도 필요하면 Image 패널을 하나 더 추가하고 Topic만
`/image_right_raw`로 선택합니다.

## 5. Depth 패널

1. `Add panel` -> `Image`를 한 번 더 누릅니다.
2. 톱니바퀴 -> `General` -> `Topic`에서
   `/StereoNetNode/stereonet_depth`를 선택합니다.
3. 같은 설정에서 `Value min`을 `150`, `Value max`를 `4000` 정도로
   지정합니다. 가까운 물체와 먼 물체의 명암 차이가 더 잘 보입니다.
4. Depth 영상의 픽셀 값 단위는 mm입니다. 예를 들어 `500`은 0.5 m입니다.

## 6. IMU 그래프 패널

IMU는 각속도와 가속도를 두 개의 Plot 패널로 나누면 보기 쉽습니다.

### 각속도

1. `Add panel` -> `Plot`을 누릅니다.
2. 톱니바퀴를 열고 X-axis가 `Timestamp`인지 확인합니다.
3. `Series`에서 `Add series` 또는 `+` 버튼을 눌러 아래 Y-value를 하나씩
   추가합니다.

```text
/imu/left/data_calibrated.angular_velocity.x
/imu/left/data_calibrated.angular_velocity.y
/imu/left/data_calibrated.angular_velocity.z
```

### 가속도

1. `Add panel` -> `Plot`을 다시 누릅니다.
2. 아래 세 개의 Y-value를 추가합니다.

```text
/imu/left/data_calibrated.linear_acceleration.x
/imu/left/data_calibrated.linear_acceleration.y
/imu/left/data_calibrated.linear_acceleration.z
```

자세를 roll, pitch, yaw 그래프로 보고 싶으면 별도 Plot 패널에 다음 값을
추가할 수 있습니다.

```text
/imu/left/data_calibrated.orientation.@rpy.roll
/imu/left/data_calibrated.orientation.@rpy.pitch
/imu/left/data_calibrated.orientation.@rpy.yaw
```

중요: `/imu/left/data_calibrated`까지가 토픽 이름이고, 그 뒤의 필드는 `/`가 아니라
`.`으로 연결합니다.

## 7. LiDAR 기반 2D 맵 패널

1. `Add panel` -> `3D`를 누릅니다.
2. 3D 패널의 톱니바퀴를 엽니다.
3. `Frame` 설정에서 다음과 같이 선택합니다.
   - `Fixed frame`: `map`
   - `Display frame`: `map`
   - `Follow mode`: `Off`
4. `Topics` 항목을 펼칩니다.
5. `/map` 오른쪽의 눈 아이콘을 켭니다.
6. `/scan` 오른쪽의 눈 아이콘도 켭니다.
7. `Scene` 또는 카메라 설정에서 `3D view`를 끄거나 `2D`로 전환합니다.
   그러면 map 프레임의 XY 평면을 위에서 내려다보게 됩니다.
8. 마우스 휠로 확대/축소하고 드래그로 지도의 중심을 옮깁니다.

`/map`이 회색·검은 격자로 표시되고 `/scan` 점들이 그 위에 보이면 정상입니다.
`/tf`와 `/tf_static`은 좌표 변환에 자동 사용되므로 별도 Plot 패널을 만들
필요가 없습니다.

## 8. 패널 배치와 저장

- 패널 제목 표시줄을 드래그하면 위치를 바꿀 수 있습니다.
- 패널 메뉴의 `Split`을 이용하면 화면을 좌우 또는 위아래로 나눌 수 있습니다.
- 계정에 Layout 편집 기능이 보이면 상단 `Layouts` 메뉴에서 현재 구성을
  저장할 수 있습니다.

권장 배치는 위쪽에 카메라와 Depth, 아래쪽에 IMU Plot 두 개와 2D Map을
두는 형태입니다.

## 9. 화면이 나오지 않을 때

- 왼쪽 `Topics`가 비어 있음: SSH 터널 PowerShell 창이 열려 있는지 확인합니다.
- `/map`이 보이지 않음: 연결 후 2~3초 기다린 뒤 Frame을 다시 `map`으로
  선택합니다.
- 맵이 처음 크기에서 커지지 않음: 로봇과 LiDAR가 같이 움직이도록 하고,
  로봇을 천천히 이동시킵니다.
- Depth가 거의 검게 보임: `Value min=150`, `Value max=4000`을 확인합니다.
- `localhost:8765` 포트가 사용 중임: 터널을
  `-LocalPort 8766`으로 실행하고 Foxglove URL도
  `ws://localhost:8766`으로 바꿉니다.

Depth는 `mono16` 밀리미터 영상이며 카메라는 `nv12`입니다. 오른쪽 카메라가
필요하면 `/image_right_raw`도 별도 `Image` 패널에 추가할 수 있습니다.

현재 2D SLAM은 휠 오도메트리 없이 LiDAR 스캔 매칭으로 동작합니다. 로봇을
천천히 움직이고 급회전을 피해야 지도가 안정적으로 확장됩니다. IMU 패널은
시작 시 3초간 정지 샘플로 장착축과 gyro bias가 보정된 `base_link` 토픽을
표시합니다. SLAM에는 아직 IMU를 융합하지 않았습니다.

터널을 닫으려면 터널 PowerShell 창에서 `Ctrl+C`를 누릅니다.
