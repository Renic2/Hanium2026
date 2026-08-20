# Windows 호스트에서 센서 대시보드 바로 실행하기

이 문서는 Windows 호스트 PC에서 ROS 2나 Docker를 설치하지 않고 RDK-X5의
카메라, Depth, 보정 IMU, LiDAR와 LiDAR 기반 2D 맵을 한 화면에서 보는 가장
짧은 실행 절차입니다.

대시보드는 RDK-X5 내부의 loopback 주소에만 열려 있으므로
`http://192.168.55.151:8080`으로 직접 접속하지 않습니다. 제공된 PowerShell
스크립트로 SSH 터널을 연 뒤 호스트의 `localhost`로 접속합니다.

## 1. 준비 사항

- Windows 호스트와 RDK-X5가 같은 네트워크에 연결되어 있어야 합니다.
- 프로젝트가 호스트에서 `R:\robot_sensors`로 보여야 합니다.
- SSH 개인 키의 기본 예시 경로는 `R:\.ssh\id_ed25519`입니다.
- Windows OpenSSH Client가 필요합니다. PowerShell에서 다음 명령이 버전을
  출력하면 설치된 상태입니다.

```powershell
ssh -V
```

로봇의 현재 기본 접속 정보는 다음과 같습니다.

```text
IP:   192.168.55.151
User: hanium
```

## 2. 가장 빠른 실행

Windows PowerShell을 열고 아래 명령을 그대로 실행합니다. 관리자 권한은
필요하지 않습니다.

```powershell
Set-Location R:\robot_sensors
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\host_view\start_dashboard.ps1 `
  -IdentityFile R:\.ssh\id_ed25519
```

정상적으로 연결되면 다음과 비슷한 안내가 출력됩니다.

```text
Opening the secure sensor-dashboard tunnel to hanium@192.168.55.151.
Keep this window open and browse to http://localhost:8080/...
```

이 PowerShell 창을 닫지 말고 Chrome 또는 Edge에서 다음 주소를 엽니다.

<http://localhost:8080>

화면에는 다음 데이터가 자동으로 표시됩니다.

- 180도 회전된 왼쪽 카메라 영상
- 180도 회전된 컬러 Depth 영상
- `/imu/left/data_calibrated` 기반 roll, pitch, yaw, 각속도, 가속도
- `/scan`과 `/map`을 이용한 LiDAR 기반 2D 매핑 화면

별도의 Foxglove 패널 설정이나 호스트 ROS 2 설치는 필요하지 않습니다.

## 3. 실행 중 지켜야 할 사항

- 터널을 실행한 PowerShell 창은 대시보드를 보는 동안 계속 열어 둡니다.
- 터널 종료는 해당 창에서 `Ctrl+C`를 누릅니다.
- 호스트 대시보드를 여는 것만으로 IMU가 재시작되지는 않습니다.
- RDK-X5의 `imu` 컨테이너를 재시작한 경우에만 로봇을 평평한 곳에 놓고
  약 3초 동안 움직이지 않아야 합니다.
- 보정 IMU가 정상일 때 정지 가속도는 대략 `x=0`, `y=0`,
  `z=+9.81 m/s²`로 표시됩니다. Z가 0이 아닌 것은 중력이 포함되기 때문입니다.

## 4. 로봇 IP 또는 포트가 다를 때

로봇 IP가 바뀌었다면 `-RobotHost`를 지정합니다.

```powershell
Set-Location R:\robot_sensors
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\host_view\start_dashboard.ps1 `
  -RobotHost 192.168.0.8 `
  -IdentityFile R:\.ssh\id_ed25519
```

호스트의 `8080`, `19092`, `19093` 포트가 이미 사용 중이면 다른 포트를
지정합니다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\host_view\start_dashboard.ps1 `
  -IdentityFile R:\.ssh\id_ed25519 `
  -DashboardPort 18080 `
  -RosbridgePort 19192 `
  -ImageBridgePort 19193
```

이 경우 스크립트가 출력하는 전체 URL을 그대로 엽니다. 위 예시의 URL은
다음과 같습니다.

<http://localhost:18080/?rosbridgePort=19192&imageBridgePort=19193>

SSH agent에 개인 키가 이미 등록되어 있다면 `-IdentityFile`을 생략할 수
있습니다. 네트워크 드라이브에 있는 키를 지정하면 스크립트가 Windows
OpenSSH용 임시 사본에 안전한 ACL을 적용하고, 터널 종료 시 자동 삭제합니다.

## 5. 연결 상태 빠르게 확인하기

터널 PowerShell 창을 연 상태에서 새 PowerShell 창을 하나 더 열고 다음을
확인합니다.

```powershell
# RDK-X5의 SSH 포트
Test-NetConnection 192.168.55.151 -Port 22

# 호스트로 전달된 웹/ROSBridge 포트
Test-NetConnection localhost -Port 8080
Test-NetConnection localhost -Port 19092
Test-NetConnection localhost -Port 19093
```

네 결과 모두 `TcpTestSucceeded : True`가 권장 상태입니다. 브라우저에서 이전
JavaScript가 캐시되어 있으면 `Ctrl+F5`로 강력 새로고침합니다.

## 6. 자주 발생하는 오류

### `localhost에서 연결을 거부했습니다`

터널 PowerShell 창이 닫혔거나 SSH 접속이 실패한 상태입니다.

1. `Test-NetConnection 192.168.55.151 -Port 22`를 실행합니다.
2. 성공하면 2절의 `start_dashboard.ps1` 명령을 다시 실행합니다.
3. 터널 창을 열린 상태로 유지하고 <http://localhost:8080>을 다시 엽니다.

### 화면 상단에 `ROSBridge 연결 중`만 계속 표시됨

```powershell
Test-NetConnection localhost -Port 19092
Test-NetConnection localhost -Port 19093
```

- `19092` 실패: IMU, LiDAR, 맵용 telemetry 터널이 열리지 않은 상태입니다.
- `19093` 실패: 카메라와 Depth용 image 터널이 열리지 않은 상태입니다.
- 두 포트가 모두 실패하면 터널을 `Ctrl+C`로 종료한 뒤 다시 실행합니다.
- 포트 사용 중 오류가 나오면 4절의 대체 포트 명령을 사용합니다.

### `Permission denied (publickey)`

`-IdentityFile` 경로가 실제 키 파일을 가리키는지 확인합니다.

```powershell
Test-Path R:\.ssh\id_ed25519
```

결과가 `False`이면 실제 개인 키의 절대 경로로 바꿉니다. 키가 SSHFS나
네트워크 드라이브에 있어도 `start_dashboard.ps1`이 권한이 제한된 임시
사본을 사용하므로, `ssh` 명령을 직접 실행하는 것보다 스크립트 사용을
권장합니다.

### 웹 화면은 열리지만 IMU만 `대기 중`

RDK-X5에서 IMU 컨테이너가 막 시작되었다면 보정 완료까지 약 3초가 필요합니다.
로봇을 평평한 바닥에서 완전히 정지시키고 5초 정도 기다린 뒤 브라우저를
새로고침합니다. 대시보드는 보정된 `/imu/left/data_calibrated`만 사용합니다.

### 카메라 또는 Depth만 나오지 않음

`localhost:19093` 연결을 확인하고 `Ctrl+F5`로 새로고침합니다. 영상은
텔레메트리와 다른 ROSBridge 포트를 사용하므로 IMU와 맵이 보여도 영상 포트만
끊어질 수 있습니다.

## 7. Foxglove를 사용하고 싶을 때

고정 웹 대시보드 대신 Foxglove 패널을 직접 구성하려면
[host_view/README.md](host_view/README.md)를 참고합니다. 이 경우
`host_view/start_tunnel.ps1`을 실행하고 Foxglove WebSocket 주소
`ws://localhost:8765`에 연결합니다.

각 ROS 2 토픽의 메시지 타입, 단위, QoS와 처리 예제는
[루트 README의 센서 토픽 가져오기와 처리 방법](README.md#센서-토픽-가져오기와-처리-방법)에
정리되어 있습니다.
