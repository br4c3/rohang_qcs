# ARECADA Ground Control Station

`qcs.py`의 PyQt5 관제 화면을 Vanilla JavaScript 웹 대시보드로 옮긴 프로젝트입니다.

## Electron 앱 실행

```bash
npm install
npm start
```

Electron 앱이 자체 로컬 서버를 임의 포트에 실행하므로 별도의 웹 서버는 필요하지 않습니다.
Swagger UI도 앱 상단의 `API DOCS`에서 열 수 있습니다.

## PX4 SITL + Gazebo 연동

터미널 1에서 지면 방향 단안 카메라가 장착된 PX4 SITL 기체, Gazebo와
Micro XRCE-DDS Agent를 실행합니다.

```bash
chmod +x scripts/start_sitl.sh
./scripts/start_sitl.sh
```

터미널 2에서 Electron 앱을 실행합니다.

```bash
npm start
```

`/fmu/out/vehicle_status_v4`, `vehicle_local_position_v1`,
`airspeed_validated_v1`, `vehicle_gps_position`, `battery_status_v1`
토픽이 들어오면 상단 연결 상태가
`SITL ONLINE`으로 바뀌며 실제 시뮬레이션 값이 표시됩니다.

이전 PX4의 접미사 없는 토픽 이름도 브리지에서 함께 지원합니다.

Gazebo의 `/sensor/camera/image` 토픽은 별도의 바이너리 카메라 브리지가
JPEG로 압축해 Electron CAMERA 패널에 최대 12 FPS로 전달합니다. 다른 영상
토픽을 사용할 때는 `GZ_CAMERA_TOPIC` 환경 변수로 지정할 수 있습니다.

## QGroundControl 연동

Electron의 `QGC MAP` 탭을 선택하면 QGroundControl AppImage를 자동 실행합니다.
PX4 SITL이 기본 MAVLink UDP 포트 `14550`으로 전송하므로 QGC가 기체를 자동으로
검색하고 연결합니다.

Plan 패널에서는 QGC `.plan` 파일을 선택해 전체 웨이포인트를 검증·미리보기하고,
MAVROS를 통해 PX4에 업로드한 뒤 `ARM + START`로 `AUTO.MISSION`을 시작할 수
있습니다. Plan 파싱과 마지막 호버 지점 검증에는 수정하지 않은
`mission/config.py`의 `load_qgc_plan()`과 `validate_hover_plan()`을 사용합니다.

기본 QGC 경로는 `/home/br4c3/apps/QGroundControl.AppImage`입니다. 다른 경로를
사용할 때는 다음처럼 지정합니다.

```bash
QGC_PATH=/path/to/QGroundControl.AppImage npm start
```

## 실제 기체: Jetson 원격 제어 구성

비전, FSM, Offboard setpoint와 MAVROS 미션 업로드는 Jetson에서 실행하고,
Electron은 지상 컴퓨터에서 시각화와 고수준 명령만 담당할 수 있습니다.
Jetson에는 이 저장소의 `mission/`과 `bridge/` 디렉터리가 모두 있어야 합니다.

Jetson에서 ROS 2 및 기체 workspace를 먼저 source한 다음 게이트웨이를
실행합니다. 토큰은 HM30 링크에서 비행 명령 API를 보호하므로 필수입니다.

```bash
source /opt/ros/humble/setup.bash
source /path/to/vehicle_ws/install/setup.bash
export JETSON_GCS_TOKEN='replace-with-a-long-random-token'
./scripts/start_jetson_gateway.sh
```

기본 수신 주소는 `0.0.0.0:8765`입니다. 필요하면
`JETSON_GCS_HOST`와 `JETSON_GCS_PORT`로 변경할 수 있습니다.

지상 컴퓨터에서는 HM30에서 접근할 수 있는 Jetson IP와 동일한 토큰을
지정합니다.

```bash
export JETSON_GCS_URL='http://192.168.1.20:8765'
export JETSON_GCS_TOKEN='replace-with-a-long-random-token'
npm start
```

`JETSON_GCS_URL`이 설정되면 Electron은 로컬 PX4 브리지, MAVROS 및 Gazebo
카메라 브리지를 실행하지 않습니다. 텔레메트리는 Jetson의 `/status` API에서
가져오며 Plan 검증·업로드·ARM·AUTO.MISSION 시작도 Jetson에서 수행됩니다.
환경변수가 없으면 기존 PX4 SITL 로컬 모드로 동작합니다.

연결 확인:

```bash
curl \
  -H "Authorization: Bearer $JETSON_GCS_TOKEN" \
  "$JETSON_GCS_URL/health"
```

실제 SIYI 영상 전송은 이 API에 포함되지 않습니다. 비압축 ROS Image 대신
RTSP 또는 GStreamer H.264/H.265 스트림을 별도로 사용하는 것을 권장합니다.

기본 PX4 위치가 다른 경우 다음처럼 지정할 수 있습니다.

```bash
PX4_AUTOPILOT_DIR=/path/to/PX4-Autopilot ./scripts/start_sitl.sh
```

## 설치 파일 빌드

```bash
npm run dist
```

현재 운영체제에 맞는 설치 파일이 `dist/`에 생성됩니다.

## 브라우저에서 실행

별도 빌드나 패키지 설치 없이 실행할 수 있습니다.

```bash
python3 -m http.server 8000
```

브라우저에서 `http://localhost:8000`을 열면 됩니다.

Swagger API 문서는 `http://localhost:8000/swagger.html`에서 확인할 수 있습니다.

## 현재 동작

- CAMERA / QGC MAP 화면 전환
- PX4 비행 모드와 VTOL 상태 제어
- QGC Plan 검증, 업로드, ARM 및 AUTO.MISSION 실행
- OpenStreetMap 기반 임무 경로와 기체 헤딩 표시
- Gazebo 하향 카메라 실시간 영상
- PX4 uORB 기반 GPS, 모터 출력, 고도, 대기속도
- IMU, EKF, 목표/실제 위치 및 GPS/EKF 궤적 분석
- PX4 타임스탬프 기반 추정 상태 로그
- OpenAPI 3.0 명세와 Swagger UI
- Electron 데스크톱 창과 반응형 레이아웃

Electron 메인 프로세스가 ROS 2 DDS와 Gazebo Transport 브리지를 실행하고,
렌더러에는 실제 PX4 및 카메라 데이터만 전달합니다. PX4 연결이 없으면 수치는
임의 데이터 대신 `—`로 표시됩니다.
