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
