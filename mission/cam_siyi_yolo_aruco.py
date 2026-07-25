#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SIYI RTSP 영상에서 mission phase에 맞는 YOLO 객체와 ArUco 마커를 탐지하는 ROS2 Lifecycle 노드.

발행 규칙:
  1. tray phase (source=1)에서는 tray YOLO 중심 좌표를 /yolo/center/detection 으로 발행
  2. vertiport_aruco phase (source=2)에서는 ArUco 중심 좌표 우선, 없으면 YOLO 발행

Point 메시지 사용:
  x: 중심 픽셀 u 좌표
  y: 중심 픽셀 v 좌표
  z: 소스 구분값 (0.0 = YOLO, 1.0 = ArUco)
"""
import gc
import os 
import yaml
import queue
import threading
from time import sleep

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

# Lifecycle 및 커스텀 메시지 임포트
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState
from sensor_msgs.msg import Image
from mission_msgs.msg import MissionState
from ultralytics import YOLO

class LatestFrameCapture:
    """카메라 프레임을 백그라운드에서 비동기적으로 계속 읽어 최신 프레임만 보관한다."""

    def __init__(self, source, logger, bufsize=2):
        self.logger = logger
        self.source = source
        self.bufsize = bufsize
        self.q = queue.Queue(maxsize=1)
        self._stop = False
        
        # "0"처럼 들어온 카메라 번호 문자열은 OpenCV 장치 번호로 변환한다.
        if isinstance(self.source, str) and self.source.isdigit():
            self.source = int(self.source)

        # 🌟 중요: 메인 쓰레드에서 VideoCapture를 열지 않고, 빈 객체만 만들어 둡니다.
        # 이렇게 하면 카메라가 연결 안 돼서 발생하는 30초 딜레이가 on_configure 단계에서 발생하지 않습니다.
        self.cap = cv2.VideoCapture()

        # 쓰레드 시작 (카메라 연결과 프레임 읽기를 백그라운드에서 비동기로 수행)
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        # 🌟 백그라운드 쓰레드 내에서 카메라 오픈을 시도합니다.
        self.logger.info(f"🔄 [LatestFrameCapture] 카메라 백그라운드 연결 시도 중: {self.source}")
        
        if isinstance(self.source, str) and self.source.startswith("rtsp://"):
            self.cap.open(
                self.source,
                cv2.CAP_FFMPEG,
                [
                    cv2.CAP_PROP_HW_ACCELERATION,
                    cv2.VIDEO_ACCELERATION_ANY,
                ],
            )
        else:
            self.cap.open(self.source)

        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.bufsize)
            self.logger.info("🟢 [LatestFrameCapture] 카메라 연결 성공!")
        else:
            self.logger.error(f"🔴 [LatestFrameCapture] 카메라/비디오 소스를 열 수 없습니다: {self.source}")

        # 프레임 읽기 루프
        while not self._stop:
            if not self.cap.isOpened():
                # 연결이 실패했거나 끊어졌다면 주기적으로 재연결 시도 가능 (안전 장치)
                sleep(1.0)
                continue

            ret, frame = self.cap.read()
            if not ret or frame is None:
                sleep(0.02)
                continue

            # 큐에는 항상 최신 프레임 하나만 남겨 추론 지연이 누적되지 않게 한다.
            if not self.q.empty():
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        try:
            
            frame = self.q.get_nowait()
            return True, frame
        except queue.Empty:
            return False, None

    def release(self):
        self._stop = True
        if self.cap.isOpened():
            self.cap.release()

class SiyiYoloArucoCenterNode(LifecycleNode):
    """FSM의 Intent Profile(YAML)을 참조하여 동작하는 라이프사이클 비전 노드."""

    YOLO_SOURCE = 0.0 
    ARUCO_SOURCE = 1.0 
    PHASE_TRAY = "tray_approach"
    PHASE_VERTIPORT_ARUCO = "vertiport_approach"

    # YAML source와 비전 모드 매핑
    VISION_PHASES = {
        PHASE_TRAY: {
            "model": "tray",
            "use_yolo": True,
            "use_aruco": False,  # ArUco 탐지 비활성화
        },
        PHASE_VERTIPORT_ARUCO: {
            "model": "vertiport",
            "use_yolo": True,
            "use_aruco": True,   # ArUco 탐지 활성화 (vertiport에서만)
        },
    }

    def __init__(self):
        super().__init__("yolo_aruco")

        # ---------- ROS 파라미터 선언 ----------
        self.declare_parameter("video_source", "rtsp://192.168.144.25:8554/main.264")
        self.declare_parameter('model_path_tray', '/home/kyh/Downloads/vision_detection_vertiport/runs/detect/train/weights/best.pt')
        self.declare_parameter('model_path_vertiport', '/home/kyh/Downloads/vision_detection_vertiport/runs/detect/train/weights/best.pt')
        self.declare_parameter("initial_model", "idle")
        self.declare_parameter("initial_phase", "idle")
        
        # FSM과 동일하게 intent_profiles_yaml 파일 경로를 받음
        self.declare_parameter("intent_profiles_yaml", "")

        self.declare_parameter("center_topic", '/yolo/filtered_center')
        self.declare_parameter("debug_image_topic", "detection_image")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("draw_debug", True)

        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("aruco_marker_id", 23)
        self.declare_parameter("aruco_min_area", 100.0)

        # 멤버 변수 초기화
        self.bridge = None
        self.center_pub = None
        self.image_pub = None
        self.sub_mission_state = None
        self.timer = None
        self.cap = None
        
        self.model_paths = {}
        self.current_phase = None
        self.current_model_name = None
        self.current_model = None
        self.loaded_model_name = None
        self.aruco_detector = None
        self.intent_profiles = {}

    def load_yaml(self, path: str):
        """fsmfms.py 방식과 동일하게 YAML 파싱"""
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                raw_yaml = yaml.safe_load(f) or {}
                if '/**' in raw_yaml and 'ros__parameters' in raw_yaml['/**']:
                    return raw_yaml['/**']['ros__parameters']
                return raw_yaml
        self.get_logger().warn(f"⚠️ YAML 파일을 찾을 수 없습니다: {path} (기본값 작동)")
        return {}

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() 호출됨: 노드 구성 중...")

        # 1. YAML 로드
        ip_path = self.get_parameter("intent_profiles_yaml").value
        self.intent_profiles = self.load_yaml(ip_path)

        video_source = self.get_parameter("video_source").value
        center_topic = self.get_parameter("center_topic").value
        debug_image_topic = self.get_parameter("debug_image_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.draw_debug = bool(self.get_parameter("draw_debug").value)
        self.aruco_marker_id = int(self.get_parameter("aruco_marker_id").value)
        self.aruco_min_area = float(self.get_parameter("aruco_min_area").value)

        # 2. Pub/Sub 생성
        self.bridge = CvBridge()
        self.center_pub = self.create_lifecycle_publisher(Point, center_topic, 10)
        self.image_pub = self.create_lifecycle_publisher(Image, debug_image_topic, 10)
        
        # String 대신 커스텀 메시지(MissionState) 구독
        self.sub_mission_state = self.create_subscription(MissionState, "/mission/state", self.mission_state_callback, 10)

        # 3. 모델 경로 및 초기 페이즈
        self.model_paths = {
            "tray": self.get_parameter("model_path_tray").value,
            "vertiport": self.get_parameter("model_path_vertiport").value,
        }
        
        # 4. ArUco 및 캡처 준비
        self.aruco_detector = self.create_aruco_detector()
        self.cap = LatestFrameCapture(video_source, self.get_logger())
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("🟢 노드 활성화됨 (Mode 1): 추론 타이머 가동")
        self.center_pub.on_activate(state)
        self.image_pub.on_activate(state)
        
        self.timer = self.create_timer(1.0 / max(1.0, self.publish_rate), self.detection_loop)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("🔴 노드 비활성화됨 (Mode 0): 추론 중지")
        self.center_pub.on_deactivate(state)
        self.image_pub.on_deactivate(state)

        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
            
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup() 호출됨: 모든 자원 정리 중...")
        
        if self.timer is not None:
            try:
                self.destroy_timer(self.timer)
            except Exception:
                pass
            self.timer = None

        if self.cap is not None:
            self.cap.release()
            self.cap = None

        self.unload_current_yolo_model()

        if self.center_pub: 
            self.destroy_publisher(self.center_pub)
            self.center_pub = None
        if self.image_pub: 
            self.destroy_publisher(self.image_pub)
            self.image_pub = None
        if self.sub_mission_state: 
            self.destroy_subscription(self.sub_mission_state)
            self.sub_mission_state = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() 호출됨: 모든 자원 강제 해제 중...")
        if self.timer is not None:
            try:
                self.destroy_timer(self.timer)
            except Exception:
                pass
            self.timer = None

        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
            
        self.unload_current_yolo_model()
        return TransitionCallbackReturn.SUCCESS

    def create_aruco_detector(self):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco 모듈이 없습니다.")
        dictionary_name = str(self.get_parameter("aruco_dictionary").value)
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(cv2.aruco, "DetectorParameters"):
            parameters = cv2.aruco.DetectorParameters()
        else:
            parameters = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2, "aruco") and hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            return detector.detectMarkers

        def detect_markers(gray):
            return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        return detect_markers

    def detection_loop(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("⚠️ 카메라 프레임을 읽어올 수 없습니다.", throttle_duration_sec=2.0)
            return

        # 🌟 current_phase가 None일 때의 KeyError 예외 방지 (Idle 상태 대처)
        if self.current_phase is None:
            self.get_logger().warn("⚠️ 설정된 Vision Phase가 없습니다. (Idle 상태 무시)", throttle_duration_sec=5.0)
            return

        annotated = frame.copy()
        phase = self.VISION_PHASES.get(self.current_phase)
        if not phase:
            return
        
        yolo_target, aruco_target, all_corners, all_ids = None, None, None, None

        # ArUco 탐지 (Vertiport Phase 에서만 활성화 됨)
        if phase["use_aruco"]:
            aruco_target, all_corners, all_ids = self.detect_aruco(frame)

        if phase["use_yolo"] and aruco_target is None:
            yolo_target = self.detect_yolo(frame)

        if self.draw_debug:
            self.draw_debug_overlay(annotated, yolo_target, aruco_target, all_corners, all_ids)

        if aruco_target is not None:
            cx, cy, marker_id, area = aruco_target
            self.publish_center(cx, cy, self.ARUCO_SOURCE)
        elif yolo_target is not None:
            x1, y1, x2, y2, score = yolo_target
            cx, cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
            self.publish_center(cx, cy, self.YOLO_SOURCE)

        self.publish_debug_image(annotated)

    def detect_yolo(self, frame):
        model = self.get_current_yolo_model()
        if model is None: return None

        result = model(frame, verbose=False)[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0: return None

        best_target = None
        best_score = self.conf_threshold
        for box in boxes:
            score = float(box.conf[0].item())
            if score < best_score: continue
            x1, y1, x2, y2 = box.xyxy[0].detach().cpu().tolist()
            best_target = (x1, y1, x2, y2, score)
            best_score = score
        return best_target

    def detect_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector(gray)

        if ids is None or len(ids) == 0:
            return None, corners, ids

        selected = None
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            marker_id = int(marker_id)
            if self.aruco_marker_id >= 0 and marker_id != self.aruco_marker_id:
                continue

            pts = marker_corners.reshape(4, 2)
            area = float(cv2.contourArea(pts.astype(np.float32)))
            if area < self.aruco_min_area: continue

            center = pts.mean(axis=0)
            candidate = (float(center[0]), float(center[1]), marker_id, area)

            if selected is None or candidate[3] > selected[3]:
                selected = candidate
        return selected, corners, ids

    def publish_center(self, cx, cy, source):
        msg = Point()
        msg.x, msg.y, msg.z = float(cx), float(cy), float(source)
        self.center_pub.publish(msg)

    def publish_debug_image(self, frame):
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"⚠️ 이미지 발행 실패: {exc}")
            

    def draw_debug_overlay(self, frame, yolo_target, aruco_target, all_corners, all_ids):
        if yolo_target is not None:
            x1, y1, x2, y2, score = yolo_target
            ix1, iy1, ix2, iy2 = map(int, [x1, y1, x2, y2])
            cv2.rectangle(frame, (ix1, iy1), (ix2, iy2), (0, 255, 0), 2)
            cv2.putText(frame, f"YOLO {score:.2f}", (ix1, max(20, iy1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if all_ids is not None and len(all_ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, all_corners, all_ids)

        if aruco_target is not None:
            cx, cy, marker_id, _ = aruco_target
            cv2.circle(frame, (int(cx), int(cy)), 7, (0, 0, 255), -1)
            cv2.putText(frame, f"ARUCO id={marker_id}", (int(cx) + 10, max(20, int(cy) - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    def set_vision_phase(self, phase_name):
        phase = self.VISION_PHASES.get(phase_name)
        if not phase: return False
        
        model_name = phase["model"]
        if model_name not in self.model_paths: return False

        if phase_name == self.current_phase: return True

        if self.current_model_name != model_name:
            self.unload_current_yolo_model()

        self.current_phase = phase_name
        self.current_model_name = model_name
        
        self.get_logger().info(f"🚀 Vision 전환: {phase_name} (yolo={phase['use_yolo']}, aruco={phase['use_aruco']})")
        return True

    def get_current_yolo_model(self):
        if self.current_model_name is None: return None
        if self.current_model is not None and self.loaded_model_name == self.current_model_name:
            return self.current_model

        path = self.model_paths.get(self.current_model_name)
        self.unload_current_yolo_model()
        
        self.get_logger().info(f"📦 YOLO 모델 메모리에 로드 중: {self.current_model_name}")
        self.current_model = YOLO(path)
        self.loaded_model_name = self.current_model_name
        return self.current_model

    def unload_current_yolo_model(self):
        if self.current_model is None: return
        self.get_logger().info(f"🧹 YOLO 모델 메모리 반환: {self.loaded_model_name}")
        self.current_model = None
        self.loaded_model_name = None
        gc.collect()
        try:
            import torch
            if torch.cuda.is_available(): torch.cuda.empty_cache()
        except:
            pass

    def mission_state_callback(self, msg: MissionState):
        """FSM에서 발행하는 MissionState의 intent_profile과 YAML 파라미터를 결합하여 source 결정"""
        intent_name = msg.intent_profile
        
        # YAML에서 해당 프로필의 데이터를 조회
        profile_data = self.intent_profiles.get(intent_name, {})
        yolo_params = profile_data.get("yolo_aruco", {})
        
        # source 및 mode 값 확인 (기본값 0)
        source = yolo_params.get("source", 0)
        mode = yolo_params.get("mode", 0)
        
        # 🌟 mode와 source 조건 매핑 연동
        if mode == 1 and source == 1:
            self.set_vision_phase(self.PHASE_TRAY)
        elif mode == 1 and source == 2:
            self.set_vision_phase(self.PHASE_VERTIPORT_ARUCO)
        else:
            # 🌟 source가 0이거나 mode가 0일 때 (Idle 상태) -> 모델 언로드하여 자원 대폭 절약
            if self.current_phase is not None:
                self.get_logger().info("💤 비전 탐지 비활성화 (Idle 페이즈 전환 및 YOLO 언로드)")
                self.unload_current_yolo_model()
                self.current_phase = None
                self.current_model_name = None


def main(args=None):
    rclpy.init(args=args)
    node = SiyiYoloArucoCenterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.get_logger().info("Shutting down...")
        if hasattr(node, 'cap') and node.cap is not None:
            try:
                node.cap.release()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()