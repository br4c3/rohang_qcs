#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from enum import Enum, auto

from std_msgs.msg import Empty
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleStatus,
)
from mission_msgs.msg import ControlTick

# Vehicle_command topic QOS 맞추기 위해서 임의로 하나 만듬
QOS_VEHICLE_DEFAULT = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)

# QoS 프로파일 정의
QOS_DEFAULT = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

# 상태 Enum 정의
class Phase(Enum):
    IDLE = auto()
    WAIT_FOR_MISSION_END = auto()
    TRAY = auto()
    SAFEPOINT = auto()
    VERTIPORT = auto()

class Task(Enum):
    IDLE = auto()
    MOVE = auto()
    APPROACH = auto()
    GRIPPER = auto()
    LAND = auto()

# PX4 상수 정의
class PX4:
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_NAV_LAND = 21
    PX4_CUSTOM_MAIN_MODE = 1
    PX4_OFFBOARD_SUB_MODE = 6
    ARM_COMMAND = 1.0
    DISARM_COMMAND = 0.0


    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    VEHICLE_CMD_DO_SET_MODE          = 176
    VEHICLE_CMD_NAV_LAND             = 21

    # DO_SET_MODE 파라미터
    MODE_CUSTOM                      = 1.0   # param1: 'custom mode 사용' 플래그
    MAIN_MODE_OFFBOARD               = 6.0   # param2: PX4 main mode (OFFBOARD)

    ARM_COMMAND   = 1.0
    DISARM_COMMAND= 0.0

class FSM(Node):
    """
    드론의 미션을 관리하는 FSM 노드.
    QGC 미션 종료(LOITER)를 감지한 후, 제어권을 받아 Offboard 미션을 수행합니다.
    이 노드는 Setpoint를 생성하지 않고, 상태 관리 및 모드 변경만 담당합니다.
    """
    def __init__(self):
        super().__init__("fsm")

        # ---------- 파라미터 선언 및 초기화 ----------
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("gripper_duration_s", 15.0)

        self.rate_hz = self.get_parameter("rate_hz").value
        self.dt_nom = 1.0 / max(self.rate_hz, 1.0)
        self.gripper_duration_s = self.get_parameter("gripper_duration_s").value

        # ---------- 상태 변수 ----------
        self.phase = Phase.IDLE   #self.phase = TRAY  self.phase = Phase.IDLE
        self.task = Task.IDLE  #self.task = APPROACH  self.task = Task.IDLE
        self.frame_id = 0
        self.prev_time = self.get_clock().now()
        self.task_enter_time = self.prev_time

        self.pos_ned = np.zeros(3, dtype=float)
        self.vehicle_status = None
        self.is_armed = False
        self.is_offboard = False

        self.gripper_hold = False

        self.suppress_takeover = False

        self.failed_count = 0

        # ---------- ROS Pub/Sub ----------
        self.tick_pub = self.create_publisher(ControlTick, "/mission/control_tick", QOS_VEHICLE_DEFAULT)
        self.ocm_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", QOS_VEHICLE_DEFAULT)
        self.vcmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", QOS_DEFAULT)

        self.create_subscription(Empty, "/mission/approach_finished", self._approach_finished_cb, QOS_VEHICLE_DEFAULT)
        self.create_subscription(Empty, "/mission/approach_failed", self._approach_failed_cb, QOS_VEHICLE_DEFAULT)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._vlp_cb, QOS_SENSOR)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self._vehicle_status_cb, QOS_SENSOR)


        # ---------- 타이머 ----------
        self.timer = self.create_timer(self.dt_nom, self._on_timer)
        self.get_logger().info(f"✅ [FSM] started ({self.rate_hz} Hz)")

    def _on_timer(self):
        """ 메인 FSM 루프. 타이머에 의해 주기적으로 호출됩니다. """
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        dt = dt if dt > 0.0 else self.dt_nom
        self.prev_time = now

        # 1. QGC 미션 종료 대기 상태
        if self.phase == Phase.WAIT_FOR_MISSION_END:
            if self.frame_id % (10 * int(self.rate_hz)) == 0: # 10초마다 로그 출력
                self.get_logger().info("Waiting for QGC to complete (monitoring LOITER)...")
            self._publish_tick(dt)
            self._publish_ocm()
            return
        
        # 2. Offboard 모드 전환 시퀀스
        if not self.is_offboard and self.phase != Phase.IDLE and not self.suppress_takeover:
            self._handle_offboard_takeover_sequence()
            self._publish_tick(dt)
            self._publish_ocm()
            return

        # --- 여기서부터는 Offboard 모드에서 FSM 미션을 수행합니다 ---
        if self.is_armed and self.is_offboard:
            # 3-1. OffboardControlMode는 항상 스트리밍
            self._publish_ocm()
            # 3-2. 현재 상태에 맞는 Setpoint 발행 (이 노드에서는 수행하지 않음)
            #self._maybe_publish_position_sp()
            # 3-3. 상태 전이 로직 (타임아웃 등)
            #self._housekeeping(now)

        # 4. ControlTick 발행
        self._publish_tick(dt)
        #     manual 수동 개입 감지 

        if self.task == Task.LAND:
            self._send_vehicle_command(PX4.VEHICLE_CMD_NAV_LAND)
            return
        


    def _vlp_cb(self, msg: VehicleLocalPosition):
        self.pos_ned[:] = [msg.x, msg.y, msg.z]

    def _vehicle_status_cb(self, msg: VehicleStatus):
        """ VehicleStatus 콜백. Arming, 비행 모드를 업데이트하고 QGC 미션 종료를 감지합니다. """
        self.vehicle_status = msg
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.is_offboard = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        if (msg.nav_state == VehicleStatus.NAVIGATION_STATE_MANUAL or msg.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL) and self.phase != Phase.IDLE:
            if not self.suppress_takeover:
                self.suppress_takeover = True
                self.get_logger().warn("User override (MANUAL/POSCTL). Suppressing Offboard takeover.")
            return
        
        if self.phase == Phase.IDLE and msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION:
            self._set_phase_task(Phase.WAIT_FOR_MISSION_END, Task.IDLE)
            self.get_logger().info("✈️ QGC Start Detacted.")
            return

        # QGC 미션 종료 감지 로직
        if self.phase == Phase.WAIT_FOR_MISSION_END and msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            self.get_logger().info("✈️ QGC Mission End detected. Initiating takeover.")
            self._set_phase_task(Phase.TRAY, Task.MOVE)

    def _approach_finished_cb(self, _msg: Empty):
        if self.task == Task.MOVE:
            self._set_task(Task.APPROACH)
            self.get_logger().info("🚩Move finished. Transitioning to Approach.")
            return
        
        if self.phase in (Phase.TRAY, Phase.SAFEPOINT) and self.task == Task.APPROACH:
            self.get_logger().info("🚩Approach finished. Transitioning to GRIPPER.")
            self._set_task(Task.GRIPPER)
            self.gripper_hold = False
            return
        
        if self.phase == Phase.VERTIPORT and self.task == Task.APPROACH:
            self.get_logger().info("🚩Approach finished. LAND.")
            self._set_phase_task(Phase.IDLE, Task.LAND)
            self._send_vehicle_command(PX4.VEHICLE_CMD_NAV_LAND)
            return
        
        if self.task == Task.GRIPPER:
            if self.phase == Phase.TRAY:
                self._set_phase_task(Phase.SAFEPOINT, Task.MOVE)
                self.get_logger().info("🚩Phase finished. Transitioning to Safepoint.")
            elif self.phase == Phase.SAFEPOINT:
                self._set_phase_task(Phase.VERTIPORT, Task.MOVE)
                self.get_logger().info("🚩Phase finished. Transitioning to Vertiport.")
            return
    
    def _approach_failed_cb(self, msg : Empty):
        if self.phase == Phase.TRAY and self.task == Task.GRIPPER:
            self._set_task(Task.APPROACH)
            self.get_logger().error(f"{self.failed_count}번 실패")
            self.failed_count += 1
            if self.failed_count == 5:
                self._set_phase_task(Phase.SAFEPOINT, Task.MOVE)
                self.get_logger().error("5번 실패")
                self.failed_count = 0
            return

    def _publish_tick(self, dt: float):
        self.frame_id += 1
        msg = ControlTick()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.frame_id = self.frame_id
        msg.dt = float(dt)
        msg.phase = self.phase.name
        msg.task = self.task.name
        self.tick_pub.publish(msg)

    def _publish_ocm(self):
        ocm = OffboardControlMode()
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ocm.position = (self.task == Task.MOVE) 
        ocm.velocity = (self.task != Task.MOVE)
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.ocm_pub.publish(ocm)

    def _publish_ocm_pos(self):
        ocm = OffboardControlMode()
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.ocm_pub.publish(ocm)

    def _maybe_publish_position_sp(self):
        """
        Setpoint 발행 함수. 이 FSM 노드에서는 Setpoint를 생성하지 않으므로 비워둡니다.
        혹시 생성할 일 있을까봐 냅둠
        """
        pass

    def _handle_offboard_takeover_sequence(self):
        """
        이미 비행 중인 기체의 제어권을 Offboard 모드로 가져오는 시퀀스를 처리합니다.
        메뉴얼 모드가 아닌 경우에는 다시 오프보드로 전환 시도
        """
        if self.frame_id % 10 == 0: # 너무 자주 보내지 않도록 조절
            # vehicle_status가 있고, 현재 모드가 MANUAL이 아닐 때만 명령 전송
            if self.vehicle_status and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_MANUAL:
                self.get_logger().info("Attempting to switch to Offboard mode...")
                self._send_vehicle_command(
                    PX4.VEHICLE_CMD_DO_SET_MODE,
                    PX4.MODE_CUSTOM,             # param1 = 1.0 (custom mode 사용)
                    PX4.MAIN_MODE_OFFBOARD,      # param2 = 6.0 (OFFBOARD main mode)
                )
            elif self.vehicle_status is None:
                self.get_logger().warn("Waiting for vehicle status before attempting to switch mode.")
            else:
                self.get_logger().info("In MANUAL mode, will not attempt to switch to Offboard.")

    def _send_vehicle_command(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1, msg.param2, msg.param3 = float(p1), float(p2), float(p3)
        msg.param4, msg.param5, msg.param6, msg.param7 = float(p4), float(p5), float(p6), float(p7)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vcmd_pub.publish(msg)
        self.get_logger().debug(f"Sent VehicleCommand: {command} ({p1}, {p2})")

    def _housekeeping(self, now):
        """
        GRIPPER 단계 경과 시간을 재고 다음 페이즈로 넘기는 함수
        """
        if self.task == Task.GRIPPER:
            elapsed = (now - self.task_enter_time).nanoseconds * 1e-9
            if elapsed >= self.gripper_duration_s:
                self.get_logger().info(f"Gripper duration ({self.gripper_duration_s}s) elapsed.")
                if self.phase == Phase.TRAY:
                    self._set_phase_task(Phase.SAFEPOINT, Task.MOVE)
                elif self.phase == Phase.SAFEPOINT:
                    self._set_phase_task(Phase.VERTIPORT, Task.MOVE)
                elif self.phase == Phase.VERTIPORT:
                    self.get_logger().info("Final gripper task finished. Mission complete. Idling.")
                    self._set_phase_task(Phase.IDLE, Task.MOVE)

    def _set_task(self, new_task: Task):
        if not isinstance(new_task, Task) or self.task == new_task: return
        self.task = new_task
        self.task_enter_time = self.get_clock().now()
        self.get_logger().info(f"➡️  TASK TRANSITION -> {self.task.name}")

    def _set_phase_task(self, new_phase: Phase, new_task: Task):
        if not isinstance(new_phase, Phase) or not isinstance(new_task, Task): return
        if self.phase == new_phase and self.task == new_task: return
        self.phase = new_phase
        self.task = new_task
        self.task_enter_time = self.get_clock().now()
        self.get_logger().info(f"➡️  PHASE/TASK TRANSITION -> {self.phase.name}/{self.task.name}")

def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        if node.is_armed:
            node.get_logger().info("Node is shutting down, sending DISARM command.")
            node._send_vehicle_command(PX4.VEHICLE_CMD_COMPONENT_ARM_DISARM, PX4.DISARM_COMMAND)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
