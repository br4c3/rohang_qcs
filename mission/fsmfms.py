import os
import yaml
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from std_msgs.msg import String
from mission_msgs.msg import MissionState, MissionEvent
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus, TrajectorySetpoint, VehicleLandDetected, VehicleLocalPosition

QOS_VEHICLE_DEFAULT = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)

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

class PX4:
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_NAV_LAND    = 21
    MODE_CUSTOM            = 1.0   
    MAIN_MODE_OFFBOARD     = 6.0   

class FSM(Node):
    def __init__(self):
        super().__init__("fsm")
        
        self.declare_parameter("state_graph_yaml", "")
        self.declare_parameter("intent_profiles_yaml", "") 
        self.declare_parameter("tick_hz", 20.0)

        sg_path = self.get_parameter("state_graph_yaml").value
        self.state_graph = self.load_yaml(sg_path)

        ip_path = self.get_parameter("intent_profiles_yaml").value
        raw_intent_yaml = self.load_yaml(ip_path)
        
        if '/**' in raw_intent_yaml and 'ros__parameters' in raw_intent_yaml['/**']:
            self.intent_profiles = raw_intent_yaml['/**']['ros__parameters']
        else:
            self.intent_profiles = raw_intent_yaml 
            
        self.current_state = self.state_graph.get("initial", "wait_for_mission_end")
        self.current_intent = "" 
       
        self.state_change_sub = self.create_subscription(MissionEvent, "/mission/event", self.mission_event_cb, 10)
        self.create_subscription(String, '/mission/force_state', self._force_state_callback, 10)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", self._vehicle_status_cb, QOS_SENSOR)
        # self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected",self.land_detected_cb, QOS_SENSOR)
        #=================== sils 확인 용 
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
            self._local_pos_cb, QOS_SENSOR
        )
#================================================

        self.ocm_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", QOS_VEHICLE_DEFAULT)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", QOS_VEHICLE_DEFAULT)
        self.vcmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", QOS_DEFAULT)
        self.state_pub = self.create_publisher(MissionState, "/mission/state", 10)
        
 
        self.lc_clients = {
            'coordinate': self.create_client(ChangeState, '/coordinate/change_state'),
            'yolo_aruco': self.create_client(ChangeState, '/yolo_aruco/change_state'),
            'gripper': self.create_client(ChangeState, '/gripper/change_state'),
            'mission_upload': self.create_client(ChangeState, '/mission_upload/change_state'),
            'yaw_heading': self.create_client(ChangeState, '/yaw_heading/change_state'),
        }

        self.node_active_states = {
            'coordinate': False,
            'yolo_aruco': False,
            'gripper': False,
            'mission_upload': False,
            'yaw_heading': False
        }
        self.is_restart = False

        self.is_armed = False
        self.is_offboard = False
        self.suppress_takeover = False
        self.vehicle_status = None
        
        self.frame_id = 0
        self.mission_started = False
       
        self.loiter_start_time = None
        self.takeover_requested = False

        self.land_state = None
        self.is_landed = False

        self.local_pos = None # sils

        self.wait_mission_event = 'FIRST_REACHED'

        tick_hz = self.get_parameter("tick_hz").value
        self.create_timer(1.0 / tick_hz, self.loop_tick)
        self.get_logger().info(f"🚀 FSM 시작 초기 상태: {self.current_state}")

    def change_lifecycle_state(self, client, transition_id, node_name="Node"):
        if not client.service_is_ready():
        
            return
        
        req = ChangeState.Request()
        req.transition.id = transition_id
        client.call_async(req)
# ============
    def _local_pos_cb(self, msg):
        self.local_pos = msg


    def _check_landed_by_altitude(self, alt_threshold=0.15, vz_threshold=0.15):
        if self.local_pos is None:
            return False
        near_ground = abs(self.local_pos.z) < alt_threshold  # z는 보통 down이 +
        low_vertical_speed = abs(self.local_pos.vz) < vz_threshold
        return near_ground and low_vertical_speed
#===========
    def lc_manage_nodes(self, intent_profile_name: str):
        if not intent_profile_name:
            return
        
        profile_data = self.intent_profiles.get(intent_profile_name, {})

        for node_name, params in profile_data.items():
            # guidance는 plain Node라 lifecycle 관리 대상이 아님 (mode 값은
            # /mission/state 구독으로 스스로 처리) → lc_clients에 없는 노드는 스킵
            if node_name not in self.lc_clients:
                continue

            mode = params.get('mode', 0)
            client = self.lc_clients[node_name]
            is_active = self.node_active_states[node_name]
            
            should_be_active = mode > 0

            if node_name == 'mission_upload' and self.is_restart:
                should_be_active = True

            if is_active == should_be_active:
                continue
            
            # 4. 실제 상태 변경 실행 (들여쓰기 최소화)
            if should_be_active:
                self.change_lifecycle_state(client, Transition.TRANSITION_ACTIVATE, node_name.capitalize())
                self.node_active_states[node_name] = True
                self.get_logger().info(f"🟢 [{node_name.capitalize()}] 활성화 명령 전송 (Mode: {mode})")
            else:
                self.change_lifecycle_state(client, Transition.TRANSITION_DEACTIVATE, node_name.capitalize())
                self.node_active_states[node_name] = False
                self.get_logger().info(f"🔴 [{node_name.capitalize()}] 대기 명령 전송 (Deactivated)")
    
    # def land_detected_cb(self, msg: VehicleLandDetected):
    #     self.is_landed = msg.landed

    def _vehicle_status_cb(self, msg: VehicleStatus):
        self.vehicle_status = msg
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.is_offboard = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION and not self.mission_started:
            self.mission_started = True
            self.get_logger().info("🎯 QGC 미션 실제 비행 시작 감지")

        # 수동 개입이 감지되면 이후(미션 완료 대기) 로직은 볼 필요 없음 → 즉시 종료
        if self.handle_manual(msg):
            return


        # if self.land_state and msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND and self.is_landed:
        #     self.transition_state(self.land_state)
        #     self.land_state = None
        #     return
        #==========================================
        if self.land_state and msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND \
            and self._check_landed_by_altitude():
            self.transition_state(self.land_state)
            self.land_state = None
            self.is_landed = True
            return
#==================================

        # wait_for_mission_end 상태에서 실제 미션이 시작된 경우에만 아래 로직 필요
        if self.current_state != "wait_for_mission_end" or not self.mission_started:
            return


        self.update_loiter_wait(msg)
        self.try_offboard_takeover()


    def handle_manual(self, msg: VehicleStatus) -> bool:
        """조종사 수동 개입 감지. 개입 상태(MANUAL/POSCTL)면 True 반환."""
        is_manual = msg.nav_state in (
            VehicleStatus.NAVIGATION_STATE_MANUAL,
            VehicleStatus.NAVIGATION_STATE_POSCTL,
        )
        if not is_manual:
            return False

        if not self.suppress_takeover and self.current_state != "wait_for_mission_end":
            self.suppress_takeover = True
            self.get_logger().warn("🚨 [Override] 조종사 수동 개입 감지! Offboard 자동 제어를 중단합니다.")

        return True


    def update_loiter_wait(self, msg: VehicleStatus):
        """QGC 미션 완료(LOITER) 감지 → 3초 대기 후 takeover_requested 설정."""
        if msg.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            if not self.takeover_requested:
                self.loiter_start_time = None
            return

        if self.loiter_start_time is None:
            self.loiter_start_time = self.get_clock().now()
            self.get_logger().info("⏳ QGC 미션 완료(LOITER) 감지. 3초간 대기합니다...")
            return

        elapsed = (self.get_clock().now() - self.loiter_start_time).nanoseconds / 1e9
        if elapsed >= 3.0 and not self.takeover_requested:
            self.get_logger().info("✅ 3초 대기 완료! PX4에 Offboard 전환을 요청합니다.")
            self.takeover_requested = True

    def try_offboard_takeover(self):
        """Offboard 진입이 확인되면 FSM 상태를 FIRST_REACHED로 전환."""
        if not (self.takeover_requested and self.is_offboard):
            return

        transitions = self.state_graph.get("states", {}) \
                                    .get("wait_for_mission_end", {}) \
                                    .get("transitions", {})
        next_state = transitions.get(self.wait_mission_event)
        if next_state is None:
            self.get_logger().warn("⚠️ wait_for_mission_end에 FIRST_REACHED 전이가 정의되어 있지 않습니다.")
            return

        self.current_state = next_state
        self.get_logger().info(
            f"🚀 PX4 Offboard 진입 확인! FSM 상태 전환: wait_for_mission_end ➡️ {self.current_state}"
        )
        self.is_restart = False
        self.mission_started = False
        self.loiter_start_time = None
        self.takeover_requested = False

    def mission_event_cb(self, msg: MissionEvent):
        event_name = msg.name
        self.get_logger().info(f"📩 미션 이벤트 수신: {event_name}")

        states_cfg = self.state_graph.get("states", {})
        current_state_cfg = states_cfg.get(self.current_state, {})
        transitions = current_state_cfg.get("transitions", {})

        next_state = transitions.get(event_name)
        if next_state is None:
            self.get_logger().warn(
                f"⚠️ 무시된 이벤트: 현재 상태 '{self.current_state}'에는 '{event_name}' 전환(transition)이 정의되어 있지 않습니다."
            )
            return
        
        if event_name == "APPROACH_DONE":
            self.landing(next_state)
            return
        self.is_landed = False
        self.transition_state(next_state)

    def transition_state(self,next_state):
        
        prev_state = self.current_state
        self.current_state = next_state
        
        if self.current_state =="restart_mission":
            self.is_restart =True
        
        if self.current_state =="wait_for_mission_end":
            self.wait_mission_event = "SECOND_REACHED"
        self.get_logger().info(f"🔄 상태 전환: {prev_state} ➡️ {self.current_state}")
   
    def landing(self, next_state):
        if self.land_state:
            return
        self.land_state = next_state
        self._send_vehicle_command(PX4.VEHICLE_CMD_NAV_LAND)


    def _force_state_callback(self, msg: String):
        target_state = msg.data
        states_cfg = self.state_graph.get("states", {})
        
        if target_state in states_cfg:
            self.get_logger().warn(f"🚨 [강제 개입] 상태 강제 전환: {self.current_state} ➡️ {target_state}")
            self.current_state = target_state
        else:
            self.get_logger().error(f"❌ [강제 개입 실패] '{target_state}'는 YAML에 없는 상태입니다.")

    def load_yaml(self, path: str):
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        self.get_logger().error(f"❌ YAML 파일을 찾을 수 없습니다: {path}")
        return {}

    def loop_tick(self):
        if not hasattr(self, 'state_graph') or not self.state_graph:
            return

        states_cfg = self.state_graph.get("states", {})
        current_state_cfg = states_cfg.get(self.current_state, {})
        new_intent = current_state_cfg.get("intent_profile", "")

        if new_intent != self.current_intent:
            self.lc_manage_nodes(new_intent)
            self.current_intent = new_intent

        self._publish_ocm(self.current_intent)
        # if not self.is_offboard:
        #     self._publish_dummy_setpoint()

        OFFBOARD_MODE = {'rescue/tray/move','deliver/vertiport/move',
                         'rescue/tray/approach','deliver/vertiport/approach'}

        should_takeover = (self.current_state in OFFBOARD_MODE or self.takeover_requested) and not self.is_landed and self.land_state is None #and not self.is_landed) or self.takeover_requested
        
        if should_takeover and not self.suppress_takeover:
            self._handle_offboard_takeover_sequence()

        msg = MissionState()
        msg.state = self.current_state
        msg.intent_profile = new_intent
        self.state_pub.publish(msg)
        
        self.frame_id += 1 

    def _handle_offboard_takeover_sequence(self):

        if not self.is_offboard:
            if self.frame_id % 10 == 0:  
                if self.vehicle_status and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_MANUAL:
                    self.get_logger().info("🎯 PX4에 Offboard 제어권 전환 요청 중...",throttle_duration_sec=5.5)
                    self._send_vehicle_command(
                        PX4.VEHICLE_CMD_DO_SET_MODE,
                        PX4.MODE_CUSTOM,
                        PX4.MAIN_MODE_OFFBOARD
                    )
                elif self.vehicle_status is None:
                    self.get_logger().warn("⚠️ VehicleStatus 수신 대기 중으로 Offboard 전환 명령 보류...",throttle_duration_sec=5.5)
    
    def _publish_ocm(self, intent_profile: str):
        ocm = OffboardControlMode()
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        if intent_profile and ("move" in intent_profile.lower() or "position" in intent_profile.lower()):
            ocm.position = True
            ocm.velocity = False
        else:
            ocm.position = False
            ocm.velocity = True
            
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.ocm_pub.publish(ocm)

    def _publish_dummy_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [0.0, 0.0, 0.0]  
        msg.yaw = float('nan')
        self.traj_pub.publish(msg)

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

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FSM())
    rclpy.shutdown()

if __name__ == "__main__":
    main()