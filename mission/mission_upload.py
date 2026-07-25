import sys
import time
from pathlib import Path

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from mavros_msgs.msg import State as MavrosState, Waypoint, WaypointReached
from mavros_msgs.srv import (
    CommandBool,
    SetMode,
    WaypointClear,
    WaypointPush,
    WaypointSetCurrent,
)
from mission_msgs.msg import MissionEvent  # FSM과 통신할 이벤트 메시지 추가

from .config import load_qgc_plan, validate_hover_plan

AUTO_MISSION_MODE = "AUTO.MISSION"
SERVICE_TIMEOUT = 30.0
CONNECTION_TIMEOUT = 30.0
MISSION_TIMEOUT = 900.0
MISSION_PLAN = Path(
    "/home/kyh/sils_ws/src/mission/config/outbound.plan"
)


class MissionUploader(LifecycleNode):
    def __init__(self, items):
        # 기존 Node 대신 LifecycleNode로 초기화
        super().__init__("mission_uploader")
        self.state = None
        self.reached_sequence = -1
        self.items = items 
        self.is_active = False
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.service_cb_group = ReentrantCallbackGroup()

        self.clear_client = self.create_client(
            WaypointClear, "/mavros/mission/clear",
            callback_group=self.service_cb_group,
        )
        self.push_client = self.create_client(
            WaypointPush, "/mavros/mission/push",
            callback_group=self.service_cb_group,
        )
        self.current_client = self.create_client(
            WaypointSetCurrent, "/mavros/mission/set_current",
            callback_group=self.service_cb_group,
        )
        self.arm_client = self.create_client(
            CommandBool, "/mavros/cmd/arming",
            callback_group=self.service_cb_group,
        )
        self.mode_client = self.create_client(
            SetMode, "/mavros/set_mode",
            callback_group=self.service_cb_group,
        )

        self.create_subscription(
            MavrosState, "/mavros/state", self.on_state, 10,
            callback_group=self.service_cb_group,
        )
        self.create_subscription(
            WaypointReached, "/mavros/mission/reached", self.on_waypoint_reached, 10,
            callback_group=self.service_cb_group,
        )

        self.event_pub = self.create_lifecycle_publisher(MissionEvent, "/mission/event", 10)

        self.get_logger().info("🟢 MissionUploader 구성(Configure) 완료 — activate 대기 중")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        super().on_activate(state)
        self.is_active = True
        self.get_logger().info("🟢 MissionUploader 활성화 — 미션 업로드 시작")
        self.upload_timer = self.create_timer(0.01, self.run_mission)
        
        return TransitionCallbackReturn.SUCCESS

    def run_mission(self):
        self.upload_timer.cancel()
        try:
            self.wait_until_connected()
            self.upload(self.items)
            self.start_mission()
            self.wait_until_complete(len(self.items))
            self.get_logger().info(
                "마지막 waypoint에서 호버링합니다. 착륙 명령은 운용자가 별도로 내려야 합니다."
            )
        except (TimeoutError, RuntimeError) as error:
            self.get_logger().error(f"❌ 미션 업로드/실행 실패: {error}")
            self.trigger_deactivate()  

        

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.is_active = False
        self.get_logger().info("🔴 MissionUploader 비활성화(Deactivate) 완료")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("🧹 MissionUploader 정리(Cleanup) 완료")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_state(self, message):
        self.state = message

    def on_waypoint_reached(self, message):
        if not self.is_active:
            return
        self.reached_sequence = max(self.reached_sequence, message.wp_seq)
        self.get_logger().info(f"미션 진행: waypoint {message.wp_seq} 도착")

    def wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.05)
        raise TimeoutError(f"제한 시간 안에 {description} 상태가 되지 않았습니다")
    
    def call_service(self, client, request, description):
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            raise TimeoutError(f"MAVROS {description} 서비스를 찾을 수 없습니다")
        future = client.call_async(request)
        deadline = time.monotonic() + SERVICE_TIMEOUT
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                break
            time.sleep(0.05)
        if not future.done():
            raise TimeoutError(f"MAVROS {description} 서비스 응답이 없습니다")
        if future.exception() is not None:
            raise RuntimeError(f"MAVROS {description} 서비스 실패: {future.exception()}")
        return future.result()

    def wait_until_connected(self):
        self.wait_for(
            lambda: self.state is not None and self.state.connected,
            CONNECTION_TIMEOUT,
            "PX4 연결",
        )
        self.get_logger().info("PX4 연결 확인")

    def upload(self, waypoints):
        clear_response = self.call_service(self.clear_client, WaypointClear.Request(), "미션 삭제")
        if clear_response is None or not clear_response.success:
            raise RuntimeError("기존 미션 삭제에 실패했습니다")

        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = [to_ros_waypoint(item) for item in waypoints]
        push_response = self.call_service(self.push_client, request, "미션 업로드")
        if (
            push_response is None
            or not push_response.success
            or push_response.wp_transfered != len(waypoints)
        ):
            transferred = 0 if push_response is None else push_response.wp_transfered
            raise RuntimeError(
                f"{len(waypoints)}개 중 {transferred}개만 업로드됐습니다"
            )

        current_request = WaypointSetCurrent.Request()
        current_request.wp_seq = 0
        current_response = self.call_service(
            self.current_client, current_request, "첫 waypoint 선택"
        )
        if current_response is None or not current_response.success:
            raise RuntimeError("첫 waypoint 선택에 실패했습니다")

        self.reached_sequence = -1
        self.get_logger().info(f"{len(waypoints)}개 waypoint 업로드 완료")

    def arm(self):
        if self.state is not None and self.state.armed:
            return

        request = CommandBool.Request()
        request.value = True
        response = self.call_service(self.arm_client, request, "arm")
        if response is None or not response.success:
            raise RuntimeError(
                f"PX4 arm 거부: MAV_RESULT={getattr(response, 'result', 'unknown')}"
            )

        self.wait_for(lambda: self.state is not None and self.state.armed, 10, "arm")

    def start_mission(self):
        request = SetMode.Request()
        request.custom_mode = AUTO_MISSION_MODE
        response = self.call_service(self.mode_client, request, "AUTO.MISSION 전환")
        if response is None or not response.mode_sent:
            raise RuntimeError("AUTO.MISSION 모드 전환에 실패했습니다")

        self.wait_for(
            lambda: self.state is not None and self.state.mode == AUTO_MISSION_MODE,
            10,
            "AUTO.MISSION",
        )

        self.arm()
        self.get_logger().info("미션 시작")

        # ======= 추가/수정된 핵심 기능 =======
        # PX4가 AUTO.MISSION 모드에 성공적으로 진입했으므로 FSM에 이벤트 발행
        event_msg = MissionEvent()
        event_msg.name = "SECOND_REACHED"
        self.event_pub.publish(event_msg)
        self.get_logger().info("📢 PX4 미션 모드 진입 확인: 'REACHED' 이벤트 FSM으로 발행 완료")
        # ==================================

    def wait_until_complete(self, waypoint_count):
        final_sequence = waypoint_count - 1
        self.wait_for(
            lambda: self.reached_sequence >= final_sequence,
            MISSION_TIMEOUT,
            "미션 마지막 waypoint 도착",
        )
        self.get_logger().info("마지막 waypoint 도착")


def to_ros_waypoint(item):
    waypoint = Waypoint()
    waypoint.frame = item["frame"]
    waypoint.command = item["command"]
    waypoint.is_current = item["is_current"]
    waypoint.autocontinue = item["autocontinue"]
    waypoint.param1 = item["param1"]
    waypoint.param2 = item["param2"]
    waypoint.param3 = item["param3"]
    waypoint.param4 = item["param4"]
    waypoint.x_lat = item["latitude"]
    waypoint.y_long = item["longitude"]
    waypoint.z_alt = item["altitude"]
    return waypoint


def print_plan_check(summary):
    path, item_count, start, end = summary
    print("\n========== 업로드할 미션 확인 ==========")
    print(f"미션 파일: {path}")
    print(f"항목 {item_count}개, {start} → {end}")
    print("마지막 항목 도착 후 착륙하지 않고 호버링합니다.")
    print("========================================\n")


def run():
    items, summary = load_qgc_plan(MISSION_PLAN)
    validate_hover_plan(items)

    print_plan_check(summary)

    node = MissionUploader(items)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        node.trigger_configure()
        executor.spin()
    finally:
        node.destroy_node()


def main():
    rclpy.init()
    try:
        run()
    except (TimeoutError, RuntimeError, ValueError) as error:
        print(f"오류: {error}", file=sys.stderr)
        raise SystemExit(1) from error
    except KeyboardInterrupt:
        print("사용자가 실행을 중단했습니다")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
