#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from mission_msgs.action import MovePos


QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

QOS_DEFAULT = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

R_EARTH = 6371000.0 


class PositionActionServer(Node):
    def __init__(self):
        super().__init__('position_action_server')
        
        # 내부 상태 변수 초기화
        self.pos_ned = np.zeros(3, dtype=float)
        self.vel_ned = np.zeros(3, dtype=float)
        self.current_yaw = 0.0
        self.is_pos_received = False


        self.loc_pos = None
        self.origin_set = False
        self.home_LLA = np.zeros(3, dtype=float)

        self.callback_group = ReentrantCallbackGroup()

        self.sp_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", QOS_DEFAULT)
        
        self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position_v1', 
            self.local_pos_cb, 
            QOS_SENSOR,
            callback_group=self.callback_group
        )

        self._action_server = ActionServer(
            self,
            MovePos,
            '/guidance/move_to_position',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("[Position Action Server]가 정상적으로 시작되었습니다.")

    def local_pos_cb(self, msg: VehicleLocalPosition):
        """ 드론의 현재 NED 위치 및 맵 기준 Heading 수신 """
        self.pos_ned[:] = [msg.x, msg.y, msg.z]
        self.vel_ned[:] = [msg.vx, msg.vy, msg.vz]
        self.current_yaw = msg.heading
        self.is_pos_received = True

        self.loc_pos = msg
        self._init_origin_if_needed()

    def goal_callback(self, goal_request):
        
        if not self.is_pos_received:
            self.get_logger().warn("⚠️ 아직 PX4로부터 Local Position을 받지 못해 명령을 거절합니다.",throttle_duration_sec=5.0)
            return GoalResponse.REJECT
        
        self.get_logger().info(f"🎯 새로운 이동 목표 수신: {goal_request.target_lla}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """ 이동 중 FSM 취소나 수동 개입 시 안전하게 Cancel 승인 """
        self.get_logger().warn("🚨 이동 명령 취소 요청 접수!",)
        return CancelResponse.ACCEPT

    def cal_yaw_toward_next_position(self, init_x, init_y, des_x, des_y):
        
        dx = des_x - init_x
        dy = des_y - init_y

        if dx == 0:
            return 1.57
        elif math.sqrt(dx**2 + dy**2) < 1.0:
            return float('nan')
        else:
            return math.atan2(dy, dx)

    def publish_position_ned(self, target_ned, des_yaw):
        
        ts = TrajectorySetpoint()
        ts.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ts.position = [float(p) for p in target_ned]
        ts.velocity = [math.nan, math.nan, math.nan]
        ts.acceleration = [math.nan, math.nan, math.nan]
        ts.yaw = float(des_yaw)
        ts.yawspeed = math.nan
        self.sp_pub.publish(ts)


    def _init_origin_if_needed(self):
        if self.origin_set or self.loc_pos is None:
            return
        
        lp = self.loc_pos
        if (math.isfinite(lp.ref_lat) and lp.ref_lat != 0.0 and
            math.isfinite(lp.ref_lon) and lp.ref_lon != 0.0 and
            math.isfinite(lp.ref_alt) and lp.ref_timestamp != 0):
            self.home_LLA[0] = float(lp.ref_lat)
            self.home_LLA[1] = float(lp.ref_lon)
            self.home_LLA[2] = float(lp.ref_alt)
            self.origin_set = True
            ned0 = self.geodetic_to_ned(self.home_LLA)
            self.get_logger().info(f"Origin LLA set.{self.home_LLA}")
            self.get_logger().info(f"Valid info : Home NED is {ned0}")


    def geodetic_to_ned(self, LLA):
        if not self.is_pos_received or not self.origin_set:
            return GoalResponse.REJECT
        dlat  = math.radians(LLA[0] - self.home_LLA[0])
        dlon  = math.radians(LLA[1] - self.home_LLA[1])
        north = dlat * R_EARTH
        east  = dlon * R_EARTH * math.cos(math.radians(self.home_LLA[0]))
        down  = self.home_LLA[2] - LLA[2]
        self.home_NED = np.array([north, east, down])
        self.get_logger().info(f'{self.home_NED}')
        return np.array([north, east, down])


    def execute_callback(self, goal_handle):
        self.get_logger().info("🛫 위치 제어 액션 루프 가동 시작...")
        
        target_lla = goal_handle.request.target_lla 
        
        target_ned = self.geodetic_to_ned(target_lla)
        
        req_yaw = goal_handle.request.des_yaw
        
        # if math.isnan(req_yaw):
        #     des_yaw = self.cal_yaw_toward_next_position(self.pos_ned[0], self.pos_ned[1], target_ned[0], target_ned[1])
        # else:
        #     des_yaw = req_yaw
        des_yaw = float("nan")
        feedback_msg = MovePos.Feedback()
        result = MovePos.Result()
        loop_rate = self.create_rate(20.0)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn("🛑 액션이 정상적으로 취소되었습니다.")
                result.success = False
                return result

            self.publish_position_ned(target_ned, des_yaw)
            distance = float(np.linalg.norm(self.pos_ned - target_ned))
            feedback_msg.distance_remaining = distance
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(
                f" [비행중] 남은거리: {distance:.2f}m | "
                f"현재 NED: [{self.pos_ned[0]:.2f}, {self.pos_ned[1]:.2f}, {self.pos_ned[2]:.2f}] | "
                f"목표 NED: [{target_ned[0]:.2f}, {target_ned[1]:.2f}, {target_ned[2]:.2f}]",
                throttle_duration_sec=1.0
            )

            if distance < 1.0:
                self.get_logger().info(f"🚩 [도달 완료] 목표지 오차 {distance:.2f}m 이내 진입.")
                break

            loop_rate.sleep()

        goal_handle.succeed()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = PositionActionServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()