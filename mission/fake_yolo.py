#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SITL 테스트용 가짜 YOLO 노드.

- 평소에는 이미지 중심(cx, cy) 근처에서 랜덤 지터를 주며 /yolo/filtered_center 를 발행한다.
- 스페이스바를 누르면 '정렬(aligned)' 모드로 토글되어, 정확히 (cx, cy) 값을 발행한다.
  (coordinate_tf.py 의 K_inv, cx/cy 파라미터와 동일한 값을 써야 실제 픽셀 좌표와 맞음)
- q 를 누르면 종료.

주의: coordinate_tf.py 는 현재 yolo 구독이 주석 처리되어 있어서, 아래처럼
on_configure 에서 구독을 살리고 yolo_cb 를 추가해야 이 노드가 발행하는 값을
실제로 받아서 처리한다.

    self.create_subscription(Point, '/yolo/filtered_center', self.yolo_cb, qos_profile)

    def yolo_cb(self, msg: Point):
        self.yolo_center = np.array([msg.x, msg.y, 1.0])
        self.last_yolo_time = self.get_clock().now()
"""

import sys
import tty
import termios
import select

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

# coordinate_tf.py 의 기본 파라미터와 동일하게 맞춰줌
CX, CY = 640.0, 360.0

# 지터 모드에서 중심으로부터 흔들리는 범위 (px)
JITTER_RANGE = 40.0
# 지터에 약간의 관성(랜덤워크 느낌)을 주기 위한 이동 폭
JITTER_STEP = 6.0

PUBLISH_HZ = 20.0


class FakeYoloNode(Node):
    def __init__(self):
        super().__init__('fake_yolo_node')

        self.pub = self.create_publisher(Point, '/yolo/filtered_center', 10)

        self.aligned = False
        self.jitter_u = 0.0
        self.jitter_v = 0.0

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.tick)

        self._settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info(
            "🎯 Fake YOLO 노드 시작. [Space] 정렬 토글 / [q] 종료 / 현재: 지터"
        )

    def _get_key_nonblocking(self):
        """터미널에서 Enter 없이 한 글자 논블로킹으로 읽기."""
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if rlist:
            return sys.stdin.read(1)
        return ''

    def tick(self):
        key = self._get_key_nonblocking()

        if key == ' ':
            self.aligned = not self.aligned
            mode_str = "정렬(aligned)" if self.aligned else "지터(jitter)"
            self.get_logger().info(f"🔁 모드 전환 → {mode_str}")
        elif key == 'q':
            self.get_logger().info("종료합니다.")
            self.destroy_timer(self.timer)
            rclpy.shutdown()
            return

        if self.aligned:
            u, v = CX, CY
        else:
            # 랜덤워크 + 클램프로 자연스러운 지터
            self.jitter_u = np.clip(
                self.jitter_u + np.random.uniform(-JITTER_STEP, JITTER_STEP),
                -JITTER_RANGE, JITTER_RANGE
            )
            self.jitter_v = np.clip(
                self.jitter_v + np.random.uniform(-JITTER_STEP, JITTER_STEP),
                -JITTER_RANGE, JITTER_RANGE
            )
            u = CX + self.jitter_u
            v = CY + self.jitter_v

        msg = Point()
        msg.x = float(u)
        msg.y = float(v)
        msg.z = 0.0
        self.pub.publish(msg)

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)


def main(args=None):
    rclpy.init(args=args)
    node = FakeYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
