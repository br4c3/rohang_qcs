#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from mission_msgs.msg import MissionEvent
import sys
import termios
import tty

# 키보드 입력에 매핑할 이벤트 이름들 (YAML 파일 기준)
KEY_MAPPING = {
    '1': 'REACHED',
    '2': 'APPROACH_DONE',
    '3': 'TARGET_LOST',
    '4': 'GRIP_OK',
    '5': 'GRIP_FAIL',
    '6': 'SECOND_REACHED',
    '7': 'FIRST_REACHED',
}

class KeyboardTester(Node):
    def __init__(self):
        super().__init__('keyboard_tester')
        self.event_pub = self.create_publisher(MissionEvent, '/mission/event', 10)
        self.get_logger().info('✅ 키보드 테스터 노드가 시작되었습니다.')
        self.print_menu()

    def print_menu(self):
        print("\n" + "="*40)
        print(" ⌨️ FSM 키보드 테스터 메뉴")
        print("="*40)
        for key, event in KEY_MAPPING.items():
            print(f" [{key}] : '{event}' 이벤트 발행")
        print(" [q] : 종료 (Quit)")
        print("="*40)
        print("원하는 키를 누르세요...")

    def publish_event(self, event_name: str):
        msg = MissionEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.name = event_name
        msg.severity = 0
        msg.detail = "Keyboard tester triggered"
        
        self.event_pub.publish(msg)
        self.get_logger().info(f"📢 이벤트 발행 완료: {event_name}")

def get_key():
    """엔터 키 없이 키보드 입력을 1개만 바로 읽어오는 함수"""
    tty.setraw(sys.stdin.fileno())
    select_fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(select_fd)
    try:
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(select_fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rclpy.init()
    node = KeyboardTester()

    try:
        while rclpy.ok():
            key = get_key()
            
            if key in KEY_MAPPING:
                event_name = KEY_MAPPING[key]
                node.publish_event(event_name)
            elif key == 'q' or key == '\x03': # q 또는 Ctrl+C
                print("\n종료합니다.")
                break
                
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()