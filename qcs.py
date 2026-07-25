#!/usr/bin/env python3
"""
ARECADA GCS
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

import sys

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from px4_msgs.msg import VehicleStatus



VEHICLE_STATUS_TOPIC = "/fmu/out/vehicle_status"



BG = "#F5FAFD"          # �꾩껜 諛곌꼍
PANEL = "#FFFFFF"       # �⑤꼸 諛곌꼍
PANEL_DARK = "#EAF6FC"  # 踰꾪듉 諛� �대� �곸뿭
BORDER = "#B9DCEA"      # �뚮몢由�
ACCENT = "#45B7E8"      # 媛뺤“��
ACCENT_DARK = "#2196D2" # �좏깮 諛� �대┃ �곹깭
TEXT = "#1D3442"        # 湲곕낯 湲���
MUTED = "#668494"       # 蹂댁“ 湲���


class FlightModeNode(Node):
    def __init__(self, mode_callback):
        super().__init__("gcs_flight_mode_node")
        self.mode_callback = mode_callback

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            VehicleStatus,
            VEHICLE_STATUS_TOPIC,
            self._vehicle_status_callback,
            qos,
        )

    def _vehicle_status_callback(self, msg):
        nav_state = int(msg.nav_state)

        if nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION:
            mode_text = "MISSION"
        elif nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            mode_text = "OFFBOARD"
        elif nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL:
            mode_text = "POSITION"
        elif nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            mode_text = "HOLD"
        elif nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
            mode_text = "RETURN"
        elif nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            mode_text = "LAND"
        elif nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
            mode_text = "TAKEOFF"
        else:
            mode_text = f"MODE {nav_state}"

        self.mode_callback(mode_text)

class ControlButton(QPushButton):
    """湲곕뒫 �대쫫留� �쒖떆�섎뒗 �쇰컲 踰꾪듉."""

    def __init__(self, text: str, parent=None):
        super().__init__(text, parent)
        self.setObjectName("controlButton")
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(38)


class EmptyDisplayPage(QFrame):
    """移대찓�� �먮뒗 QGC �붾㈃�� �ㅼ뼱媛� 鍮� 怨듦컙."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("emptyDisplay")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(500, 350)


class GroundStationMockup(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("GCS")
        self.resize(1280, 760)
        self.setMinimumSize(1000, 620)

        self.ros_node = None
        self.ros_timer = None

        self._build_ui()
        self._apply_style()
        self._start_ros()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)

        outer_layout = QHBoxLayout(root)
        outer_layout.setContentsMargins(24, 20, 24, 20)
        outer_layout.setSpacing(16)

        outer_layout.addWidget(self._create_left_panel())
        outer_layout.addLayout(self._create_right_panel(), 1)

    def _create_left_panel(self):
        panel = QFrame()
        panel.setObjectName("leftPanel")
        panel.setFixedWidth(290)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        layout.addWidget(self._section_title("FLIGHT MODE"))

        self.flight_mode_display = QLabel("WAITING FOR PX4")
        self.flight_mode_display.setObjectName("modeDisplay")
        self.flight_mode_display.setAlignment(Qt.AlignCenter)
        self.flight_mode_display.setMinimumHeight(48)
        layout.addWidget(self.flight_mode_display)

        layout.addWidget(self._section_title("MISSION STATE"))
        self.state_button = QPushButton("STATE")
        self.state_button.setObjectName("largeButton")
        layout.addWidget(self.state_button)

        layout.addWidget(self._section_title("MODULE CONTROL"))

        control_names = [
            "cam_yolo",
            "coordinate",
            "yaw_align",
            "gripper",
            "mission_upload",
            "�섎� 媛쒗룓",
        ]

        self.control_buttons = {}
        for name in control_names:
            button = ControlButton(name)
            self.control_buttons[name] = button
            layout.addWidget(button)

        layout.addWidget(self._section_title("VTOL FLIGHT STATE"))
        self.vtol_state_button = QPushButton("FW / TRANSITION / MC")
        self.vtol_state_button.setObjectName("largeButton")
        layout.addWidget(self.vtol_state_button)

        layout.addStretch(1)
        return panel

    @staticmethod
    def _section_title(text: str):
        label = QLabel(text)
        label.setObjectName("sectionTitle")
        return label

    def _create_right_panel(self):
        right_layout = QVBoxLayout()
        right_layout.setSpacing(12)

        right_layout.addWidget(self._create_main_display(), 1)
        right_layout.addLayout(self._create_telemetry_panel())

        return right_layout

    def _create_main_display(self):
        frame = QFrame()
        frame.setObjectName("displayFrame")

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        toolbar = QHBoxLayout()
        toolbar.addStretch(1)

        self.camera_button = QPushButton("CAMERA")
        self.qgc_button = QPushButton("QGC")
        self.camera_button.setObjectName("viewButton")
        self.qgc_button.setObjectName("viewButton")

        self.camera_button.setCheckable(True)
        self.qgc_button.setCheckable(True)
        self.camera_button.setChecked(True)

        button_group = QButtonGroup(self)
        button_group.setExclusive(True)
        button_group.addButton(self.camera_button, 0)
        button_group.addButton(self.qgc_button, 1)

        toolbar.addWidget(self.camera_button)
        toolbar.addWidget(self.qgc_button)

        layout.addLayout(toolbar)

        self.display_stack = QStackedWidget()
        self.camera_page = EmptyDisplayPage()
        self.qgc_page = EmptyDisplayPage()

        self.display_stack.addWidget(self.camera_page)
        self.display_stack.addWidget(self.qgc_page)

        self.camera_button.clicked.connect(
            lambda: self.display_stack.setCurrentWidget(self.camera_page)
        )
        self.qgc_button.clicked.connect(
            lambda: self.display_stack.setCurrentWidget(self.qgc_page)
        )

        layout.addWidget(self.display_stack, 1)
        return frame

    def _create_telemetry_panel(self):
        layout = QHBoxLayout()
        layout.setSpacing(10)

        self.telemetry_labels = {}

        initial_values = {
            "GPS": "NO DATA",
            "TELEM": "DISCONNECTED",
            "THROTTLE": "-- %",
            "ALTITUDE": "--.- m",
            "AIRSPEED": "--.- m/s",
        }

        for title in [
            "GPS",
            "TELEM",
            "THROTTLE",
            "ALTITUDE",
            "AIRSPEED",
        ]:
            card = QFrame()
            card.setObjectName("telemetryCard")
            card.setSizePolicy(
                QSizePolicy.Expanding,
                QSizePolicy.Fixed,
            )

            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(10, 5, 10, 10)
            card_layout.setSpacing(3)

            title_label = QLabel(title)
            title_label.setObjectName("telemetryTitle")
            title_label.setAlignment(Qt.AlignCenter)

            value_label = QLabel(initial_values[title])
            value_label.setObjectName("telemetryValue")
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setMinimumHeight(32)

            self.telemetry_labels[title] = value_label

            card_layout.addWidget(title_label)
            card_layout.addWidget(value_label)

            layout.addWidget(card)

        return layout

    def _start_ros(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self.ros_node = FlightModeNode(self._update_flight_mode)

            self.ros_timer = QTimer(self)
            self.ros_timer.timeout.connect(self._spin_ros_once)
            self.ros_timer.start(50)

        except Exception as error:
            self.flight_mode_display.setText("ROS ERROR")
            print(f"Failed to start ROS node: {error}")
            self.ros_node = None

    def _spin_ros_once(self):
        if self.ros_node is None:
            return

        try:
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)
        except Exception as error:
            self.flight_mode_display.setText("ROS ERROR")
            print(f"ROS spin error: {error}")

    def _update_flight_mode(self, mode_text):
        self.flight_mode_display.setText(mode_text)

    def closeEvent(self, event):
        if self.ros_timer is not None:
            self.ros_timer.stop()

        if self.ros_node is not None:
            self.ros_node.destroy_node()
            self.ros_node = None

        if rclpy.ok():
            rclpy.shutdown()

        event.accept()

    def _apply_style(self):
        self.setStyleSheet(f"""
            * {{
                font-family: "Noto Sans CJK KR", "Noto Sans KR",
                            "DejaVu Sans", sans-serif;
                color: {TEXT};
            }}

            QMainWindow,
            QWidget {{
                background-color: {BG};
            }}

            QFrame#leftPanel,
            QFrame#displayFrame {{
                background-color: {PANEL};
                border: 1px solid {BORDER};
                border-radius: 14px;
            }}

            QLabel#sectionTitle {{
                color: {ACCENT};
                font-size: 12px;
                font-weight: 800;
                letter-spacing: 1px;
                padding-top: 4px;
            }}

            /* 泥� 踰덉㎏ 肄붾뱶�� ROS 鍮꾪뻾 紐⑤뱶 �쒖떆李� */
            QLabel#modeDisplay {{
                min-height: 48px;
                background-color: {PANEL_DARK};
                border: 2px solid {ACCENT};
                border-radius: 10px;
                color: {TEXT};
                font-size: 18px;
                font-weight: 800;
            }}

            QPushButton#largeButton {{
                min-height: 48px;
                background-color: {PANEL_DARK};
                border: 2px solid {ACCENT};
                border-radius: 10px;
                color: {TEXT};
                font-size: 18px;
                font-weight: 800;
            }}

            QPushButton#largeButton:hover {{
                background-color: #D9F1FB;
                border-color: {ACCENT_DARK};
            }}

            QPushButton#largeButton:pressed {{
                background-color: #BFE7F7;
            }}

            QPushButton#controlButton {{
                min-height: 38px;
                background-color: #F3FAFD;
                border: 1px solid {BORDER};
                border-radius: 8px;
                color: #1D3442;
                font-size: 14px;
                font-weight: 700;
                text-align: left;
                padding-left: 14px;
            }}

            QPushButton#controlButton:hover {{
                border: 1px solid {ACCENT};
                background-color: #DFF3FB;
                color: #163A4A;
            }}

            QPushButton#controlButton:pressed {{
                background-color: {ACCENT_DARK};
                color: white;
            }}

            QPushButton#viewButton {{
                min-width: 100px;
                min-height: 34px;
                background-color: {PANEL_DARK};
                border: 1px solid {BORDER};
                border-radius: 7px;
                color: {MUTED};
                font-weight: 700;
            }}

            QPushButton#viewButton:hover {{
                border-color: {ACCENT};
                background-color: #DFF3FB;
            }}

            QPushButton#viewButton:checked {{
                background-color: {ACCENT_DARK};
                border: 1px solid {ACCENT};
                color: white;
            }}

            QFrame#emptyDisplay {{
                background-color: #070B10;
                border: 1px solid {BORDER};
                border-radius: 10px;
            }}

            QFrame#telemetryCard {{
                background-color: {PANEL};
                border: 1px solid {BORDER};
                border-radius: 10px;
            }}

            QLabel#telemetryTitle {{
                color: #24546B;
                font-size: 13px;
                font-weight: 900;
                background-color: transparent;
            }}

            QLabel#telemetryValue {{
                background-color: #F0F8FC;
                border: 1px solid #B9DCEA;
                border-radius: 5px;
                color: #163A4A;
                font-size: 15px;
                font-weight: 800;
                padding: 3px;
            }}
        """)


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("GCS")

    window = GroundStationMockup()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()