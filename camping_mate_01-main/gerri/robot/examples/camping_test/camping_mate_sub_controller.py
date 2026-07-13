# -*- coding: utf-8 -*-
import os
import threading
import time

from robot_status import RobotStatus
from gerri.robot.examples.camping_mate.camping_mate_controller import CampingMateController


class CampingMateSubController:
    def __init__(self):
        # ✅ 전원 껐다 켜도 그대로 쓰게: 환경변수로도 조절 가능
        port = os.environ.get("DMD485_PORT", "/dev/rs485_dmd")
        baud = int(os.environ.get("DMD485_BAUD", "19200"))
        dev_id = int(os.environ.get("DMD485_ID", "1"))

        advance_speed = int(os.environ.get("CM_ADVANCE_SPEED", "400"))          # 속도(핵심)
        turn_speed = int(os.environ.get("CM_TURN_SPEED", "200"))   # 좌우 민감도
        steering_speed = int(os.environ.get("CM_STEERING_SPEED", "200"))
        timeout = float(os.environ.get("CM_TIMEOUT", "0.25"))
        debug = os.environ.get("CM_DEBUG", "0") == "1"

        self.robot_controller = CampingMateController(
            port=port,
            baud=baud,
            dev_id=dev_id,
            advance_speed=advance_speed,
            turn_speed=turn_speed,
            steering_speed=steering_speed,
            timeout=timeout,
            debug=debug,
        )

        self.base_controller = None
        self.status = None
        self._lock = threading.Lock()
        self._run = True

    def connect(self):
        self.status = RobotStatus(
            robot_id=self.base_controller.robot_id,
            model=self.base_controller.robot_model,
            category=self.base_controller.robot_category,
        )
        threading.Thread(target=self._update_loop, daemon=True).start()

    def _update_loop(self):
        while self._run:
            with self._lock:
                # 필요하면 status 업데이트(예: 배터리, 위치 등)
                pass
            time.sleep(0.1)

    def disconnect(self):
        self._run = False
        try:
            self.robot_controller.stop()
        finally:
            self.robot_controller.close()

    # ✅ base_controller가 여기로 몰아줌
    def on_message(self, topic, value, option=None, raw=None):
        self.robot_controller.handle_message(topic=topic, value=value, option=option)

    # MobileFunction에서 topic == 'move'로 올 때도 들어올 수 있음
    def move(self, value, option):
        self.robot_controller.handle_message(topic="move", value=value, option=option)

    def handle_joy_command(self, value=None, option=None):
        self.robot_controller.handle_message(topic="/joy", value=value, option=option)

    def hello_universe(self, value):
        self.send_message(value)
        return "hello_universe_by_robot"

    def send_message(self, message):
        self.base_controller.send_message(message)