# -*- coding: utf-8 -*-
import os
import threading
import time

from robot_status import RobotStatus
from camping_mate_controller import CampingMateController

class CampingMateSubController:
    def __init__(self, robot_info):
        # ✅ 전원 껐다 켜도 그대로 쓰게: 환경변수로도 조절 가능
        port = os.environ.get("DMD485_PORT", "/dev/rs485_dmd")
        baud = int(os.environ.get("DMD485_BAUD", "19200"))
        dev_id = int(os.environ.get("DMD485_ID", "1"))

        advance_speed = int(os.environ.get("CM_ADVANCE_SPEED", "300"))          # 속도(핵심)
        reverse_speed = int(os.environ.get("CM_REVERSE_SPEED", "150"))
        turn_speed = int(os.environ.get("CM_TURN_SPEED", "200"))   # 좌우 민감도
        steering_speed = int(os.environ.get("CM_STEERING_SPEED", "200"))
        speed_change = int(os.environ.get("CM_SPEED_CHANGE", "50"))
        timeout = float(os.environ.get("CM_TIMEOUT", "0.25"))
        debug = os.environ.get("CM_DEBUG", "0") == "1"

        self.robot_controller = CampingMateController(
            port=port,
            baud=baud,
            dev_id=dev_id,
            advance_speed=advance_speed,
            reverse_speed=reverse_speed,
            turn_speed=turn_speed,
            steering_speed=steering_speed,
            speed_change=speed_change,
            timeout=timeout,
            debug=debug,
        )

        self.base_controller = None
        self.status = RobotStatus(robot_info.get("id"), robot_info.get("model"), robot_info.get("category"))
        self._run = True

    def connect(self):
        threading.Thread(target=self._update_loop, daemon=True).start()

    def _update_loop(self):
        while self._run:
            time.sleep(0.1)

    def disconnect(self):
        self._run = False
        try:
            self.robot_controller.stop()
        finally:
            self.robot_controller.close()

    def move(self, vx, vy, vth):
        self.robot_controller.move(vx, vy, vth)

    def slower(self):
        self.robot_controller.slower()

    def faster(self):
        self.robot_controller.faster()

    def hello_universe(self, value):
        self.send_message(value)
        return "hello_universe_by_robot"

    def send_message(self, message):
        self.base_controller.send_message(message)