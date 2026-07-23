#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Robot-side entry point for remove_snow FY-C100 robot.

Operator 쪽 파일은 수정하지 않고, robot 쪽에서 기존 camping_mate/GERRI 메시지를 받아
FY-C100 RS485 모터 제어로 연결한다.
"""

import os
import sys
import time

# 현재 파일 위치:
#   <PROJECT_ROOT>/gerri/robot/examples/remove_snow/snow_robot_robot.py
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
# PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "../../../.."))
print(CURRENT_DIR)
# print(PROJECT_ROOT)

# if PROJECT_ROOT not in sys.path:
    # sys.path.insert(0, PROJECT_ROOT)
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from _and_.and_robot import AdaptiveNetworkDaemon

from snow_robot_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO, FY_CONFIG

# ==============================================================================
# 기존 BaseController 구현에서 패키지 내에 포함된 CommonBaseController 사용으로대체
# ==============================================================================
# from snow_robot_base_controller import SnowRobotBaseController
from gerri.utils.common_base_controller import BaseController

from snow_robot_sub_controller import SnowRobotSubController

# def main():
print("[remove_snow] starting")
# print(f"[remove_snow] project root = {PROJECT_ROOT}")
print(f"[remove_snow] robot id     = {ROBOT_INFO.get('id')}")
print("[remove_snow] VIDEO_INFO =", VIDEO_INFO)

daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network="ketirtc",
    command="command",
    video_info=VIDEO_INFO,
    audio_info=AUDIO_INFO,
)
daemon.connect()

# =============================================================================
# BaseController에 Subcontroller 주입 / RubberNeck Joystick handler register
# =============================================================================
sub_controller = SnowRobotSubController(robot_info=ROBOT_INFO, fy_config=FY_CONFIG) # subcontroller get ROBOT_INFO
from utils.rubberneck_joystick_handler import RubberNeckJoyStickHandler

joy_handler = RubberNeckJoyStickHandler(sub_controller=sub_controller)
joy_handler.connect()

base_controller = BaseController(ROBOT_INFO, sub_controller_list=sub_controller)
base_controller.joystick_message_handler.append(joy_handler)
base_controller.connect()

print("[remove_snow] running. Ctrl+C to stop.")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[remove_snow] Ctrl+C")
finally:
    try:
        base_controller.disconnect()
    except Exception as e:
        print(f"[remove_snow] disconnect error: {e}")


# if __name__ == "__main__":
#     main()
