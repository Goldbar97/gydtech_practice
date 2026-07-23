#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
from camping_mate_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO
from gerri.utils.common_base_controller import BaseController
from camping_mate_sub_controller import CampingMateSubController
from utils.rubberneck_joystick_handler import RubberNeckJoyStickHandler

daemon = AdaptiveNetworkDaemon(
    robot_info=ROBOT_INFO,
    network="ketirtc",
    command="command",
    video_info=VIDEO_INFO,
    audio_info=AUDIO_INFO,
)
daemon.connect()

sub = CampingMateSubController(ROBOT_INFO)
joy_handler = RubberNeckJoyStickHandler(sub_controller=sub)
joy_handler.connect()

base = BaseController(ROBOT_INFO, sub_controller_list=[sub])
base.joystick_message_handler.append(joy_handler)
base.connect()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    try:
        base.disconnect()
    except Exception:
        pass