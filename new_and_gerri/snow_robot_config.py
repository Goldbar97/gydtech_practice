# -*- coding: utf-8 -*-
"""
remove_snow robot configuration.

이 파일은 gerri/robot/examples/remove_snow/ 폴더 안에서 직접 import된다.
ROBOT_INFO의 id/api_key는 rubberneck에 등록한 로봇 정보로 맞춰야 한다.
"""

import os

# ROBOT_INFO = {
#     "id": "snow_01",
#     "robot_client_id": "snow_01",
#     "model": "snow",
#     "category": "mobile",
#     "api_key": "f7b9b13ec594bfb315bdbfb618d2b22b",
# }
ROBOT_INFO = {
    "id": "camping_test_01",
    "robot_client_id": "camping_test_01",
    "model": "snow",
    "category": "mobile",
    "api_key": "31870cb7bc4e2a064b4eb9d649fba210",
}
VIDEO_INFO = {
    "front_cam": {"source": 0, "width": 640, "height": 480},
    "rear":      {"source": 2, "width": 640, "height": 480},
}


AUDIO_INFO = {
    "audio": {
        "input": os.getenv("SNOW_AUDIO_INPUT", "default"),
        "output": os.getenv("SNOW_AUDIO_OUTPUT", "default"),
    },
}

FY_CONFIG = {
    "port": os.getenv("FY_PORT", "/dev/ttyACM0"),
    "baud": int(os.getenv("FY_BAUD", "115200")),
    "left_addr": int(os.getenv("FY_LEFT_ADDR", "0x11"), 0),
    "right_addr": int(os.getenv("FY_RIGHT_ADDR", "0x12"), 0),
    "pwm": int(os.getenv("FY_PWM", "300")),
    "slow_pwm": int(os.getenv("FY_SLOW_PWM", "160")),
    "current_a": float(os.getenv("FY_CURRENT_A", "8.0")),
    "input_timeout": float(os.getenv("FY_INPUT_TIMEOUT", "0.15")),
    "verbose": os.getenv("FY_VERBOSE", "0") == "1",
}
