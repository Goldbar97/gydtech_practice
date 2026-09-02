# -*- coding: utf-8 -*-
"""
remove_snow robot configuration.

이 파일은 gerri/robot/examples/remove_snow/ 폴더 안에서 직접 import된다.
ROBOT_INFO의 id/api_key는 rubberneck에 등록한 로봇 정보로 맞춰야 한다.
"""

import os

ROBOT_INFO = {
    "id": "snow_02",
    "robot_client_id": "snow_02",
    "model": "snow",
    "category": "mobile",
    "api_key": "27987d8dad9756fea274c53b4ea4c8d8",
}
VIDEO_INFO = {
    "front_cam": {"source": 0, "width": 640, "height": 480},
    "rear":      {"source": 2, "width": 640, "height": 480},
}


AUDIO_INFO = {
    "audio": {
        "input": os.getenv("SNOW_AUDIO_INPUT", "PowerConf"),
        "output": os.getenv("SNOW_AUDIO_OUTPUT", "PowerConf"),
    },
}

FY_CONFIG = {
    "port": os.getenv("FY_PORT", "/dev/ttyACM0"),
    "baud": int(os.getenv("FY_BAUD", "115200")),
    "left_addr": int(os.getenv("FY_LEFT_ADDR", "0x11"), 0),
    "right_addr": int(os.getenv("FY_RIGHT_ADDR", "0x12"), 0),
    "pwm": int(os.getenv("FY_PWM", "300")),
    "slow_pwm": int(os.getenv("FY_SLOW_PWM", "50")),
    "current_a": float(os.getenv("FY_CURRENT_A", "8.0")),
    "input_timeout": float(os.getenv("FY_INPUT_TIMEOUT", "0.15")),
    "verbose": os.getenv("FY_VERBOSE", "0") == "1",
}

# 팬틸트 카메라 짐벌 (Waveshare 2-Axis Pan-Tilt, ESP32 General Driver, UART/JSON)
# 모터(FY_CONFIG)와 완전히 별개의 시리얼 포트를 사용한다.
GIMBAL_CONFIG = {
    "port": os.getenv("GIMBAL_PORT", "/dev/ttyUSB1"),
    "baud": int(os.getenv("GIMBAL_BAUD", "115200")),
    "deg_per_sec": float(os.getenv("GIMBAL_DEG_PER_SEC", "60.0")),
    # BACK(버튼6)/START(버튼7)로 조절하는 회전속도 범위/step
    "speed_step": float(os.getenv("GIMBAL_SPEED_STEP", "15.0")),
    "speed_min": float(os.getenv("GIMBAL_SPEED_MIN", "15.0")),
    "speed_max": float(os.getenv("GIMBAL_SPEED_MAX", "135.0")),
    "x_min": float(os.getenv("GIMBAL_X_MIN", "-135")),
    "x_max": float(os.getenv("GIMBAL_X_MAX", "135")),
    "y_min": float(os.getenv("GIMBAL_Y_MIN", "-30")),
    "y_max": float(os.getenv("GIMBAL_Y_MAX", "90")),
    "enabled": os.getenv("GIMBAL_ENABLED", "1") == "1",
}
