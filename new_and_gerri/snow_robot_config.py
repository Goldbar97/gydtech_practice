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
    "front_cam": {"source": 1, "width": 640, "height": 480},
    "rear":      {"source": 3, "width": 640, "height": 480},
}


AUDIO_INFO = {
    "audio": {
        "input": os.getenv("SNOW_AUDIO_INPUT", "PowerConf"),
        "output": os.getenv("SNOW_AUDIO_OUTPUT", "PowerConf"),
    },
}

FY_CONFIG = {
    "port": os.getenv("FY_PORT", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7B022613-if00"),
    "baud": int(os.getenv("FY_BAUD", "115200")),
    "left_addr": int(os.getenv("FY_LEFT_ADDR", "0x11"), 0),
    "right_addr": int(os.getenv("FY_RIGHT_ADDR", "0x12"), 0),
    "pwm": int(os.getenv("FY_PWM", "300")),
    "slow_pwm": int(os.getenv("FY_SLOW_PWM", "50")),
    "current_a": float(os.getenv("FY_CURRENT_A", "8.0")),
    # C1/C2 버튼으로 조절하는 주행 속도(pwm) 범위와 step
    "pwm_step": int(os.getenv("FY_PWM_STEP", "100")),
    "pwm_min": int(os.getenv("FY_PWM_MIN", "100")),
    "pwm_max": int(os.getenv("FY_PWM_MAX", "1000")),
    "input_timeout": float(os.getenv("FY_INPUT_TIMEOUT", "0.15")),
    "verbose": os.getenv("FY_VERBOSE", "0") == "1",
}

GIMBAL_CONFIG = {
    "port": os.getenv("GIMBAL_PORT", "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_227909a9bc72ef11b263744f8fcc3fa0-if00-port0"),
    "baud": int(os.getenv("GIMBAL_BAUD", "115200")),
    "deg_per_sec": float(os.getenv("GIMBAL_DEG_PER_SEC", "60.0")),
    "speed_step": float(os.getenv("GIMBAL_SPEED_STEP", "15.0")),
    "speed_min": float(os.getenv("GIMBAL_SPEED_MIN", "15.0")),
    "speed_max": float(os.getenv("GIMBAL_SPEED_MAX", "135.0")),
    "x_min": float(os.getenv("GIMBAL_X_MIN", "-135")),
    "x_max": float(os.getenv("GIMBAL_X_MAX", "135")),
    "y_min": float(os.getenv("GIMBAL_Y_MIN", "-30")),
    "y_max": float(os.getenv("GIMBAL_Y_MAX", "90")),
    "enabled": os.getenv("GIMBAL_ENABLED", "1") == "1",
}
