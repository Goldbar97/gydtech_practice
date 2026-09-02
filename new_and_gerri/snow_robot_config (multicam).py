# -*- coding: utf-8 -*-
import os
import time
import threading
import cv2
import numpy as np

from _and_.keti_rtc.robot.cam_manager import CameraManager

ROBOT_INFO = {
    "id": "snow_02",
    "robot_client_id": "snow_02",
    "model": "snow",
    "category": "mobile",
    "api_key": "27987d8dad9756fea274c53b4ea4c8d8",
}


class CombinedCameraManager:
    """
    3개 카메라(front, rear, side)를 하나의 프레임으로 합성.
    layout="horizontal": 가로로 3개 나란히
    layout="grid": 2x2 그리드 (4번째 칸은 빈칸)
    """
    def __init__(self, cam_indices, cell_width=640, cell_height=480, fps=30,
                 layout="horizontal", camera_name="combined_cam"):
        self.camera_name = camera_name
        self.cell_width = cell_width
        self.cell_height = cell_height
        self.layout = layout

        # cam_indices: {"front": 0, "rear": 2, "side": 4} 같은 dict
        self.cams = {}
        for name, index in cam_indices.items():
            self.cams[name] = CameraManager(
                camera_name=f"{name}_cam", camera_index=index,
                width=cell_width, height=cell_height, fps=fps
            )
            time.sleep(0.5)  # 순차적으로 안정적으로 열기

        n = len(self.cams)
        if layout == "horizontal":
            self.width = cell_width * n
            self.height = cell_height
        elif layout == "grid":
            cols = 2
            rows = (n + 1) // 2
            self.width = cell_width * cols
            self.height = cell_height * rows

        self.last_frame = None
        self.running = True
        self._thread = threading.Thread(target=self._merge_loop, daemon=True)
        self._thread.start()

    def _merge_loop(self):
        blank = np.zeros((self.cell_height, self.cell_width, 3), dtype="uint8")
        names = list(self.cams.keys())

        while self.running:
            frames = []
            for name in names:
                f = self.cams[name].get_frame()
                if f is None:
                    f = blank
                elif f.shape[:2] != (self.cell_height, self.cell_width):
                    f = cv2.resize(f, (self.cell_width, self.cell_height))
                frames.append(f)

            if self.layout == "horizontal":
                combined = cv2.hconcat(frames)
            elif self.layout == "grid":
                # 2열 그리드로 배치, 부족한 칸은 검은 화면으로 채움
                cols = 2
                rows = (len(frames) + 1) // 2
                padded = frames + [blank] * (rows * cols - len(frames))
                row_imgs = []
                for r in range(rows):
                    row_imgs.append(cv2.hconcat(padded[r*cols:(r+1)*cols]))
                combined = cv2.vconcat(row_imgs)

            self.last_frame = combined
            time.sleep(1 / 30)

    def get_frame(self):
        return self.last_frame

    def get_camera(self, name):
        """개별 카메라 인스턴스를 다른 채널에서 재사용하고 싶을 때 사용"""
        return self.cams.get(name)

    def stop(self):
        self.running = False
        for cam in self.cams.values():
            cam.stop()


_combined = CombinedCameraManager(
    cam_indices={
        "front": 0,   # C922 (85E6386F)
        "rear": 2,    # C922 (BAB5386F)
        "side": 4,    # Sonix USB 2.0 Camera
    },
    cell_width=640,
    cell_height=480,
    layout="horizontal",   # 3개를 가로로 나란히. 2x2로 하려면 "grid"
)

VIDEO_INFO = {
    "front_cam": {"source": _combined},   # 3개 합성 화면 (원격 페이지가 인식하는 이름 유지)
    "rear": {"source": _combined.get_camera("side")},
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
