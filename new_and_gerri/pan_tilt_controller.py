# -*- coding: utf-8 -*-
"""Waveshare 2-Axis Pan-Tilt Camera Module controller.

General Driver for Robots (ESP32) 보드에 JSON 명령을 시리얼(UART/USB)로
전송해서 팬틸트를 제어한다.

주의: CMD_GIMBAL_USER_CTRL(T:141)은 위키 문서에는 있지만 이 보드 펌웨어에서
실제로 동작하지 않는 것으로 확인됨. 대신 실기기에서 검증된
CMD_GIMBAL_CTRL_MOVE(T:134, 연속 이동)와 CMD_GIMBAL_CTRL_STOP(T:135, 정지)를 사용한다.

    CMD_GIMBAL_CTRL_MOVE = {"T":134,"X":x,"Y":y,"SX":sx,"SY":sy}
        x: 목표 pan 각도(-180~180, 양수=오른쪽)
        y: 목표 tilt 각도(-30~90, 양수=위)
        sx/sy: 각 축 이동 속도
        -> 방향키를 누르고 있는 동안 목표 각도(끝까지)로 반복 전송하면
           눌려있는 동안 계속 이동하고, 떼면 STOP(T:135)을 보내 멈춘다.
    CMD_GIMBAL_CTRL_STOP = {"T":135}
"""

from __future__ import annotations

import json
import threading
import time
from typing import Optional

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover
    serial = None


class PanTiltController:
    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200,
                 timeout: float = 0.2, verbose: bool = False,
                 boot_wait: float = 2.0):
        self.port = port
        self.baud = baud
        self.verbose = verbose
        self._lock = threading.Lock()
        self._ser: Optional["serial.Serial"] = None

        if serial is None:
            print("[PanTiltController] pyserial이 설치되어 있지 않습니다. "
                  "'pip install pyserial' 실행 필요.")
            return

        try:
            self._ser = serial.Serial(port, baud, timeout=timeout)
            # pyserial이 포트를 열 때 DTR 토글로 ESP32가 리셋된다.
            # 리셋/부팅이 끝나기 전에 보낸 명령은 씹히므로 대기한다.
            if boot_wait > 0:
                time.sleep(boot_wait)
            print(f"[PanTiltController] connected port={port} baud={baud}")
        except Exception as e:
            print(f"[PanTiltController] serial open error ({port}): {e}")
            self._ser = None

    def _send(self, cmd: dict):
        if self._ser is None:
            if self.verbose:
                print(f"[PanTiltController] (no serial) would send: {cmd}")
            return
        line = (json.dumps(cmd) + "\n").encode("utf-8")
        with self._lock:
            try:
                self._ser.write(line)
                if self.verbose:
                    print(f"[PanTiltController] TX {cmd}")
            except Exception as e:
                print(f"[PanTiltController] write error: {e}")

    # ---- 절대각 제어 (T:133) ----
    def move_to(self, x: float, y: float, speed: int = 0, acc: int = 0):
        x = max(-180, min(180, x))
        y = max(-30, min(90, y))
        self._send({"T": 133, "X": x, "Y": y, "SPD": speed, "ACC": acc})

    # ---- 연속 이동 (T:134) - 방향키 누르는 동안 계속 호출 ----
    def move_continuous(self, x: float, y: float, sx: int = 200, sy: int = 200):
        self._send({"T": 134, "X": x, "Y": y, "SX": sx, "SY": sy})

    # ---- 정지 (T:135) ----
    def stop(self):
        self._send({"T": 135})

    def center(self):
        self.move_to(0, 0)

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
