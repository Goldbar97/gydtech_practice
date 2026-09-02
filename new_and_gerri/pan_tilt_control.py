"""
Waveshare 2-Axis Pan-Tilt Camera Module 제어 스크립트 (Jetson, Python)

이 팬틸트는 ESP32 기반 'General Driver for Robots' 보드를 통해
UART(시리얼)로 연결되며, JSON 문자열 명령을 한 줄씩(개행 포함) 보내서 제어합니다.
라즈베리파이 전용 프로토콜이 아니라 순수 시리얼 통신이므로 Jetson에서도 동일하게 동작합니다.

필요 패키지:
    pip install pyserial

연결 방법 (둘 중 하나):
    1) USB-시리얼: 보드의 USB(Type-C, ESP32 UART용) 포트를 Jetson USB에 연결
       -> 보통 /dev/ttyUSB0 로 잡힘
    2) GPIO UART: 40핀 헤더의 UART TX/RX/GND를 Jetson 40핀 헤더에 직결
       -> Jetson 모델/JetPack 버전에 따라 /dev/ttyTHS1, /dev/ttyTHS0 등으로 잡힘
          (`ls /dev/ttyTHS*` `ls /dev/ttyUSB*` 로 확인)

권한 문제 시:
    sudo usermod -aG dialout $USER   (재로그인 필요)
"""

import json
import time
import threading
import serial


class PanTiltController:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # ESP32 리셋/부팅 대기
        self._lock = threading.Lock()

    def close(self):
        self.ser.close()

    def _send(self, cmd: dict):
        """JSON 명령을 한 줄로 만들어 전송"""
        line = json.dumps(cmd) + "\n"
        with self._lock:
            self.ser.write(line.encode("utf-8"))

    def read_response(self, wait=0.1):
        """보드에서 오는 응답(있다면) 읽기 - 필요할 때만 사용"""
        time.sleep(wait)
        data = b""
        while self.ser.in_waiting:
            data += self.ser.read(self.ser.in_waiting)
        return data.decode("utf-8", errors="ignore")

    # ---- 기본 절대각 제어 (CMD_GIMBAL_CTRL_SIMPLE, T=133) ----
    def move_to(self, x: float, y: float, speed: int = 0, acc: int = 0):
        """
        x: 좌우(Pan) 각도, -180 ~ 180 (양수=오른쪽)
        y: 상하(Tilt) 각도, -30 ~ 90 (양수=위)
        speed/acc: 0이면 최대 속도/가속도
        """
        x = max(-180, min(180, x))
        y = max(-30, min(90, y))
        self._send({"T": 133, "X": x, "Y": y, "SPD": speed, "ACC": acc})

    # ---- 연속 속도 제어 (CMD_GIMBAL_CTRL_MOVE, T=134) ----
    def move_continuous(self, x: float, y: float, sx: int = 300, sy: int = 300):
        self._send({"T": 134, "X": x, "Y": y, "SX": sx, "SY": sy})

    # ---- 정지 (CMD_GIMBAL_CTRL_STOP, T=135) ----
    def stop(self):
        self._send({"T": 135})

    # ---- 자체 안정화 (CMD_GIMBAL_STEADY, T=137) ----
    def set_steady(self, enable: bool, target_y: float = 0):
        self._send({"T": 137, "s": 1 if enable else 0, "y": target_y})

    # ---- LED 밝기 제어 (CMD_LED_CTRL, T=132) ----
    def set_led(self, brightness: int):
        """brightness: 0~255"""
        brightness = max(0, min(255, brightness))
        self._send({"T": 132, "IO4": brightness, "IO5": brightness})

    # ---- 중앙 정렬 ----
    def center(self):
        self.move_to(0, 0)


if __name__ == "__main__":
    # 사용 예시
    pt = PanTiltController(port="/dev/ttyUSB1", baudrate=115200)

    try:
        print("중앙 정렬")
        pt.center()
        time.sleep(1.5)

        print("오른쪽 45도, 위쪽 20도로 이동")
        pt.move_to(45, 20, speed=50, acc=10)
        time.sleep(1.5)

        print("왼쪽으로 스캔")
        pt.move_to(-90, 0, speed=30, acc=10)
        time.sleep(2)

        print("LED 켜기")
        pt.set_led(150)
        time.sleep(1)

        print("LED 끄기")
        pt.set_led(0)

        print("중앙으로 복귀")
        pt.center()

    finally:
        pt.close()
