# -*- coding: utf-8 -*-
import os
import time
import threading
import serial

# ============================================================
ADVANCE_SPEED = 300
REVERSE_SPEED = 150
TURN_SPEED = 200
STEERING_SPEED = 100
SPEED_CHANGE = 50

# 입력 끊김 자동정지(초): 손 떼면 멈추는 민감도
INPUT_TIMEOUT_SEC = 0.35

# 디버그
DEBUG = False

# ===== MDROBOT RS485 프레임 =====
MID_PC = 172
MID_MOTOR = 183

PID_TQ_OFF = 5
PID_RETURN_TYPE = 149
PID_REQ_PID_DATA = 4

PID_PNT_OPEN_VEL_CMD = 208
PID_PNT_MAIN_DATA = 210

# {"axes":[0,0,0,0,0,0],"buttons":[0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0]} 9
# {"axes":[0,0,0,0,0,0],"buttons":[0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0]} 0
# {"axes":[0,0,0,0,0,0],"buttons":[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]} SPACEBAR, A
# {"axes":[0,0,0,0],"buttons":[0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0]} X
# {"axes":[0,0,0,0],"buttons":[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0]} B
# {"axes":[0,0,0,0],"buttons":[0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0]} Y
IDX_JOY_A = 0
IDX_JOY_B = 1
IDX_JOY_X= 2
IDX_JOY_Y = 3
IDX_KEY_SPACEBAR = 0
IDX_KEY_9 = 10
IDX_KEY_0 = 11

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def checksum_twos_complement(payload: bytes) -> int:
    s = sum(payload) & 0xFF
    return ((~s + 1) & 0xFF)


def pack_frame(dev_id: int, pid: int, data: bytes) -> bytes:
    header = bytes([MID_MOTOR, MID_PC, dev_id & 0xFF, pid & 0xFF, len(data) & 0xFF])
    chk = checksum_twos_complement(header + data)
    return header + data + bytes([chk])


def le_i16(v: int) -> bytes:
    return int(v).to_bytes(2, "little", signed=True)


def le_u16(v: int) -> bytes:
    return int(v).to_bytes(2, "little", signed=False)


class MD485:
    """논블로킹 TX 중심(PID208 오픈루프)"""
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0, write_timeout=0,
        )

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def write(self, frame: bytes):
        try:    
            self.ser.write(frame)
        except Exception as e:
            import datetime
            with open("/home/keti/write_err_count.txt", "a") as f:
                f.write(f"{datetime.datetime.now()}|err on camping_mate_controller.py / write : {e}\n")
            print(f"err on camping_mate_controller.py / write : {e}")

    def tq_off(self, dev_id: int):
        self.write(pack_frame(dev_id, PID_TQ_OFF, bytes([0])))

    def set_return_type(self, dev_id: int, ret_type: int):
        self.write(pack_frame(dev_id, PID_RETURN_TYPE, le_u16(ret_type)))

    def req_pid(self, dev_id: int, pid_to_read: int):
        self.write(pack_frame(dev_id, PID_REQ_PID_DATA, bytes([pid_to_read & 0xFF])))

    def pnt_open(self, dev_id: int, out1: int, out2: int, enable1=1, enable2=1, ret_req=0):
        out1 = clamp(int(out1), -1023, 1023)
        out2 = clamp(int(out2), -1023, 1023)
        data = (
            bytes([enable1 & 0xFF]) + le_i16(out1) +
            bytes([enable2 & 0xFF]) + le_i16(out2) +
            bytes([ret_req & 0xFF])
        )
        self.write(pack_frame(dev_id, PID_PNT_OPEN_VEL_CMD, data))


class CampingMateController:
    """
    rubberneck에서 오는 /joy, move를 RS485(PID208 오픈루프)로 변환.
    - 곡선주행: 전/후진 중에는 "한쪽만 느리게" 되도록 turn을 base 크기 안쪽으로 제한
    - 워치독: 일정 시간 입력이 없으면 자동 정지(손 떼면 멈춤)

    ✅ 속도 조절:
      - 위쪽 SPEED_PERCENT(%)만 바꾸면 됨.
        (PWM에서 duty % 바꾸던 것과 동일한 감각)
    """

    def __init__(
        self,
        port="/dev/rs485_dmd",
        baud=19200,
        dev_id=1,
        advance_speed=None,
        reverse_speed=None,
        turn_speed=None,
        steering_speed=None,
        speed_change=None,
        timeout=None,
        debug=DEBUG,
    ):
        self.dev_id = int(dev_id)
        self._prev_buttons = {}

        # ---- % 기반 기본값 계산 ----
        default_advance_speed = clamp(ADVANCE_SPEED, 0, 1023)
        default_reverse_speed = clamp(REVERSE_SPEED, 0, 1023)
        default_turn_speed = clamp(TURN_SPEED, 0, 1023)
        default_steering_speed = clamp(STEERING_SPEED, 0, 1023)
        default_speed_change = clamp(SPEED_CHANGE, 0, 1023)
        default_timeout = float(max(0.05, INPUT_TIMEOUT_SEC))

        # ============================================================

        if (advance_speed is not None) and ("CM_ADVANCE_SPEED" not in os.environ):
            advance_speed = None
        if (reverse_speed is not None) and ("CM_REVERSE_SPEED" not in os.environ):
            reverse_speed = None
        if (turn_speed is not None) and ("CM_TURN_SPEED" not in os.environ):
            turn_speed = None
        if (steering_speed is not None) and ("CM_STEERING_SPEED" not in os.environ):
            steering_speed = None
        if (speed_change is not None) and ("CM_SPEED_CHANGE" not in os.environ):
            speed_change = None
        if (timeout is not None) and ("CM_TIMEOUT" not in os.environ):
            timeout = None

        if advance_speed is None:
            advance_speed = default_advance_speed
        if reverse_speed is None:
            reverse_speed = default_reverse_speed
        if turn_speed is None:
            turn_speed = default_turn_speed
        if steering_speed is None:
            steering_speed = default_steering_speed
        if speed_change is None:
            speed_change = default_speed_change
        if timeout is None:
            timeout = default_timeout

        self.advance_speed = clamp(advance_speed, 0, 1023)
        self.reverse_speed = clamp(reverse_speed, 0, 1023)
        self.turn_speed = clamp(turn_speed, 0, 1023)
        self.steering_speed = clamp(steering_speed, 0, 1023)
        self.speed_change = clamp(speed_change, 0, 1023)
        self.timeout = float(max(0.05, float(timeout)))
        self.debug = bool(debug)
        
        # 최소 변경 원칙: 파라미터 목록은 건드리지 않고 내부 상수로만 적용
        self.md = MD485(port, int(baud))
        self.md.set_return_type(self.dev_id, 0)  # 응답 OFF

        self._lock = threading.Lock()
        self._last_cmd_ts = time.time()
        self._last_out = (0, 0)

        self._stop_evt = threading.Event()
        self._wd_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._wd_thread.start()

        if self.debug:
            print(f"[RS485] OPEN OK: {self.md.ser.name} baud={baud} id={self.dev_id}")
            print(
                f"advance_speed={self.advance_speed}, reverse_speed={self.reverse_speed}, "
                f"turn_speed={self.turn_speed}, steering_speed={self.steering_speed}, "
                f"speed_change={self.speed_change}, timeout={self.timeout}"
            )

    def close(self):
        try:
            self.stop()
        finally:
            self._stop_evt.set()
            try:
                self.md.close()
            except Exception:
                pass

    def tq_off(self):
        with self._lock:
            self.md.tq_off(self.dev_id)
            self._last_out = (0, 0)

    def stop(self):
        self.set_openloop(0, 0)

    def set_openloop(self, out_l: int, out_r: int):

        with self._lock:
            self.md.pnt_open(self.dev_id, out_l, out_r, enable1=1, enable2=1, ret_req=0)
            self._last_out = (out_l, out_r)
            self._last_cmd_ts = time.time()

    def _watchdog_loop(self):
        # 입력 끊기면 자동정지(손 떼면 멈춤)
        while not self._stop_evt.is_set():
            time.sleep(0.02)
            with self._lock:
                dt = time.time() - self._last_cmd_ts
                if dt > self.timeout and self._last_out != (0, 0):
                    self.md.pnt_open(self.dev_id, 0, 0, enable1=1, enable2=1, ret_req=0)
                    self._last_out = (0, 0)

    def _apply_params_from_option(self, option):
        """option으로 실시간 파라미터 변경 가능(없으면 무시)"""
        if not isinstance(option, dict):
            return
        if "advance_speed" in option:
            self.advance_speed = clamp(int(option["advance_speed"]), 0, 1023)
        if "reverse_speed" in option:
            self.advance_speed = clamp(int(option["reverse_speed"]), 0, 1023)
        if "turn_speed" in option:
            self.turn_speed = clamp(int(option["turn_speed"]), 0, 1023)
        if "steering_speed" in option:
            self.steering_speed = clamp(int(option["steering_speed"]), 0, 1023)
        if "timeout" in option:
            self.timeout = max(0.05, float(option["timeout"]))

    def move(self, vx, vy, vth):
        if vx < 0:
            base = int(-vx * self.advance_speed)
        else:
            base = int(-vx * self.reverse_speed)

        if vx != 0:
            turn = int(-vy * self.steering_speed)
        else:
            turn = int(-vy * self.turn_speed)

        self._mix_and_send(base, turn)

    def slower(self):
        self.advance_speed -= SPEED_CHANGE
        self.advance_speed = clamp(self.advance_speed, 0, 1023)

    def faster(self):
        self.advance_speed += SPEED_CHANGE
        self.advance_speed = clamp(self.advance_speed, 0, 1023)

    def handle_message(self, topic: str, value, option=None):
        """
        topic:
          /joy: value={"axes":[x,y,...], "buttons":[...]}  (x=좌우, y=앞뒤)
          move: value={"vx":-1~1, "vth":-1~1} 또는 {"x":..,"y":..}
          /open: value={"l":int, "r":int}
          stop: value 무시
        """
        self._apply_params_from_option(option)

        if topic in ("/stop", "stop"):
            self.stop()
            return

        if topic in ("/open", "open"):
            if isinstance(value, dict):
                l = int(value.get("l", 0))
                r = int(value.get("r", 0))
            else:
                l, r = 0, 0
            self.set_openloop(l, r)
            return

        if topic in ("/move", "move"):
            if not isinstance(value, dict):
                self.stop()
                return
            vx = float(value.get("vx", value.get("y", 0.0)))
            vth = float(value.get("vth", value.get("x", 0.0)))
            base = int(clamp(vx, -1.0, 1.0) * self.advance_speed)
            turn = int(clamp(vth, -1.0, 1.0) * self.turn_speed)
            self._mix_and_send(base, turn)
            return

        if topic in ("/joy", "joy"): # 키보드, 조이스틱
            if not isinstance(value, dict):
                self.stop()
                return
            axes = value.get("axes", [])
            buttons = value.get("buttons", [])

            if self._btn(buttons, IDX_KEY_9) or self._btn(buttons, IDX_JOY_Y):
                self.advance_speed += SPEED_CHANGE
                self.advance_speed = clamp(self.advance_speed, 0, 1023)
            
            if self._btn(buttons, IDX_KEY_0) or self._btn(buttons, IDX_JOY_B):
                self.advance_speed -= SPEED_CHANGE
                self.advance_speed = clamp(self.advance_speed, 0, 1023)
            
            # if self._btn(buttons, IDX_KEY_SPACEBAR):

            x = clamp(float(axes[0]), -1.0, 1.0) if len(axes) > 0 else 0.0   # 좌(-) 우(+)
            y = clamp(float(axes[1]), -1.0, 1.0) if len(axes) > 0 else 0.0   # 위(-) 아래(+)
            if y < 0:
                base = -y * self.advance_speed   # 위(-1)=전진
            else:
                base = -y * self.reverse_speed

            if y != 0:
                turn = -x * self.steering_speed
            else:
                turn = -x * self.turn_speed

            self._mix_and_send(base, turn)
            return

    def _mix_and_send(self, base: int, turn: int):
        """
        곡선주행:
        - base != 0(전/후진 중)엔 turn이 base를 뒤집지 않게 제한해서
          "한쪽 바퀴만 느리게/빠르게"가 되도록 만든다.
        """
        if base != 0:
            left = base + turn
            right = base - turn
        else:
            # 제자리 회전
            left = +turn
            right = -turn

        self.set_openloop(left,right)
    
    def _btn(self, buttons, idx):
        cur = len(buttons) > idx and buttons[idx] == 1
        prev = self._prev_buttons.get(idx, False)
        self._prev_buttons[idx] = cur
        return cur and not prev