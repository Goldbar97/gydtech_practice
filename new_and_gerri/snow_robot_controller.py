#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FY-C100 RS485 motor controller + USB relay controller for remove_snow / and_gerri.

Motor:
- USB-RS485: /dev/ttyACM0
- baud: 115200 8N1
- left addr  = 0x11
- right addr = 0x12
- dir 0 = forward
- dir 1 = backward
- FY-C100 C1 frame 반복 송신

Relay:
- USB relay: CH340/CH341 serial converter
- default auto detect: /dev/ttyUSB* first, excluding motor port
- direct override: RELAY_PORT=/dev/ttyUSB0
- default baud: 9600
- ON     : A0 01 01 A2
- OFF    : A0 01 00 A1
- STATUS : FF

Rules:
- w/a/s/d : motor
- Space : relay toggle only
- Space first press  -> relay ON latch
- Space second press -> relay OFF latch
"""

from __future__ import annotations

import glob
import logging
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional, Set, Tuple

import serial

from snow_robot_log import log, log_every


# ============================================================
# Motor defaults
# ============================================================

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200

LEFT_ADDR = 0x11
RIGHT_ADDR = 0x12

DIR_FORWARD = 0
DIR_BACKWARD = 1

DEFAULT_PWM = 300
DEFAULT_SLOW_PWM = 160
DEFAULT_CURRENT_A = 8.0

SEND_INTERVAL = 0.05
PER_ADDR_GAP = 0.035
RX_WAIT = 0.04

MOTOR_KEYS = {"w", "a", "s", "d"}
RELAY_KEY_ALIASES = {
    "relay_toggle",
    "relay_toggle_pressed",
    "space",
    "spacebar",
    "key_space",
    "keyboard_space",
    " ",
}
CONTROL_KEYS = MOTOR_KEYS | {"relay_toggle", "relay_toggle_pressed"}


# ============================================================
# Relay defaults
# ============================================================

RELAY_ON_FRAME = bytes([0xA0, 0x01, 0x01, 0xA2])
RELAY_OFF_FRAME = bytes([0xA0, 0x01, 0x00, 0xA1])
RELAY_STATUS_FRAME = bytes([0xFF])

DEFAULT_RELAY_BAUD = 9600


def hx(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, int(value)))


# ============================================================
# FY-C100 frame helpers
# ============================================================

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def add_crc(payload: bytes) -> bytes:
    crc = crc16_modbus(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def make_c0_status(addr: int) -> bytes:
    return add_crc(bytes([addr & 0xFF, 0x06, 0xC0, 0x00]))


def make_c1(addr: int, run: int, direction: int, pwm: int, current_a: float) -> bytes:
    pwm = clamp(pwm, 0, 1000)
    cur = clamp(round(float(current_a) * 100), 0, 0xFFFF)

    payload = bytes([
        addr & 0xFF,
        0x0C,
        0xC1,
        run & 0xFF,
        direction & 0xFF,
        (pwm >> 8) & 0xFF,
        pwm & 0xFF,
        (cur >> 8) & 0xFF,
        cur & 0xFF,
        0x00,
    ])
    return add_crc(payload)


def make_run(addr: int, direction: int, pwm: int, current_a: float) -> bytes:
    return make_c1(addr=addr, run=1, direction=direction, pwm=pwm, current_a=current_a)


def make_stop(addr: int, current_a: float) -> bytes:
    return make_c1(addr=addr, run=0, direction=DIR_FORWARD, pwm=0, current_a=current_a)


# ============================================================
# Config
# ============================================================

@dataclass
class FYConfig:
    # motor
    port: str = DEFAULT_PORT
    baud: int = DEFAULT_BAUD
    left_addr: int = LEFT_ADDR
    right_addr: int = RIGHT_ADDR
    pwm: int = DEFAULT_PWM
    slow_pwm: int = DEFAULT_SLOW_PWM
    current_a: float = DEFAULT_CURRENT_A
    input_timeout: float = 0.70
    verbose: bool = False

    # 주행 속도 조절 (C1/C2 버튼): pwm을 step씩 올리고 내리며 min~max로 제한한다.
    pwm_step: int = 100
    pwm_min: int = 100
    pwm_max: int = 1000

    # relay
    relay_enabled: bool = True
    relay_port: Optional[str] = None
    relay_baud: int = DEFAULT_RELAY_BAUD
    relay_verbose: bool = True
    # 자동탐지 시 릴레이로 오인하면 안 되는 추가 포트들 (예: 팬틸트 짐벌 포트).
    relay_exclude_ports: Tuple[str, ...] = ()


# ============================================================
# USB Relay
# ============================================================

def list_serial_ports():
    ports = []
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        ports.extend(glob.glob(pattern))
    return sorted(set(ports))


def auto_find_relay_port(motor_port: str, exclude_ports: Optional[Tuple[str, ...]] = None) -> Optional[str]:
    """릴레이 포트 자동 탐색.

    모터 RS485 포트와 exclude_ports(예: 팬틸트 짐벌 포트)는 제외한다.
    /dev/ttyUSB*를 우선 선택한다.
    """
    # /dev/serial/by-id/... 같은 심볼릭 링크로 지정해도 실제 장치(/dev/ttyUSB0 등)로 비교한다.
    exclude = {os.path.realpath(motor_port)}
    if exclude_ports:
        exclude |= {os.path.realpath(p) for p in exclude_ports}

    ports = [p for p in list_serial_ports() if os.path.realpath(p) not in exclude]

    if not ports:
        return None

    usb_ports = [p for p in ports if os.path.basename(p).startswith("ttyUSB")]
    if usb_ports:
        return usb_ports[0]

    return ports[0]


class RelayUSB:
    """USB relay serial controller."""

    def __init__(self, port: str, baud: int = DEFAULT_RELAY_BAUD, verbose: bool = True):
        self.port = port
        self.baud = int(baud)
        self.verbose = bool(verbose)
        self.state: Optional[bool] = None
        self.lock = threading.Lock()

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.10,
            write_timeout=1.0,
        )

        time.sleep(0.10)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        print(f"[RelayUSB] opened port={self.port} baud={self.baud}")

    def _send(self, frame: bytes, read_wait: float = 0.05) -> bytes:
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            self.ser.flush()
            time.sleep(read_wait)
            rx = self.ser.read(self.ser.in_waiting or 0)

        if self.verbose:
            print(f"[RelayUSB] TX={hx(frame)} RX={hx(rx) if rx else '<no response>'}")

        return rx

    def set_state(self, on: bool):
        """on=True면 릴레이 ON, on=False면 릴레이 OFF."""
        on = bool(on)

        if self.state is on:
            return

        if on:
            self._send(RELAY_ON_FRAME)
            self.state = True
            print("[RelayUSB] ON")
        else:
            self._send(RELAY_OFF_FRAME)
            self.state = False
            print("[RelayUSB] OFF")

    def on(self):
        self.set_state(True)

    def off(self):
        self.set_state(False)

    def status(self) -> bytes:
        rx = self._send(RELAY_STATUS_FRAME, read_wait=0.08)
        print(f"[RelayUSB] STATUS RX={hx(rx) if rx else '<no response>'}")
        return rx

    def close(self):
        # 종료 시 강제 OFF하지 않는다.
        try:
            self.ser.close()
        finally:
            print("[RelayUSB] closed")


def parse_feedback(rx: bytes, addr: int) -> Optional[dict]:
    """C0/C1 응답(0xD0/0xD1, 22바이트)을 해석한다. 못 찾거나 CRC가 틀리면 None.

    프레임: addr, 0x16, cmd, Val1(버전), Val2(시스템 상태), Val3(모드), Val4(시작/정지),
            Val5(방향), Val6-7(최대 PWM), Val8-9(실제 PWM), Val10-11(최대 전류, 10mA),
            Val12-13(실제 전류, 10mA), Val14-15(전원 전압, 10mV), Val16(온도, 오프셋 40), Val17, CRC_L, CRC_H
    시스템 상태: > 0x80 정상, < 0x30 오류 코드.
    """
    n = len(rx)
    for i in range(0, n - 21):
        if rx[i] == (addr & 0xFF) and rx[i + 1] == 0x16 and rx[i + 2] in (0xD0, 0xD1):
            f = rx[i:i + 22]
            crc = crc16_modbus(f[:20])
            if f[20] != (crc & 0xFF) or f[21] != ((crc >> 8) & 0xFF):
                continue
            return {
                "status": f[4],
                "ok": f[4] > 0x80,
                "running": f[6],
                "pwm": (f[10] << 8) | f[11],
                "current_a": ((f[14] << 8) | f[15]) / 100.0,
                "voltage_v": ((f[16] << 8) | f[17]) / 100.0,
                "temp_c": f[18] - 40,
                "time": time.time(),
            }
    return None


# ============================================================
# FY-C100 motor drive
# ============================================================

class FYDrive:
    def __init__(self, config: FYConfig):
        self.left_addr = int(config.left_addr)
        self.right_addr = int(config.right_addr)
        self.pwm = clamp(config.pwm, 0, 1000)
        self.slow_pwm = clamp(config.slow_pwm, 0, 1000)
        self.current_a = float(config.current_a)
        self.verbose = bool(config.verbose)
        self.lock = threading.Lock()
        self.last_pair: Optional[Tuple[int, int, int, int]] = None

        # 모터 컨트롤러 응답 감시 (주소별). C1 명령마다 22바이트 상태 응답이 온다.
        self.no_resp_limit = int(os.getenv("FY_NO_RESP_LIMIT", "10"))  # 연속 무응답 허용 횟수
        self.fb = {
            self.left_addr: {"name": "LEFT", "fails": 0, "down": False, "last": None, "bad_status": None},
            self.right_addr: {"name": "RIGHT", "fails": 0, "down": False, "last": None, "bad_status": None},
        }

        self.ser = serial.Serial(
            port=config.port,
            baudrate=int(config.baud),
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=1.0,
        )

        time.sleep(0.15)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        print(
            f"[FYDrive] opened port={config.port} baud={config.baud} "
            f"left={hex(self.left_addr)} right={hex(self.right_addr)} "
            f"pwm={self.pwm} slow_pwm={self.slow_pwm} current={self.current_a}A"
        )

    def send_and_read(self, frame: bytes, wait: float = RX_WAIT) -> bytes:
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            self.ser.flush()
            time.sleep(wait)
            rx = self.ser.read(self.ser.in_waiting or 0)

        if self.verbose:
            print(f"TX: {hx(frame)}")
            print(f"RX: {hx(rx) if rx else '<no response>'}")

        return rx

    def status_one(self, addr: int) -> bytes:
        return self.send_and_read(make_c0_status(addr), wait=0.05)

    def print_status(self):
        for name, addr in (("LEFT", self.left_addr), ("RIGHT", self.right_addr)):
            rx = self.status_one(addr)
            print(f"[FYDrive] {name} addr={hex(addr)} RX={hx(rx) if rx else '<no response>'}")
            time.sleep(PER_ADDR_GAP)

    def send_wheel(self, addr: int, direction: int, pwm: int):
        if pwm <= 0:
            frame = make_stop(addr, self.current_a)
        else:
            frame = make_run(addr, direction, pwm, self.current_a)
        rx = self.send_and_read(frame)
        self._note_feedback(addr, rx)

    def _note_feedback(self, addr: int, rx: bytes):
        """응답을 해석해 무응답 / 복구 / 컨트롤러 오류 상태를 기록한다 (상태가 바뀔 때만)."""
        st = self.fb.get(addr)
        if st is None:
            return
        info = parse_feedback(rx, addr) if rx else None
        name = f"{st['name']}(0x{addr:02X})"

        if info is None:
            st["fails"] += 1
            if st["fails"] >= self.no_resp_limit and not st["down"]:
                st["down"] = True
                log.error(f"[FY] {name} 모터 컨트롤러 응답 없음 (연속 {st['fails']}회) "
                          f"-> 모터 전원 / RS485 배선 / 주소 확인")
            return

        if st["down"]:
            log.warning(f"[FY] {name} 응답 복구")
        st["down"] = False
        st["fails"] = 0
        st["last"] = info

        if not info["ok"]:
            if st["bad_status"] != info["status"]:
                log.error(f"[FY] {name} 컨트롤러 오류 상태 0x{info['status']:02X} "
                          f"(전류 {info['current_a']:.1f}A, 전압 {info['voltage_v']:.1f}V, 온도 {info['temp_c']}C)")
            st["bad_status"] = info["status"]
        else:
            if st["bad_status"] is not None:
                log.warning(f"[FY] {name} 오류 상태 해제")
            st["bad_status"] = None

    def feedback_health(self):
        """(정상 여부, 설명). snow_robot_log 헬스 체크용."""
        bad = []
        for addr, st in self.fb.items():
            if st["down"]:
                bad.append(f"{st['name']} 응답 없음")
            elif st["bad_status"] is not None:
                bad.append(f"{st['name']} 오류 0x{st['bad_status']:02X}")
            elif st["last"] and st["last"]["temp_c"] >= 85:
                bad.append(f"{st['name']} 온도 {st['last']['temp_c']}C")
        return (not bad), ", ".join(bad)

    def battery_snapshot(self, max_age: float = 3.0) -> Optional[dict]:
        """두 모터 컨트롤러의 최근 응답을 합쳐 전원 상태를 돌려준다. 유효한 응답이 없으면 None.

        voltage: 응답이 있는 컨트롤러들의 평균 전압(V)
        current: 전류 합계(A)
        temperature: 컨트롤러 온도 중 최댓값(C)
        """
        now = time.time()
        infos = [
            st["last"] for st in self.fb.values()
            if st["last"] is not None and not st["down"] and now - st["last"]["time"] <= max_age
        ]
        if not infos:
            return None
        return {
            "voltage": round(sum(i["voltage_v"] for i in infos) / len(infos), 2),
            "current": round(sum(i["current_a"] for i in infos), 2),
            "temperature": max(i["temp_c"] for i in infos),
        }

    def feedback_summary(self) -> str:
        parts = []
        for addr, st in self.fb.items():
            l = st["last"]
            if st["down"] or l is None:
                parts.append(f"{st['name'][0]}:NO_RESP")
            else:
                parts.append(f"{st['name'][0]}:{l['voltage_v']:.1f}V/{l['current_a']:.1f}A/{l['temp_c']}C")
        return ",".join(parts)

    def drive_pair(self, left_dir: int, left_pwm: int, right_dir: int, right_pwm: int):
        self.send_wheel(self.left_addr, left_dir, left_pwm)
        time.sleep(PER_ADDR_GAP)
        self.send_wheel(self.right_addr, right_dir, right_pwm)
        time.sleep(PER_ADDR_GAP)
        self.last_pair = (left_dir, left_pwm, right_dir, right_pwm)

    def stop_once(self):
        self.send_wheel(self.left_addr, DIR_FORWARD, 0)
        time.sleep(PER_ADDR_GAP)
        self.send_wheel(self.right_addr, DIR_FORWARD, 0)
        time.sleep(PER_ADDR_GAP)
        self.last_pair = (DIR_FORWARD, 0, DIR_FORWARD, 0)

    def stop_all(self):
        for _ in range(4):
            self.stop_once()
            time.sleep(0.035)

    def close(self):
        try:
            self.stop_all()
        finally:
            self.ser.close()
            print("[FYDrive] closed")


# ============================================================
# Motion + relay controller
# ============================================================

class FYMotionController:
    """key set을 유지하고 FY-C100 C1 명령과 relay 명령을 처리한다."""

    def __init__(self, config: FYConfig):
        if float(config.input_timeout) < 0.45:
            print(
                f"[FYMotionController] input_timeout {config.input_timeout} is too short; "
                "using 0.70s"
            )
            config.input_timeout = 0.70

        env_relay_port = os.getenv("RELAY_PORT")
        env_relay_baud = os.getenv("RELAY_BAUD")
        env_relay_disable = os.getenv("RELAY_DISABLE", "0")

        if env_relay_port:
            config.relay_port = env_relay_port

        if env_relay_baud:
            try:
                config.relay_baud = int(env_relay_baud)
            except ValueError:
                print(f"[FYMotionController] invalid RELAY_BAUD={env_relay_baud}, using {config.relay_baud}")

        if env_relay_disable in ("1", "true", "TRUE", "yes", "YES"):
            config.relay_enabled = False

        self.config = config

        self.drive: Optional[FYDrive] = None
        self.relay: Optional[RelayUSB] = None

        self.hardware_error: Optional[Exception] = None
        self.relay_error: Optional[Exception] = None

        self._keys: Set[str] = set()
        self._last_input_time = 0.0

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._prev_label: Optional[str] = None
        self._prev_pair: Optional[Tuple[int, int, int, int]] = None
        self._prev_relay_requested: Optional[bool] = None
        self._relay_latched = False

        # 속도를 바꿔도 곡선 주행(완만한 쪽 바퀴)의 비율이 유지되도록 처음 비율을 기억한다.
        self._slow_ratio = float(config.slow_pwm) / float(config.pwm) if config.pwm else 0.0
        config.pwm = clamp(config.pwm, config.pwm_min, config.pwm_max)
        config.slow_pwm = clamp(round(config.pwm * self._slow_ratio), 0, 1000)

    def start(self):
        if self._running:
            return

        if self.drive is None:
            try:
                self.drive = FYDrive(self.config)
                self.hardware_error = None
            except Exception as e:
                # 열기에 실패해도 TX 스레드는 시작한다. 스레드가 주기적으로 다시 연다.
                self.hardware_error = e
                log.error(f"[FYMotionController] 모터 시리얼 열기 실패: {e} -> 백그라운드에서 재시도")

        self._open_relay_if_enabled()

        self._running = True
        if self.drive is not None:
            try:
                self.drive.stop_once()
            except Exception as e:
                log.error(f"[FYMotionController] 초기 정지 명령 실패: {e}")
        self._thread = threading.Thread(target=self._tx_loop, name="FYTxLoop", daemon=True)
        self._thread.start()
        print("[FYMotionController] TX loop started")

    # ---- 모터 시리얼 재연결 (TX 스레드에서만 호출) ----
    REOPEN_DELAYS = (1.0, 2.0, 5.0)  # 재시도 간격(초). 마지막 값을 계속 사용

    def _drop_drive(self, reason: str):
        """죽은 시리얼을 정리하고 재연결 대기 상태로 만든다. (정지 명령은 못 보내므로 보내지 않는다)"""
        drive, self.drive = self.drive, None
        log.error(f"[FY] 모터 시리얼 연결 끊김으로 판단 ({reason}) -> 재연결 시도")
        self._reopen_idx = 0
        self._next_reopen = time.time() + self.REOPEN_DELAYS[0]
        if drive is not None:
            try:
                drive.ser.close()
            except Exception:
                pass

    def _try_reopen_drive(self):
        """재연결 시간이 되면 FYDrive를 다시 만든다. 성공하면 True."""
        now = time.time()
        if now < getattr(self, "_next_reopen", 0.0):
            return False
        try:
            drive = FYDrive(self.config)
        except Exception as e:
            self.hardware_error = e
            idx = min(getattr(self, "_reopen_idx", 0) + 1, len(self.REOPEN_DELAYS) - 1)
            self._reopen_idx = idx
            self._next_reopen = now + self.REOPEN_DELAYS[idx]
            log_every("fy_reopen_fail", 10.0, logging.WARNING,
                      f"[FY] 모터 시리얼 재연결 실패: {e} (다시 시도 중)")
            return False
        try:
            drive.stop_once()
        except Exception as e:
            log.error(f"[FY] 재연결 직후 정지 명령 실패: {e}")
            try:
                drive.ser.close()
            except Exception:
                pass
            self._next_reopen = now + self.REOPEN_DELAYS[-1]
            return False
        self.hardware_error = None
        self._prev_label = None
        self._prev_pair = None
        self.drive = drive
        log.warning("[FY] 모터 시리얼 재연결 성공")
        return True

    def _open_relay_if_enabled(self):
        if not self.config.relay_enabled:
            print("[FYMotionController] relay disabled")
            return

        if self.relay is not None:
            return

        relay_port = self.config.relay_port
        if not relay_port:
            relay_port = auto_find_relay_port(
                motor_port=self.config.port,
                exclude_ports=self.config.relay_exclude_ports,
            )

        if not relay_port:
            print("[FYMotionController] relay port not found. Relay control disabled.")
            return

        if os.path.realpath(relay_port) in {os.path.realpath(p) for p in (self.config.relay_exclude_ports or ())}:
            # RELAY_PORT를 명시적으로 잘못 지정한 경우까지 방어한다.
            print(
                f"[FYMotionController] relay_port={relay_port} is an excluded "
                "(e.g. gimbal) port - refusing to open it as relay. "
                "Relay control disabled."
            )
            return

        try:
            self.relay = RelayUSB(
                port=relay_port,
                baud=int(self.config.relay_baud),
                verbose=bool(self.config.relay_verbose),
            )
            # 시작 시 OFF
            self.relay.off()
        except Exception as e:
            self.relay_error = e
            self.relay = None
            print(f"[FYMotionController] relay open failed: {e}. Relay control disabled.")

    def stop(self):
        self._running = False

        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

        if self.drive:
            try:
                self.drive.close()
            except Exception as e:
                log.error(f"[FY] 모터 종료 중 오류(정지 명령이 안 나갔을 수 있음): {e}")
            self.drive = None

        if self.relay:
            # 종료 시 강제 OFF를 원하면 아래 주석 해제
            # self.relay.off()
            try:
                self.relay.close()
            except Exception as e:
                log.error(f"[FY] 릴레이 종료 중 오류: {e}")
            self.relay = None

        print("[FYMotionController] stopped")

    @staticmethod
    def _normalize_key_token(token) -> Optional[str]:
        text = str(token).strip().lower()

        # 릴레이 토글 요청은 set_keys()로 처리하지 않는다.
        # Space 토글은 SnowRobotSubController.toggle_relay()에서만 상태를 바꾼다.
        if text in RELAY_KEY_ALIASES:
            return None

        if text in MOTOR_KEYS:
            return text

        return None

    def set_keys(self, keys):
        clean: Set[str] = set()

        for key in keys:
            normalized = self._normalize_key_token(key)
            if normalized in MOTOR_KEYS:
                clean.add(normalized)

        with self._lock:
            self._keys = clean
            self._last_input_time = time.time()

    def clear_keys(self):
        with self._lock:
            self._keys = set()
            self._last_input_time = time.time()

    def _snapshot_keys(self) -> Set[str]:
        with self._lock:
            if time.time() - self._last_input_time > float(self.config.input_timeout):
                return set()
            return set(self._keys)

    def set_relay_latched(self, on: bool):
        """릴레이 래치 상태를 모터 키와 분리해서 저장한다."""
        with self._lock:
            self._relay_latched = bool(on)

    def toggle_relay_latched(self) -> bool:
        """릴레이 래치 상태를 뒤집고 새 상태를 반환한다."""
        with self._lock:
            self._relay_latched = not self._relay_latched
            return self._relay_latched

    def get_relay_latched(self) -> bool:
        with self._lock:
            return bool(self._relay_latched)

    def adjust_pwm(self, direction: int) -> int:
        """주행 pwm을 pwm_step만큼 올리거나(+1) 내린다(-1). 새 pwm을 반환한다.

        _decide()가 매 tick config.pwm/slow_pwm을 읽으므로 다음 명령부터 바로 반영된다.
        slow_pwm은 처음 설정한 비율을 유지하도록 같이 갱신한다.
        """
        with self._lock:
            step = int(self.config.pwm_step)
            new_pwm = int(self.config.pwm) + (step if direction >= 0 else -step)
            new_pwm = clamp(new_pwm, self.config.pwm_min, self.config.pwm_max)
            self.config.pwm = new_pwm
            self.config.slow_pwm = clamp(round(new_pwm * self._slow_ratio), 0, 1000)
            return new_pwm

    def _split_keys(self, keys: Set[str]):
        motor_keys = {k for k in keys if k in MOTOR_KEYS}
        relay_requested = self.get_relay_latched()
        return motor_keys, relay_requested

    def _apply_relay(self, relay_requested: bool):
        if relay_requested != self._prev_relay_requested:
            print(f"[FY] RELAY {'ON' if relay_requested else 'OFF'} | latched={relay_requested}")
            self._prev_relay_requested = relay_requested

        if self.relay is not None:
            try:
                self.relay.set_state(relay_requested)
            except Exception as e:
                self.relay_error = e
                print(f"[FYMotionController] relay command failed: {e}")

    def _decide(self, keys: Set[str]):
        w = "w" in keys
        s = "s" in keys
        a = "a" in keys
        d = "d" in keys

        full = int(self.config.pwm)
        slow = int(self.config.slow_pwm)

        if w and s:
            return "STOP", DIR_FORWARD, 0, DIR_FORWARD, 0

        if a and d:
            if w:
                return "FORWARD", DIR_FORWARD, full, DIR_FORWARD, full
            if s:
                return "BACKWARD", DIR_BACKWARD, full, DIR_BACKWARD, full
            return "STOP", DIR_FORWARD, 0, DIR_FORWARD, 0

        if w:
            if a:
                return "FORWARD_LEFT_CURVE", DIR_FORWARD, slow, DIR_FORWARD, full
            if d:
                return "FORWARD_RIGHT_CURVE", DIR_FORWARD, full, DIR_FORWARD, slow
            return "FORWARD", DIR_FORWARD, full, DIR_FORWARD, full

        if s:
            if a:
                return "BACKWARD_LEFT_CURVE", DIR_BACKWARD, slow, DIR_BACKWARD, full
            if d:
                return "BACKWARD_RIGHT_CURVE", DIR_BACKWARD, full, DIR_BACKWARD, slow
            return "BACKWARD", DIR_BACKWARD, full, DIR_BACKWARD, full

        if a:
            return "PIVOT_LEFT", DIR_BACKWARD, full, DIR_FORWARD, full

        if d:
            return "PIVOT_RIGHT", DIR_FORWARD, full, DIR_BACKWARD, full

        return "STOP", DIR_FORWARD, 0, DIR_FORWARD, 0

    def _tx_loop(self):
        err_count = 0
        while self._running:
            try:
                if self.drive is None:
                    if self._try_reopen_drive():
                        err_count = 0
                    else:
                        time.sleep(0.2)
                    continue

                all_keys = self._snapshot_keys()
                motor_keys, relay_requested = self._split_keys(all_keys)

                self._apply_relay(relay_requested)

                label, ld, lp, rd, rp = self._decide(motor_keys)
                pair = (ld, lp, rd, rp)

                if label != self._prev_label or pair != self._prev_pair:
                    keys_text = "".join(sorted(motor_keys)) or "-"
                    all_text = ",".join(sorted(all_keys)) or "-"
                    print(
                        f"[FY] {label} | motor_keys={keys_text} all_keys={all_text} | "
                        f"L(dir={ld}, pwm={lp}) R(dir={rd}, pwm={rp})"
                    )
                    self._prev_label = label
                    self._prev_pair = pair

                if label == "STOP":
                    self.drive.stop_once()
                else:
                    self.drive.drive_pair(ld, lp, rd, rp)

                time.sleep(SEND_INTERVAL)
                if err_count:
                    log.warning(f"[FY] TX loop 복구 (연속 오류 {err_count}회 후)")
                    err_count = 0
            except Exception as e:
                err_count += 1
                log_every("fy_tx_err", 5.0, logging.ERROR,
                          f"[FY] TX loop 예외 #{err_count} (모터 시리얼/USB/전원 확인)", exc_info=True)
                # 시리얼 오류가 3회 연속이면 포트가 죽은 것으로 보고 다시 연다.
                if err_count >= 3 and self.drive is not None:
                    self._drop_drive(f"{type(e).__name__}: {e}")
                    err_count = 0
                time.sleep(0.2)
