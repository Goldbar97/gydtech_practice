#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Snow robot FY-C100 RS485 motor controller.

This module contains only the hardware control logic that was verified locally:
- FY-C100 RS485 C1 frames
- left addr 0x11, right addr 0x12
- dir=0 forward, dir=1 backward
- W+A / W+D curves by slowing one wheel, not reversing it

It is designed to be called from camping_mate/GERRI sub-controller code.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional, Set, Tuple

import serial


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


def hx(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


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


def make_c1(addr: int, run: int, direction: int, pwm: int, current_a: float) -> bytes:
    pwm = max(0, min(1000, int(pwm)))
    cur = max(0, min(65535, int(round(float(current_a) * 100))))
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
    return make_c1(addr, 1, direction, pwm, current_a)


def make_stop(addr: int, current_a: float) -> bytes:
    return make_c1(addr, 0, DIR_FORWARD, 0, current_a)


@dataclass
class FYConfig:
    port: str = DEFAULT_PORT
    baud: int = DEFAULT_BAUD
    left_addr: int = LEFT_ADDR
    right_addr: int = RIGHT_ADDR
    pwm: int = DEFAULT_PWM
    slow_pwm: int = DEFAULT_SLOW_PWM
    current_a: float = DEFAULT_CURRENT_A
    input_timeout: float = 0.7
    verbose: bool = False


class FYDrive:
    def __init__(self, config: FYConfig):
        self.left_addr = int(config.left_addr)
        self.right_addr = int(config.right_addr)
        self.pwm = int(config.pwm)
        self.slow_pwm = int(config.slow_pwm)
        self.current_a = float(config.current_a)
        self.verbose = bool(config.verbose)
        self.lock = threading.Lock()
        self.last_pair: Optional[Tuple[int, int, int, int]] = None

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

    def send_wheel(self, addr: int, direction: int, pwm: int):
        if pwm <= 0:
            frame = make_stop(addr, self.current_a)
        else:
            frame = make_run(addr, direction, pwm, self.current_a)
        self.send_and_read(frame)

    def drive_pair(self, left_dir: int, left_pwm: int, right_dir: int, right_pwm: int):
        # Locally verified behavior: repeated C1 command transmission keeps motion stable.
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
        for _ in range(3):
            self.stop_once()
            time.sleep(0.04)

    def close(self):
        try:
            self.stop_all()
        finally:
            self.ser.close()
            print("[FYDrive] closed")


class FYMotionController:
    """
    Maintains latest remote key state and transmits FY-C100 motor commands.

    The network side only updates keys through set_keys(). A background TX loop
    repeats the current command and stops the robot if input messages disappear.
    """

    def __init__(self, config: FYConfig):
        self.config = config
        self.drive = FYDrive(config)
        self._keys: Set[str] = set()
        self._last_input_time = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._prev_label: Optional[str] = None

    def start(self):
        if self._running:
            return
        self._running = True
        self.drive.stop_once()
        self._thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._thread.start()
        print("[FYMotionController] TX loop started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self.drive.close()
        print("[FYMotionController] stopped")

    def set_keys(self, keys):
        clean = {str(k).lower() for k in keys if str(k).lower() in {"w", "a", "s", "d"}}
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

        # Forward curves: never reverse the inner wheel. Slow one wheel only.
        if w:
            if a:
                return "FORWARD_LEFT_CURVE", DIR_FORWARD, slow, DIR_FORWARD, full
            if d:
                return "FORWARD_RIGHT_CURVE", DIR_FORWARD, full, DIR_FORWARD, slow
            return "FORWARD", DIR_FORWARD, full, DIR_FORWARD, full

        # Backward curves: never reverse the inner wheel. Slow one wheel only.
        if s:
            if a:
                return "BACKWARD_LEFT_CURVE", DIR_BACKWARD, slow, DIR_BACKWARD, full
            if d:
                return "BACKWARD_RIGHT_CURVE", DIR_BACKWARD, full, DIR_BACKWARD, slow
            return "BACKWARD", DIR_BACKWARD, full, DIR_BACKWARD, full

        # A/D alone still means pivot turn.
        if a:
            return "PIVOT_LEFT", DIR_BACKWARD, full, DIR_FORWARD, full
        if d:
            return "PIVOT_RIGHT", DIR_FORWARD, full, DIR_BACKWARD, full

        return "STOP", DIR_FORWARD, 0, DIR_FORWARD, 0

    def _tx_loop(self):
        while self._running:
            keys = self._snapshot_keys()
            label, ld, lp, rd, rp = self._decide(keys)

            if label != self._prev_label:
                keys_text = "".join(sorted(keys)) or "-"
                print(f"[FY] {label} | keys={keys_text} | L(dir={ld}, pwm={lp}) R(dir={rd}, pwm={rp})")
                if label == "STOP":
                    self.drive.stop_once()
                self._prev_label = label

            if label != "STOP":
                self.drive.drive_pair(ld, lp, rd, rp)

            time.sleep(SEND_INTERVAL)
