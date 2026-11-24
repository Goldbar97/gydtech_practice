# -*- coding: utf-8 -*-
"""GERRI robot-side sub-controller for remove_snow.

이 버전은 w/a/s/d 모터 키와 space 릴레이 키를 함께 처리한다.
중요:
- "space" 문자열을 s/a로 오인하지 않는다.
- W/A/S/D를 제외한 모든 RubberNeck 버튼은 relay key로 snow_robot_controller.py까지 전달한다.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from typing import Any, Dict, Iterable, Set

from pydantic import BaseModel, Field

from snow_robot_controller import FYConfig, FYMotionController
from snow_robot_log import log, log_every
from pan_tilt_control import PanTiltController


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


class SimpleStatus(BaseModel):
    """최신 StatusManager가 요구하는 pydantic status 모델."""

    pose: Dict[str, Any] = Field(default_factory=lambda: {
        "position": {"x": None, "y": None, "z": None},
        "orientation": {"yaw": None, "pitch": None, "roll": None},
        "2d": {"x": 0, "y": 0, "th": 0},
    })
    velocity: Dict[str, Any] = Field(default_factory=lambda: {
        "linear": {"x": None, "y": None, "z": None},
        "angular": {"x": None, "y": None, "z": None},
        "2d": {"vx": 0, "vy": 0, "vth": 0},
    })
    battery_state: Dict[str, Any] = Field(default_factory=lambda: {
        "voltage": None,
        "current": None,
        "capacity": None,
        "percentage": None,
        "temperature": None,
        "now_charging": None,
    })
    joint_state: Dict[str, Any] = Field(default_factory=lambda: {
        "name": [],
        "position": [],
        "velocity": [],
        "effort": [],
    })
    path_plan: Dict[str, Any] = Field(default_factory=lambda: {
        "global": None,
        "local": None,
    })
    sensor: Dict[str, Any] = Field(default_factory=lambda: {
        "ultrasonic": None,
        "laser": None,
    })
    map: Dict[str, Any] = Field(default_factory=lambda: {
        "name": None,
        "origin": None,
        "size": None,
        "resolution": None,
    })

    def model_dump(self, *args, **kwargs):
        try:
            return super().model_dump(*args, **kwargs)
        except AttributeError:
            return self.dict(*args, **kwargs)


class SnowRobotSubController:
    GIMBAL_REOPEN_DELAYS = (1.0, 2.0, 5.0)  # 짐벌 재연결 간격(초). 마지막 값을 계속 사용
    GIMBAL_DIR_TIMEOUT = 1.0                # 방향 입력 갱신이 이 시간 넘게 없으면 정지

    def __init__(
        self,
        robot_info: dict | None = None,
        fy_config: dict | None = None,
        gimbal_config: dict | None = None,
    ):
        robot_info = robot_info or {}
        fy_config = dict(fy_config or {})
        gimbal_config = gimbal_config or {}

        self.robot_id = robot_info.get("id", "snow_robot")
        self.robot_model = robot_info.get("model", "snow")
        self.robot_category = robot_info.get("category", "mobile")
        self.base_controller = None
        self.status = SimpleStatus()
        self._status_running = False
        self._status_thread = None

        # ---- 팬틸트 짐벌 포트를 먼저 파악해서, 릴레이 자동탐지가 이 포트를
        #      릴레이로 오인해서 가로채지 않도록 제외 목록에 넣는다. ----
        self._gimbal_enabled = gimbal_config.get("enabled", True)
        _gimbal_port = gimbal_config.get("port", "/dev/ttyUSB0")
        if self._gimbal_enabled and _gimbal_port:
            existing_exclude = tuple(fy_config.get("relay_exclude_ports", ()))
            if _gimbal_port not in existing_exclude:
                fy_config["relay_exclude_ports"] = existing_exclude + (_gimbal_port,)

        self.motion = FYMotionController(FYConfig(**fy_config))
        self._relay_latched = False

        # ---- 팬틸트 카메라 짐벌 (모터와 완전히 별개의 시리얼 포트) ----
        self._gimbal_deg_per_sec = float(gimbal_config.get("deg_per_sec", 60.0))
        self._gimbal_speed_step = float(gimbal_config.get("speed_step", 15.0))
        self._gimbal_speed_min = float(gimbal_config.get("speed_min", 15.0))
        self._gimbal_speed_max = float(gimbal_config.get("speed_max", 180.0))
        self._gimbal_x_min = float(gimbal_config.get("x_min", -180))
        self._gimbal_x_max = float(gimbal_config.get("x_max", 180))
        self._gimbal_y_min = float(gimbal_config.get("y_min", -30))
        self._gimbal_y_max = float(gimbal_config.get("y_max", 90))

        self.gimbal = None
        self._gimbal_port = gimbal_config.get("port", "/dev/ttyUSB0")
        self._gimbal_baud = int(gimbal_config.get("baud", 115200))
        # 재연결 상태 (_gimbal_loop 스레드에서만 사용)
        self._gimbal_fail = 0            # move_to 연속 실패 횟수
        self._gimbal_next_reopen = 0.0   # 이 시각 이후에 다시 열기 시도
        self._gimbal_reopen_idx = 0
        self._gimbal_last_check = 0.0    # 포트 존재 확인 시각
        self._gimbal_last_dir_time = time.time()  # 마지막으로 방향 입력이 갱신된 시각
        if self._gimbal_enabled:
            print(f"[SnowRobotSubController] gimbal opening port={self._gimbal_port} baud={self._gimbal_baud}")
            try:
                self.gimbal = PanTiltController(port=self._gimbal_port, baudrate=self._gimbal_baud)
                print(f"[SnowRobotSubController] gimbal opened OK: {self.gimbal}")
            except Exception as e:
                log.error(f"[SnowRobotSubController] gimbal serial open failed: {e} -> 백그라운드에서 재시도")
                self.gimbal = None
                self._gimbal_next_reopen = time.time() + self.GIMBAL_REOPEN_DELAYS[0]
        else:
            print("[SnowRobotSubController] gimbal disabled by config (GIMBAL_ENABLED != 1)")

        self._gimbal_x = 0.0
        self._gimbal_y = 0.0
        self._gimbal_dir = (0, 0)  # (x_dir, y_dir), each -1/0/1
        self._gimbal_dir_lock = threading.Lock()
        self._gimbal_running = False
        self._gimbal_thread = None

    def connect(self):
        print("[SnowRobotSubController] connect")
        self.motion.start()
        self._status_running = True

        if self._status_thread is None or not self._status_thread.is_alive():
            self._status_thread = threading.Thread(target=self._status_loop, daemon=True)
            self._status_thread.start()

        if self.gimbal is not None:
            print("[SnowRobotSubController] gimbal startup sweep test")
            try:
                self.gimbal.move_to(30, 60, speed=200, acc=30)
                time.sleep(1.0)
                self.gimbal.move_to(-30, -30, speed=200, acc=30)
                time.sleep(1.0)
                self.gimbal.move_to(0, 0, speed=200, acc=30)
                print("[SnowRobotSubController] gimbal startup sweep done")
            except Exception as e:
                print(f"[SnowRobotSubController] gimbal startup sweep error: {e}")

        if self._gimbal_enabled:
            # 시작 때 포트가 없어도 스레드는 시작한다. 스레드가 주기적으로 다시 연다.
            self._gimbal_running = True
            self._gimbal_thread = threading.Thread(target=self._gimbal_loop, name="GimbalLoop", daemon=True)
            self._gimbal_thread.start()
            print("[SnowRobotSubController] gimbal thread started")

    def disconnect(self):
        print("[SnowRobotSubController] disconnect")
        self._status_running = False
        self.motion.stop()

        self._gimbal_running = False
        g = self.gimbal
        if g is not None:
            try:
                g.close()
            except Exception as e:
                print(f"[SnowRobotSubController] gimbal close error: {e}")

    # ---- 팬틸트 조작 API : RubberNeckJoyStickHandler가 매 poll마다 호출 ----
    def gimbal_set_direction(self, x_dir: int, y_dir: int):
        """x_dir/y_dir: -1, 0, 1. 주행(w/a/s/d)과는 완전히 분리된 채널."""
        self._gimbal_last_dir_time = time.time()  # 입력이 살아 있다는 표시 (정지 보장용)
        with self._gimbal_dir_lock:
            if self._gimbal_dir != (x_dir, y_dir):
                print(f"[SnowRobotSubController] gimbal_dir {self._gimbal_dir} -> {(x_dir, y_dir)} (gimbal={'OK' if self.gimbal else 'None'})")
                self._gimbal_dir = (x_dir, y_dir)

    def gimbal_center(self):
        with self._gimbal_dir_lock:
            self._gimbal_dir = (0, 0)
        self._gimbal_x, self._gimbal_y = 0.0, 0.0
        g = self.gimbal
        if g is not None:
            try:
                g.move_to(0.0, 0.0, speed=200, acc=30)
            except Exception as e:
                log_every("gimbal_center_err", 5.0, logging.ERROR, f"[SnowRobotSubController] gimbal center error: {e}")

    def drive_adjust_speed(self, direction: int):
        """로봇 주행 속도(pwm)를 조절한다. direction: +1 가속, -1 감속."""
        pwm = self.motion.adjust_pwm(direction)
        cfg = self.motion.config
        print(f"[SnowRobotSubController] drive speed -> pwm={pwm} slow_pwm={cfg.slow_pwm} "
              f"(range {cfg.pwm_min}~{cfg.pwm_max}, step {cfg.pwm_step})")
        return pwm

    def gimbal_adjust_speed(self, direction: int):
        """팬틸트 회전속도(deg/sec)를 조절한다.

        direction: +1 이면 speed_step만큼 증가, -1 이면 감소.
        _gimbal_deg_per_sec는 _gimbal_loop()에서 매 tick 읽어서 각도
        적분에 바로 사용하므로, 다음 이동부터 즉시 반영된다.
        """
        with self._gimbal_dir_lock:
            step = self._gimbal_speed_step if direction >= 0 else -self._gimbal_speed_step
            new_speed = self._gimbal_deg_per_sec + step
            new_speed = max(self._gimbal_speed_min, min(self._gimbal_speed_max, new_speed))
            self._gimbal_deg_per_sec = new_speed
        print(f"[SnowRobotSubController] gimbal speed -> {new_speed:.1f} deg/s "
              f"(range {self._gimbal_speed_min:.0f}~{self._gimbal_speed_max:.0f})")

    # ---- 짐벌 시리얼 재연결 (_gimbal_loop 스레드에서만 호출) ----
    def _gimbal_drop(self, reason: str):
        """죽은 짐벌 시리얼을 버리고 재연결 대기 상태로 만든다."""
        g, self.gimbal = self.gimbal, None
        log.error(f"[SnowRobotSubController] 짐벌 연결 끊김으로 판단 ({reason}) -> 재연결 시도")
        self._gimbal_fail = 0
        self._gimbal_reopen_idx = 0
        self._gimbal_next_reopen = time.time() + self.GIMBAL_REOPEN_DELAYS[0]
        if g is not None:
            try:
                g.close()
            except Exception:
                pass

    def _gimbal_try_reopen(self) -> bool:
        """재연결 시간이 되면 다시 연다. 성공하면 위치를 중앙으로 맞추고 True."""
        now = time.time()
        if now < self._gimbal_next_reopen:
            return False
        if not os.path.exists(self._gimbal_port):
            self._gimbal_next_reopen = now + self.GIMBAL_REOPEN_DELAYS[min(self._gimbal_reopen_idx, 2)]
            log_every("gimbal_wait_port", 10.0, logging.WARNING,
                      f"[SnowRobotSubController] 짐벌 포트 없음, 대기 중: {self._gimbal_port}")
            return False
        try:
            # 생성자가 ESP32 부팅 대기로 약 2초 걸린다 (이 스레드 안에서만 블로킹)
            g = PanTiltController(port=self._gimbal_port, baudrate=self._gimbal_baud)
        except Exception as e:
            self._gimbal_reopen_idx = min(self._gimbal_reopen_idx + 1, len(self.GIMBAL_REOPEN_DELAYS) - 1)
            self._gimbal_next_reopen = now + self.GIMBAL_REOPEN_DELAYS[self._gimbal_reopen_idx]
            log_every("gimbal_reopen_fail", 10.0, logging.WARNING,
                      f"[SnowRobotSubController] 짐벌 재연결 실패: {e} (다시 시도 중)")
            return False

        # 재부팅됐을 수 있어 실제 위치를 모른다. 추정값을 0으로 맞추고 중앙으로 보낸다.
        with self._gimbal_dir_lock:
            self._gimbal_dir = (0, 0)
        self._gimbal_x, self._gimbal_y = 0.0, 0.0
        try:
            g.move_to(0.0, 0.0, speed=200, acc=30)
        except Exception as e:
            log.error(f"[SnowRobotSubController] 재연결 직후 중앙 이동 실패: {e}")
            try:
                g.close()
            except Exception:
                pass
            self._gimbal_next_reopen = time.time() + self.GIMBAL_REOPEN_DELAYS[-1]
            return False
        self.gimbal = g
        self._gimbal_fail = 0
        log.warning("[SnowRobotSubController] 짐벌 재연결 성공 (중앙으로 복귀)")
        return True

    def _gimbal_loop(self):
        """방향키가 눌려있는 동안 각도를 시간 기반으로 적분해서 부드럽게 이동.
        RubberNeck 콜백 주기와 무관하게 독립적으로 동작하도록 별도 스레드로 분리.

        추가 기능
        - 방향 입력 갱신이 GIMBAL_DIR_TIMEOUT(1초) 넘게 없으면 정지 (조이스틱 스레드가 죽은 경우 보호)
        - 시리얼 오류 3회 연속, 또는 포트가 사라지면 연결을 버리고 자동 재연결
        - 에러 로그는 5초에 한 번만 기록
        """
        last = time.time()
        while self._gimbal_running:
            now = time.time()
            dt = now - last
            last = now

            # 1) 연결 상태 확인 / 재연결
            g = self.gimbal
            if g is None:
                if self._gimbal_try_reopen():
                    last = time.time()  # 재연결 대기 시간이 각도 적분(dt)에 들어가지 않게
                else:
                    time.sleep(0.2)
                continue
            if now - self._gimbal_last_check >= 1.0:
                self._gimbal_last_check = now
                if not os.path.exists(self._gimbal_port):
                    self._gimbal_drop("포트가 사라짐(USB 분리)")
                    continue

            # 2) 방향 입력
            with self._gimbal_dir_lock:
                x_dir, y_dir = self._gimbal_dir
                if (x_dir or y_dir) and now - self._gimbal_last_dir_time > self.GIMBAL_DIR_TIMEOUT:
                    self._gimbal_dir = (0, 0)
                    x_dir, y_dir = 0, 0
                    stale = True
                else:
                    stale = False
            if stale:
                log_every("gimbal_dir_stale", 5.0, logging.WARNING,
                          "[SnowRobotSubController] 방향 입력 갱신이 1초 넘게 없어 짐벌 정지")

            moved = False
            if x_dir:
                self._gimbal_x = max(
                    self._gimbal_x_min,
                    min(self._gimbal_x_max, self._gimbal_x + x_dir * self._gimbal_deg_per_sec * dt),
                )
                moved = True
            if y_dir:
                self._gimbal_y = max(
                    self._gimbal_y_min,
                    min(self._gimbal_y_max, self._gimbal_y + y_dir * self._gimbal_deg_per_sec * dt),
                )
                moved = True

            if moved:
                try:
                    g.move_to(self._gimbal_x, self._gimbal_y, speed=200, acc=30)
                    self._gimbal_fail = 0
                except Exception as e:
                    self._gimbal_fail += 1
                    log_every("gimbal_move_err", 5.0, logging.ERROR,
                              f"[SnowRobotSubController] gimbal move error #{self._gimbal_fail}: {e}")
                    if self._gimbal_fail >= 3:
                        self._gimbal_drop(f"{type(e).__name__}: {e}")
                        continue

            time.sleep(0.03)

    def _status_loop(self):
        """RubberNeck로 보내는 status(StatusManager가 1초마다 vars(self.status)를 읽음)를 갱신한다.

        battery_state: 모터 컨트롤러 응답의 전압(V) / 전류 합계(A) / 컨트롤러 온도(C).
        응답이 없으면(전원 차단, 통신 끊김) 값은 None으로 돌아간다.
        percentage, capacity는 배터리 종류를 몰라 채우지 않는다.
        """
        while self._status_running:
            try:
                drive = self.motion.drive  # 재연결 중 None이 될 수 있어 한 번만 읽는다
                snap = drive.battery_snapshot() if drive is not None else None
                self.status.battery_state = {
                    "voltage": snap["voltage"] if snap else None,
                    "current": snap["current"] if snap else None,
                    "capacity": None,
                    "percentage": None,
                    "temperature": snap["temperature"] if snap else None,
                    "now_charging": None,
                }
            except Exception:
                log_every("status_loop_err", 30.0, logging.ERROR,
                          "[SnowRobotSubController] status 갱신 오류", exc_info=True)
            time.sleep(0.5)

    @staticmethod
    def _to_float(value: Any, default: float = 0.0) -> float:
        try:
            if value is None:
                return default
            return float(value)
        except Exception:
            return default

    @staticmethod
    def _keys_from_velocity(vx: Any = None, vy: Any = None, vth: Any = None) -> Set[str]:
        keys: Set[str] = set()
        vx_f = SnowRobotSubController._to_float(vx)
        vy_f = SnowRobotSubController._to_float(vy)
        vth_f = SnowRobotSubController._to_float(vth)

        if vx_f > 0.01:
            keys.add("w")
        elif vx_f < -0.01:
            keys.add("s")

        if vy_f > 0.01:
            keys.add("a")
        elif vy_f < -0.01:
            keys.add("d")

        if vth_f > 0.01:
            keys.add("a")
        elif vth_f < -0.01:
            keys.add("d")

        return keys

    @staticmethod
    def _normalize_single_key(value: Any) -> Set[str]:
        text = str(value).strip().lower()

        if text in RELAY_KEY_ALIASES:
            return {"relay"}

        if text in MOTOR_KEYS:
            return {text}

        return set()

    @staticmethod
    def _normalize_keys(value: Any) -> Set[str]:
        """여러 형태의 입력을 {'w','a','s','d','space'} set으로 변환한다."""

        if value is None:
            return set()

        if isinstance(value, dict):
            # 명시적 key 목록 우선 처리
            for key_name in (
                "keys",
                "key_control",
                "key",
                "pressed",
                "pressed_keys",
                "active_keys",
            ):
                if key_name in value:
                    return SnowRobotSubController._normalize_keys(value.get(key_name))

            keys: Set[str] = set()

            # 별도 space/relay toggle 필드 대응
            for relay_field in ("space", "spacebar", "key_space", "keyboard_space", "relay_toggle", "relay_toggle_pressed"):
                if bool(value.get(relay_field)):
                    keys.add("relay_toggle_pressed")

            # RubberNeck /joy dict: {'axes': [...], 'buttons': [...]}
            axes = value.get("axes")
            if isinstance(axes, (list, tuple)):
                try:
                    ax0 = float(axes[0]) if len(axes) > 0 else 0.0
                    ax1 = float(axes[1]) if len(axes) > 1 else 0.0
                    ax2 = float(axes[2]) if len(axes) > 2 else 0.0

                    # 기존 로그 기준: axes[1] < 0 = 전진
                    if ax1 < -0.2:
                        keys.add("w")
                    elif ax1 > 0.2:
                        keys.add("s")

                    # 회전은 axes[2] 우선, 없으면 axes[0]
                    turn = ax2 if abs(ax2) > abs(ax0) else ax0
                    if turn < -0.2:
                        keys.add("a")
                    elif turn > 0.2:
                        keys.add("d")
                except Exception:
                    pass

            buttons = value.get("buttons")
            if isinstance(buttons, (list, tuple)):
                # raw /joy 파서는 이동 버튼만 처리한다.
                # Space 토글은 RubberNeckJoyStickHandler의 rising edge에서 직접 처리한다.
                movement_button_map = {
                    12: "w",
                    13: "s",
                    14: "a",
                    15: "d",
                }

                for idx, key in movement_button_map.items():
                    try:
                        if idx < len(buttons) and int(buttons[idx]) == 1:
                            keys.add(key)
                    except Exception:
                        pass

            keys |= SnowRobotSubController._keys_from_velocity(
                vx=value.get("vx", value.get("linear_x", value.get("linear", None))),
                vy=value.get("vy", value.get("linear_y", None)),
                vth=value.get("vth", value.get("wz", value.get("angular", value.get("angular_z", None)))),
            )

            for k in MOTOR_KEYS:
                if bool(value.get(k)) or bool(value.get(k.upper())):
                    keys.add(k)

            return keys

        if isinstance(value, str):
            text = value.strip().lower()

            # 매우 중요:
            # "space"를 글자 단위로 풀면 s/a가 되어 모터 명령으로 오인된다.
            if text in RELAY_KEY_ALIASES:
                return {"relay_toggle_pressed"}

            if text in ("", "stop", "x", "none", "null"):
                return set()

            normalized = (
                text.replace("key_", "key_")
                .replace("+", " ")
                .replace(",", " ")
                .replace("/", " ")
            )
            parts = normalized.split()

            if len(parts) > 1:
                keys: Set[str] = set()
                for p in parts:
                    keys |= SnowRobotSubController._normalize_single_key(p)
                return keys

            # 단일 문자인 w/a/s/d만 허용.
            # "space" 같은 긴 문자열을 문자 단위로 해석하지 않는다.
            if len(text) == 1:
                return SnowRobotSubController._normalize_single_key(text)

            return set()

        if isinstance(value, Iterable):
            keys: Set[str] = set()
            for item in value:
                keys |= SnowRobotSubController._normalize_keys(item)
            return keys

        return set()

    def on_message(self, topic=None, value=None, option=None, raw=None):
        topic_text = str(topic or "").lower()
        print(f"[SnowRobotSubController] RX topic={topic_text} value={value}")

        if topic_text in ("move", "/move", "joy", "/joy", "cmd_vel", "/cmd_vel"):
            self.move(value=value, option=option)
            return

        if topic_text in ("stop", "/stop", "open", "/open"):
            self.stop(value=value, option=option)
            return

        if topic_text in ("key_mouse_control", "keyboard", "key"):
            self.move(value=value, option=option)
            return

        keys = self._normalize_keys(value)
        if "relay_toggle_pressed" in keys:
            self.toggle_relay()
            keys.discard("relay_toggle_pressed")

        if keys:
            self.motion.set_keys(keys)
            print(f"[SnowRobotSubController] inferred keys={sorted(keys)}")
            return

        print(f"[SnowRobotSubController] ignored topic={topic_text}")

    def toggle_relay(self):
        """Space rising edge에서만 호출한다.

        릴레이 상태는 모터 키와 완전히 분리된다.
        W/A/S/D 주행 중에는 이 상태를 절대 바꾸지 않는다.
        """
        self._relay_latched = self.motion.toggle_relay_latched()
        print(f"[SnowRobotSubController] SPACE toggle -> relay_latched={self._relay_latched}")
        return self._relay_latched

    def move(self, value=None, option=None, **kwargs):
        keys: Set[str] = set()

        if value is not None:
            keys |= self._normalize_keys(value)

        if kwargs:
            if "keys" in kwargs:
                keys |= self._normalize_keys(kwargs.get("keys"))

            keys |= self._keys_from_velocity(
                vx=kwargs.get("vx", kwargs.get("linear_x", kwargs.get("linear", None))),
                vy=kwargs.get("vy", kwargs.get("linear_y", None)),
                vth=kwargs.get("vth", kwargs.get("wz", kwargs.get("angular", kwargs.get("angular_z", None)))),
            )

            for k in MOTOR_KEYS:
                if bool(kwargs.get(k)) or bool(kwargs.get(k.upper())):
                    keys.add(k)

            for relay_field in ("space", "spacebar", "key_space", "keyboard_space", "relay_toggle", "relay_toggle_pressed"):
                if bool(kwargs.get(relay_field)):
                    keys.add("relay_toggle_pressed")

        if "relay_toggle_pressed" in keys:
            self.toggle_relay()
            keys.discard("relay_toggle_pressed")

        # relay 토큰은 모터 키셋으로 넘기지 않는다.
        # 릴레이 상태는 FYMotionController 내부 latch만 사용한다.
        keys.discard("relay")
        keys.discard("relay_toggle")

        if keys:
            self.motion.set_keys(keys)
        else:
            self.motion.clear_keys()
            
        #motor_keys = sorted(k for k in keys if k in MOTOR_KEYS)
        #print(
        #    f"[SnowRobotSubController] move motor_keys={motor_keys or '-'} "
        #    f"relay_latched={self._relay_latched} all_keys={sorted(keys) or '-'} kwargs={kwargs}"
        #)


    def stop(self, value=None, option=None):
        self.motion.clear_keys()
        print("[SnowRobotSubController] stop")

    def change_mode(self, value=None, option=None):
        print("[SnowRobotSubController] change_mode", value, option)

    def move_waypoint(self, value=None, option=None):
        print("[SnowRobotSubController] move_waypoint", value, option)

    def move_coord(self, value=None, option=None):
        print("[SnowRobotSubController] move_coord", value, option)

    def dock(self, value=None, option=None):
        print("[SnowRobotSubController] dock", value, option)

    def relocate(self, value=None, option=None):
        print("[SnowRobotSubController] relocate", value, option)

    def move_floor(self, value=None, option=None):
        print("[SnowRobotSubController] move_floor", value, option)

    def map_change(self, value=None, option=None):
        print("[SnowRobotSubController] map_change", value, option)

    def hello_universe(self, value=None):
        print("[SnowRobotSubController] hello_universe", value)
        if self.base_controller:
            self.base_controller.send_message({
                "topic": "hello_universe",
                "value": "hello_universe_by_remove_snow",
                "target": "all",
            })
        return "hello_universe_by_remove_snow"
