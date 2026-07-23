# -*- coding: utf-8 -*-
"""GERRI robot-side sub-controller for remove_snow.

Operator 쪽을 수정하지 않고, robot 쪽에서 다양한 입력 형태를 받아 FY-C100 key set으로 변환한다.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Iterable, Set

try:
    from robot_status import RobotStatus
except Exception:
    RobotStatus = None

from snow_robot_controller import FYConfig, FYMotionController


class SnowRobotSubController:
    def __init__(self, robot_info:dict, fy_config: dict):
        self.robot_id = robot_info.get("id")
        self.robot_model = robot_info.get("model")
        self.robot_category = robot_info.get("category")
        self.base_controller = None
        self.status = None
        self.motion = FYMotionController(FYConfig(**fy_config))

    def connect(self):
        print("[SnowRobotSubController] connect")
        self.motion.start()

        if RobotStatus is not None and self.base_controller is not None:
            self.status = RobotStatus(
                robot_id=self.base_controller.robot_id,
                model=self.base_controller.robot_model,
                category=self.base_controller.robot_category,
            )
        else:
            self.status = SimpleStatus()

        threading.Thread(target=self._status_loop, daemon=True).start()

    def disconnect(self):
        print("[SnowRobotSubController] disconnect")
        self.motion.stop()

    def _status_loop(self):
        while True:
            # 최소 heartbeat. 배터리/위치/센서는 추후 실제 센서 연결 시 채우면 됨.
            time.sleep(0.5)

    # region 기존 메시지 처리 -> BaseController, RubberNeckJoyHandler에서 처리되므로 주석처리
    # @staticmethod
    # def _normalize_keys(value: Any) -> Set[str]:
    #     """여러 입력 형태를 {'w','a','s','d'} set으로 변환."""
    #     if value is None:
    #         return set()

    #     # dict 형태:
    #     # 1) {'keys': ['w','a']}
    #     # 2) {'key_control': ['W','A']}
    #     # 3) {'vx': 0.2, 'vy': 0.0, 'vth': 0.1}
    #     # 4) {'w': True, 'a': False}
    #     if isinstance(value, dict):
    #         # keys/key_control/key/pressed 는 그대로 키 목록으로 해석한다.
    #         # buttons 는 조이스틱 버튼 배열이므로 여기서 바로 return하면 안 된다.
    #         for key_name in ("keys", "key_control", "key", "pressed"):
    #             if key_name in value:
    #                 return SnowRobotSubController._normalize_keys(value.get(key_name))

    #         keys = set()

    #         # ------------------------------------------------------------
    #         # /joy 형태 대응
    #         # 실제 로그 예:
    #         #   {'axes': [0, -1, 0, 0, 0, 0], 'buttons': [...]}
    #         # 여기서는 axes[1] = -1 일 때 전진(W)으로 들어오고 있었다.
    #         # 일반 joystick 기준으로 axes[0]은 좌/우, axes[1]은 전/후로 처리한다.
    #         # ------------------------------------------------------------
    #         axes = value.get("axes")
    #         if isinstance(axes, (list, tuple)):
    #             try:
    #                 ax0 = float(axes[0]) if len(axes) > 0 else 0.0
    #                 ax1 = float(axes[1]) if len(axes) > 1 else 0.0

    #                 # 좌우: axes[0]
    #                 # 값 방향은 operator 환경에 따라 반대일 수 있음.
    #                 # 현재 기본은 -1=A, +1=D.
    #                 if ax0 < -0.2:
    #                     keys.add("a")
    #                 elif ax0 > 0.2:
    #                     keys.add("d")

    #                 # 전후: axes[1]
    #                 # 로그에서 [0, -1, ...]가 전진 입력이므로 -1=W, +1=S.
    #                 if ax1 < -0.2:
    #                     keys.add("w")
    #                 elif ax1 > 0.2:
    #                     keys.add("s")
    #             except Exception:
    #                 pass

    #         # ------------------------------------------------------------
    #         # buttons 형태 대응
    #         # 네 로그에서 buttons[4], buttons[5]가 A/D 계열 입력으로 보였기 때문에
    #         # 보조 매핑으로 추가한다. axes가 같이 들어오면 axes와 조합된다.
    #         # 필요 시 아래 JOY_BUTTON_MAP만 바꾸면 된다.
    #         # ------------------------------------------------------------
    #         buttons = value.get("buttons")
    #         if isinstance(buttons, (list, tuple)):
    #             JOY_BUTTON_MAP = {
    #                 0: "w",   # 일부 UI에서 첫 번째 버튼이 전진으로 들어오는 경우 대비
    #                 1: "s",
    #                 4: "a",
    #                 5: "d",
    #             }
    #             for idx, key in JOY_BUTTON_MAP.items():
    #                 try:
    #                     if idx < len(buttons) and int(buttons[idx]) == 1:
    #                         keys.add(key)
    #                 except Exception:
    #                     pass

    #         # 기존 mobile move 계열: vx/vy/vth 형태 대응
    #         vx = value.get("vx", value.get("linear_x", value.get("linear", None)))
    #         vy = value.get("vy", value.get("linear_y", None))
    #         vth = value.get("vth", value.get("wz", value.get("angular", value.get("angular_z", None))))

    #         try:
    #             if vx is not None:
    #                 vx = float(vx)
    #                 if vx > 0.01:
    #                     keys.add("w")
    #                 elif vx < -0.01:
    #                     keys.add("s")

    #             if vy is not None:
    #                 vy = float(vy)
    #                 if vy > 0.01:
    #                     keys.add("a")
    #                 elif vy < -0.01:
    #                     keys.add("d")

    #             if vth is not None:
    #                 vth = float(vth)
    #                 if vth > 0.01:
    #                     keys.add("a")
    #                 elif vth < -0.01:
    #                     keys.add("d")
    #         except Exception:
    #             pass

    #         # 직접 boolean map 형태
    #         for k in ("w", "a", "s", "d"):
    #             if bool(value.get(k)):
    #                 keys.add(k)
    #             if bool(value.get(k.upper())):
    #                 keys.add(k)

    #         return keys

    #     # 문자열 형태: 'w', 'W', 'wa', 'w+a', 'KEY_W'
    #     if isinstance(value, str):
    #         text = value.lower().strip()
    #         if text in ("", "stop", "x", "none", "null"):
    #             return set()

    #         normalized = text.replace("key_", "").replace("+", " ").replace(",", " ")
    #         parts = normalized.split()
    #         if len(parts) > 1:
    #             return {p for p in parts if p in {"w", "a", "s", "d"}}
    #         return {ch for ch in normalized if ch in {"w", "a", "s", "d"}}

    #     # list/tuple/set 형태: ['W','A'] 등
    #     if isinstance(value, Iterable):
    #         keys = set()
    #         for item in value:
    #             keys |= SnowRobotSubController._normalize_keys(item)
    #         return keys

    #     return set()

    # def on_message(self, topic=None, value=None, option=None, raw=None):
    #     topic_text = str(topic or "").lower()

    #     # 디버그: 서버에서 실제로 어떤 topic/value가 들어오는지 확인
    #     print(f"[SnowRobotSubController] RX topic={topic_text} value={value}")

    #     if topic_text in ("move", "/move", "joy", "/joy", "cmd_vel", "/cmd_vel"):
    #         self.move(value, option)
    #         return

    #     if topic_text in ("stop", "/stop"):
    #         self.stop(value, option)
    #         return

    #     # 혹시 operator가 key_mouse_control 자체를 보내는 경우
    #     if topic_text in ("key_mouse_control", "keyboard", "key"):
    #         self.move(value, option)
    #         return

    #     # topic이 예상과 달라도 value 안에 키/속도 정보가 있으면 move로 처리
    #     keys = self._normalize_keys(value)
    #     if keys:
    #         self.motion.set_keys(keys)
    #         print(f"[SnowRobotSubController] inferred move keys={sorted(keys)}")
    #         return

    #     print(f"[SnowRobotSubController] ignored topic={topic_text}")
    # endregion
    
    # def move(self, value=None, option=None):
    #     keys = self._normalize_keys(value)
    #     self.motion.set_keys(keys)
    #     print(f"[SnowRobotSubController] move keys={sorted(keys)}")

    def move(self, **kwargs):
        print(f"move excuted with params : {kwargs}")

    def stop(self, value=None, option=None):
        self.motion.clear_keys()
        print("[SnowRobotSubController] stop")

    # generic mobile_function 호환 메서드
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


class SimpleStatus:
    def __init__(self):
        self.pose = {"2d": {"x": 0, "y": 0, "th": 0}}
        self.velocity = {"2d": {"vx": 0, "vy": 0, "vth": 0}}
        self.battery_state = {}
        self.joint_state = {}
        self.path_plan = {}
        self.sensor = {}
        self.map = {}
