#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Robot-side entry point for remove_snow FY-C100 robot.

최신 gyd_ws 기준 수정본.
- gerri.robot.* import 사용하지 않음
- 최신 gerri.utils.common_base_controller.BaseController 기반
- StatusManager가 요구하는 pydantic status는 snow_robot_sub_controller.SimpleStatus에서 제공
- /joy, /move, /stop 같은 target 없는 메시지도 sub_controller로 직접 라우팅
"""

from __future__ import annotations

import logging
import os
import sys
import time
from typing import Set, TYPE_CHECKING

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from _and_.and_robot import AdaptiveNetworkDaemon
from gerri.utils.common_base_controller import BaseController
from gerri.interface.joy_handler.core.joystick_handler import JoyMessageHandler
from gerri.interface.joy_handler.protocols.rubbber_neck_joystick import RubberNeckJoyStickProtocol

from snow_robot_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO, FY_CONFIG, GIMBAL_CONFIG
from snow_robot_sub_controller import SnowRobotSubController
from snow_robot_cam import SnowCamera
from snow_robot_log import setup_logging, start_monitors, mark_joy, log, log_every

if TYPE_CHECKING:
    from snow_robot_sub_controller import SnowRobotSubController as _SnowRobotSubController


BUTTONS = {
    "A": 0,
    "B": 1,
    "X": 2,
    "Y": 3,
    "LB": 4,
    "RB": 5,
    "NONE_BUTTON1": 6,
    "NONE_BUTTON2": 7,
    "BACK": 8,
    "START": 9,
    "C1": 10,
    "C2": 11,
    "FORWARD": 12,
    "BACKWARD": 13,
    "LEFT": 14,
    "RIGHT": 15,
}

AXES = {
    "AXIS_1_X": 0,
    "AXIS_1_Y": 1,
    "AXIS_2_X": 2,
    "AXIS_2_Y": 3,
    "LT": 4,
    "RT": 5,
}


class SnowRobotBaseController(BaseController):
    """최신 BaseController를 감싼 remove_snow 전용 라우터.

    최신 BaseController는 /joy만 joystick handler로 보내고, 나머지는 target 기반 KRPC로 보낸다.
    RubberNeck에서 /move, /stop, key_mouse_control 등이 target 없이 올 수 있으므로
    여기서 sub_controller.on_message()로 직접 라우팅한다.
    """

    DIRECT_TOPICS = {
        "/joy", "joy",
        "/move", "move",
        "/cmd_vel", "cmd_vel",
        "/open", "open",
        "/stop", "stop",
        "key_mouse_control", "keyboard", "key", "keydown", "keyup", "keypress",
    }

    def __init__(self, robot_info, sub_controller: SnowRobotSubController):
        super().__init__(robot_info, sub_controller_list=[sub_controller])
        self.sub_controller = sub_controller

    def receive_message(self, message):
        if not isinstance(message, dict):
            return

        topic = message.get("topic")
        value = message.get("value")
        option = message.get("option")
        target = message.get("target")
        topic_text = str(topic or "").lower()

        if topic_text in ("/joy", "joy"):
            mark_joy()
            for handler in self.joystick_message_handler:
                try:
                    handler.receive_message(value or {})
                except Exception:
                    log_every("joy_handler_err", 5.0, logging.ERROR,
                              "[SnowRobotBaseController] joystick handler 예외", exc_info=True)
            # /joy는 RubberNeckJoyStickHandler에서 처리한다.
            # raw /joy를 다시 sub_controller로 넘기지 않는다.
            return

        if topic_text in self.DIRECT_TOPICS:
            if target is None or target == "all" or str(target) == self.robot_id or str(target) in self.robot_id:
                try:
                    self.sub_controller.on_message(
                        topic=topic,
                        value=value,
                        option=option,
                        raw=message,
                    )
                except Exception as e:
                    print(f"[SnowRobotBaseController] direct route error: {e}")
            return

        # target 기반 일반 RPC는 최신 BaseController 로직에 맡긴다.
        try:
            super().receive_message(message)
        except Exception as e:
            print(f"[SnowRobotBaseController] receive_message fallback error: {e}")


class RubberNeckJoyStickHandler(JoyMessageHandler):
    """RubberNeck joystick -> SnowRobotSubController.move(keys=...)."""

    def __init__(self, sub_controller=None):
        super().__init__(buttons=BUTTONS, axes=AXES, freq=0.05, psudo_release_event=True)
        self.sub_controller: _SnowRobotSubController = sub_controller
        self.robot_type = self.sub_controller.robot_category.lower() if self.sub_controller else "mobile"
        self.register_callback(self.handle_joy)

        self.last_message = ""
        self.last_keys: Set[str] = set()
        self._space_was_pressed = False
        self._back_was_pressed = False
        self._start_was_pressed = False
        self._c1_was_pressed = False
        self._c2_was_pressed = False

        self._velocity_linear = 0.7
        self._velocity_anguler = 0.4
        self._deadzone = 0.20
        print("[RubberNeckJoyStickHandler] relay key = SPACE toggle latch only")

    def safty_call(self, method_name: str):
        try:
            getattr(self.sub_controller, method_name)()
        except AttributeError:
            message = f"'{method_name}' method is not implemented in {self.robot_type} sub_controller"
            if message != self.last_message:
                print(message)
                self.last_message = message

    @staticmethod
    def _axis_value(axis, default: float = 0.0) -> float:
        try:
            if axis.is_pressed():
                return float(axis.get_value())
        except Exception:
            pass
        return default

    @staticmethod
    def _button_pressed(button) -> bool:
        try:
            return bool(button.is_pressed())
        except Exception:
            return False

    @staticmethod
    def _button_released(button) -> bool:
        try:
            return bool(button.is_released())
        except Exception:
            return False

    def _buttons_to_keys(self, handler: RubberNeckJoyStickProtocol) -> Set[str]:
        """RubberNeck 버튼을 키셋으로 변환한다.

        규칙:
        - FORWARD/BACKWARD/LEFT/RIGHT(D-pad 화살표)는 더 이상 주행(w/a/s/d)에
          매핑하지 않는다. 이 버튼들은 팬틸트 카메라 조작 전용이며,
          _update_gimbal()에서 별도로 처리한다.
        - SPACE 키는 RubberNeck에서 A 버튼 b[0]으로 들어온다.
        - SPACE는 누르는 순간에만 sub_controller.toggle_relay()를 호출한다.
        - relay 상태는 motor key와 분리되어 유지된다.
        """
        keys: Set[str] = set()

        # SPACE = A button b[0]
        space_pressed = self._button_pressed(handler.A)
        if space_pressed and not getattr(self, "_space_was_pressed", False):
            print("[RubberNeckJoyStickHandler] SPACE press edge")
            if self.sub_controller is not None and hasattr(self.sub_controller, "toggle_relay"):
                self.sub_controller.toggle_relay()

        self._space_was_pressed = space_pressed
        return keys

    def _update_gimbal(self, handler: RubberNeckJoyStickProtocol):
        """FORWARD/BACKWARD/LEFT/RIGHT(화살표) 버튼으로 팬틸트 카메라를 조작한다.
        주행 키(w/a/s/d)와는 완전히 분리된 채널로, sub_controller의
        gimbal_set_direction()에 현재 눌림 상태만 매 poll마다 전달한다.
        실제 각도 적분/이동은 sub_controller의 백그라운드 스레드가 담당한다."""
        if self.sub_controller is None or not hasattr(self.sub_controller, "gimbal_set_direction"):
            return

        x_dir = 0
        y_dir = 0
        if self._button_pressed(handler.RIGHT):
            x_dir += 1
        if self._button_pressed(handler.LEFT):
            x_dir -= 1
        if self._button_pressed(handler.FORWARD):
            y_dir += 1
        if self._button_pressed(handler.BACKWARD):
            y_dir -= 1

        last_dir = getattr(self, "_last_gimbal_dir_logged", (0, 0))
        if (x_dir, y_dir) != last_dir:
            print(f"[RubberNeckJoyStickHandler] gimbal dir -> x_dir={x_dir} y_dir={y_dir} sub_controller={'set' if self.sub_controller else 'None'}")
            self._last_gimbal_dir_logged = (x_dir, y_dir)

        try:
            self.sub_controller.gimbal_set_direction(x_dir, y_dir)
        except Exception as e:
            print(f"[RubberNeckJoyStickHandler] gimbal update error: {e}")

    def _update_gimbal_speed(self, handler: RubberNeckJoyStickProtocol):
        """BACK(버튼6) = 팬틸트 회전속도 감소, START(버튼7) = 회전속도 증가.

        누르는 순간(edge)에만 한 번씩 적용한다 (길게 눌러도 계속 깎이지 않음).
        """
        if self.sub_controller is None or not hasattr(self.sub_controller, "gimbal_adjust_speed"):
            return

        back_pressed = self._button_pressed(handler.BACK)
        if back_pressed and not self._back_was_pressed:
            print("[RubberNeckJoyStickHandler] BACK press edge -> gimbal speed down")
            self.sub_controller.gimbal_adjust_speed(-1)
        self._back_was_pressed = back_pressed

        start_pressed = self._button_pressed(handler.START)
        if start_pressed and not self._start_was_pressed:
            print("[RubberNeckJoyStickHandler] START press edge -> gimbal speed up")
            self.sub_controller.gimbal_adjust_speed(+1)
        self._start_was_pressed = start_pressed

    def _update_drive_speed(self, handler: RubberNeckJoyStickProtocol):
        """C1(버튼 9) = 주행 속도 감소, C2(버튼 0) = 주행 속도 증가.

        누르는 순간(edge)에만 한 번씩 적용한다. 팬틸트 속도(BACK/START)와는 별개다.
        """
        if self.sub_controller is None or not hasattr(self.sub_controller, "drive_adjust_speed"):
            return

        c1_pressed = self._button_pressed(handler.C1)
        if c1_pressed and not self._c1_was_pressed:
            print("[RubberNeckJoyStickHandler] C1 press edge -> drive speed down")
            self.sub_controller.drive_adjust_speed(-1)
        self._c1_was_pressed = c1_pressed

        c2_pressed = self._button_pressed(handler.C2)
        if c2_pressed and not self._c2_was_pressed:
            print("[RubberNeckJoyStickHandler] C2 press edge -> drive speed up")
            self.sub_controller.drive_adjust_speed(+1)
        self._c2_was_pressed = c2_pressed

    def handle_joy(self, handler: RubberNeckJoyStickProtocol):
        # JoyMessageHandler 메인 루프는 콜백 예외를 처리하지 않아 스레드가 죽을 수 있다.
        # 예외가 나면 기록하고 모터/짐벌을 정지시킨 뒤 계속 동작한다.
        try:
            self._handle_joy(handler)
        except Exception:
            log_every("handle_joy_err", 5.0, logging.ERROR,
                      "[RubberNeckJoyStickHandler] handle_joy 예외 -> 모터/짐벌 정지", exc_info=True)
            try:
                if self.sub_controller is not None:
                    self.sub_controller.move(keys=set(), vx=0.0, vy=0.0, vth=0.0)
                    self.sub_controller.gimbal_set_direction(0, 0)
            except Exception:
                log_every("handle_joy_stop_err", 5.0, logging.ERROR, "정지 명령도 실패", exc_info=True)

    def _handle_joy(self, handler: RubberNeckJoyStickProtocol):
        vx = 0.0
        vy = 0.0
        vth = 0.0
        keys: Set[str] = set()

        # 왼쪽 스틱 Y: 전/후진. 기존 테스트 기준 axes[1] < 0 = 전진.
        axis_y = self._axis_value(handler.AXIS_1_Y)
        if axis_y < -self._deadzone:
            vx = self._velocity_linear
            keys.add("w")
        elif axis_y > self._deadzone:
            vx = -self._velocity_linear
            keys.add("s")

        # 오른쪽 스틱 X 우선 회전. 없으면 왼쪽 스틱 X 보조 사용.
        axis_turn = self._axis_value(handler.AXIS_2_X)
        if abs(axis_turn) <= self._deadzone:
            axis_turn = self._axis_value(handler.AXIS_1_X)

        if axis_turn < -self._deadzone:
            vth = self._velocity_anguler
            keys.add("a")
        elif axis_turn > self._deadzone:
            vth = -self._velocity_anguler
            keys.add("d")

        keys |= self._buttons_to_keys(handler)

        # 화살표(D-pad) 버튼은 주행이 아니라 팬틸트 카메라 전용으로 라우팅한다.
        self._update_gimbal(handler)
        # BACK/START(버튼 6/7)로 팬틸트 회전속도 조절.
        self._update_gimbal_speed(handler)
        # C1/C2(버튼 9/0)로 주행 속도(pwm) 조절.
        self._update_drive_speed(handler)

        if self.sub_controller is not None:
            self.sub_controller.move(keys=keys, vx=vx, vy=vy, vth=vth)

        if keys != self.last_keys:
            print(
                f"[RubberNeckJoyStickHandler] keys={sorted(keys) or '-'} "
                f"relay_latched='controller' "
                f"vx={vx:.2f} vy={vy:.2f} vth={vth:.2f} "
                f"linear={self._velocity_linear:.2f} angular={self._velocity_anguler:.2f}"
            )
            self.last_keys = set(keys)

        # 버튼 사용처: A=릴레이 토글, 방향키=팬틸트, BACK/START=팬틸트 속도,
        # C1/C2=주행 속도. 스틱은 주행(w/a/s/d)에 사용한다.



def main():
    setup_logging()
    print("[remove_snow] starting")
    print(f"[remove_snow] current dir    = {CURRENT_DIR}")
    print(f"[remove_snow] robot id       = {ROBOT_INFO.get('id')}")
    print(f"[remove_snow] robot category = {ROBOT_INFO.get('category')}")
    print("[remove_snow] VIDEO_INFO     =", VIDEO_INFO)
    print("[remove_snow] FY_CONFIG      =", FY_CONFIG)
    print("[remove_snow] GIMBAL_CONFIG  =", GIMBAL_CONFIG)
    print("[remove_snow] RELAY_PORT env =", os.getenv("RELAY_PORT"))
    print("[remove_snow] GIMBAL_PORT env=", os.getenv("GIMBAL_PORT"))
    print("[remove_snow] RELAY KEY      = SPACE toggle latch only")
    
    video_info = {}

    # 자동 복구 카메라 래퍼 (공용 CameraManager의 스레드 사망 -> 영상 멈춤 문제 대응)
    # 끄려면: SNOW_CAM_WATCHDOG=0
    if not video_info and os.getenv("SNOW_CAM_WATCHDOG", "1") == "1":
        for name, cfg in VIDEO_INFO.items():
            try:
                video_info[name] = {"source": SnowCamera(
                    camera_name=name, camera_index=cfg["source"],
                    width=cfg.get("width", 640), height=cfg.get("height", 480),
                    fps=cfg.get("fps", 30))}
                print(f"[remove_snow] self-healing camera '{name}' (source={cfg['source']})")
            except Exception as e:
                print(f"[remove_snow] camera '{name}' init failed: {e}")

    if not video_info:
        print("[remove_snow] WARNING: no camera available, falling back")
        video_info = VIDEO_INFO

    daemon = AdaptiveNetworkDaemon(
        robot_info=ROBOT_INFO,
        network="ketirtc",
        command="command",
        video_info=video_info,
        audio_info=AUDIO_INFO,
    )
    daemon.connect()

    sub_controller = SnowRobotSubController(robot_info=ROBOT_INFO, fy_config=FY_CONFIG, gimbal_config=GIMBAL_CONFIG)
    base_controller = SnowRobotBaseController(ROBOT_INFO, sub_controller=sub_controller)
    sub_controller.base_controller = base_controller

    joy_handler = RubberNeckJoyStickHandler(sub_controller=sub_controller)
    base_controller.joystick_message_handler.append(joy_handler)

    base_controller.connect()
    joy_handler.connect()

    # 운행 중 장애 감시 (네트워크 / 스레드 / 포트 / 카메라 / 온도 / 메모리)
    ports = {"motor": FY_CONFIG.get("port")}
    if GIMBAL_CONFIG.get("enabled", True):
        ports["gimbal"] = GIMBAL_CONFIG.get("port")
    cams = {n: v["source"] for n, v in video_info.items() if hasattr(v.get("source"), "health")}
    start_monitors(sub_controller, cams, ports)

    print("[remove_snow] running. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[remove_snow] Ctrl+C")
    finally:
        try:
            base_controller.disconnect()
        except Exception as e:
            print(f"[remove_snow] base_controller disconnect error: {e}")

        # JoyMessageHandler.disconnect()는 내부 thread 종료 플래그가 없어 join에서 막힐 수 있다.
        # 따라서 프로세스 종료 시 daemon thread 자연 종료에 맡긴다.

        try:
            if hasattr(daemon, "disconnect"):
                daemon.disconnect()
        except Exception as e:
            print(f"[remove_snow] daemon disconnect error: {e}")


if __name__ == "__main__":
    main()
