# -*- coding: utf-8 -*-
"""snow_robot 전용 자동 복구 카메라 래퍼.

공용 라이브러리 CameraManager(_and_.keti_rtc.robot.cam_manager)의 문제:
  1) 프레임 읽기에 한 번 실패하면 restart()가 캡처 스레드 자기 자신을 join()하려다
     RuntimeError("cannot join current thread")로 스레드가 죽는다.
     이후 get_frame()은 마지막 프레임을 계속 돌려줘서 영상이 멈춘 채 송출된다.
  2) 첫 프레임 전에는 get_frame()이 None을 돌려줘서, WebRTC 트랙(recv)이
     MediaStreamError를 내고 영상 송출이 끝날 수 있다.

이 클래스는 공용 코드를 수정하지 않고 CameraManager를 감싸서:
  - 캡처 스레드가 죽으면 새 CameraManager로 자동 재시작하고(백오프 2s -> 10s),
  - 프레임이 없거나 오래되면 'NO SIGNAL' 화면을 대신 내보낸다(None을 돌려주지 않음).

사용:
    video_info["front_cam"] = {"source": SnowCamera("front_cam", camera_index=1)}
webrtc_robot.create_channels()는 source가 int가 아니면 그 객체를 그대로 쓴다.
set_resolution / set_fps / set_publish_frame / save_frame 등은 현재 CameraManager로 위임된다.
"""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np
from pubsub import pub

from _and_.keti_rtc.robot.cam_manager import CameraManager


class SnowCamera:
    def __init__(self, camera_name, camera_index, width=640, height=480, fps=30,
                 stale_timeout=2.0, check_interval=0.5, retry_min=2.0, retry_max=10.0):
        self.camera_name = camera_name
        self.camera_index = camera_index
        self.width = int(width)
        self.height = int(height)
        self.fps = fps

        self.stale_timeout = float(stale_timeout)   # 이 시간 넘게 새 프레임이 없으면 NO SIGNAL
        self.check_interval = float(check_interval)
        self.retry_min = float(retry_min)
        self.retry_max = float(retry_max)

        self._cam = None
        self._started_at = 0.0
        self._restart_count = 0
        self._shape = (self.height, self.width, 3)
        self._no_signal = None
        self._running = True

        # 첫 오픈이 실패해도 예외를 밖으로 내보내지 않는다. 감시 스레드가 재시도한다.
        try:
            self._open()
        except Exception as e:
            print(f"[SnowCamera:{self.camera_name}] initial open failed: {e}")

        self._thread = threading.Thread(
            target=self._watch, name=f"SnowCamera-{camera_name}", daemon=True
        )
        self._thread.start()

    # ------------------------------------------------------------------
    # WebRTC 쪽에서 쓰는 인터페이스
    # ------------------------------------------------------------------
    def get_frame(self):
        """항상 ndarray를 돌려준다. 신호가 없으면 NO SIGNAL 화면."""
        cam = self._cam
        frame = getattr(cam, "last_frame", None) if cam is not None else None
        age = self._frame_age()
        if frame is not None and age is not None and age <= self.stale_timeout:
            self._shape = frame.shape
            return frame
        return self._no_signal_frame()

    def __getattr__(self, name):
        # set_resolution / set_fps / set_publish_frame / save_frame 등은 현재 카메라로 위임
        if name.startswith("_"):
            raise AttributeError(name)
        cam = self.__dict__.get("_cam")
        if cam is None:
            raise AttributeError(name)
        return getattr(cam, name)

    def health(self):
        """snow_robot_log 헬스 체크용: 캡처 스레드 생존, 마지막 프레임 경과(초), 재시작 횟수."""
        cam = self._cam
        th = getattr(cam, "thread", None) if cam is not None else None
        return {
            "alive": bool(th is not None and th.is_alive()),
            "age": self._frame_age(),
            "restarts": self._restart_count,
        }

    def stop(self):
        self._running = False
        cam = self._cam
        if cam is not None:
            try:
                cam.stop()
            except Exception as e:
                print(f"[SnowCamera:{self.camera_name}] stop error: {e}")

    # ------------------------------------------------------------------
    # 내부
    # ------------------------------------------------------------------
    def _open(self):
        cam = CameraManager(
            camera_index=self.camera_index,
            width=self.width,
            height=self.height,
            fps=self.fps,
            camera_name=self.camera_name,
            auto_start=True,
        )
        self._cam = cam
        self._started_at = time.time()

    def _frame_age(self):
        cam = self._cam
        t = getattr(cam, "last_frame_time", None) if cam is not None else None
        return None if t is None else time.time() - t

    def _no_signal_frame(self):
        h, w = self._shape[:2]
        img = self._no_signal
        if img is None or img.shape[:2] != (h, w):
            img = np.zeros((h, w, 3), np.uint8)
            scale = max(0.6, w / 640.0 * 1.2)
            cv2.putText(img, "NO SIGNAL", (int(w * 0.25), int(h * 0.5)),
                        cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(img, str(self.camera_name), (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (160, 160, 160), 1, cv2.LINE_AA)
            self._no_signal = img
        return img

    def _watch(self):
        delay = self.retry_min
        last_warn = 0.0
        while self._running:
            time.sleep(self.check_interval)
            try:
                cam = self._cam
                alive = (
                    cam is not None
                    and getattr(cam, "thread", None) is not None
                    and cam.thread.is_alive()
                )
                if alive:
                    age = self._frame_age()
                    if age is not None and age <= self.stale_timeout:
                        delay = self.retry_min          # 정상: 백오프 초기화
                    elif time.time() - last_warn > 10.0 and time.time() - self._started_at > 5.0:
                        # 스레드는 살아 있지만 프레임이 안 온다(read()가 막힘 등).
                        # 열려 있는 장치를 다른 스레드에서 강제로 닫는 건 위험해서 경고만 한다.
                        last_warn = time.time()
                        shown = "없음" if age is None else f"{age:.1f}s"
                        print(f"[SnowCamera:{self.camera_name}] 새 프레임 없음 (마지막 프레임 이후 {shown})")
                    continue

                self._restart(delay)
                delay = min(delay * 2.0, self.retry_max)
            except Exception as e:
                print(f"[SnowCamera:{self.camera_name}] watchdog error: {e}")

    def _restart(self, delay):
        old = self._cam
        self._restart_count += 1
        print(f"[SnowCamera:{self.camera_name}] 캡처 스레드 종료 감지 -> "
              f"{delay:.0f}s 뒤 재시작 (#{self._restart_count})")

        if old is not None:
            old.running = False
            try:
                if getattr(old, "cap", None) is not None:
                    old.cap.release()
            except Exception:
                pass
            for topic, name in (("set_publish_frame", "set_publish_frame"), ("save_frame", "save_frame")):
                try:
                    pub.unsubscribe(getattr(old, name), topic)
                except Exception:
                    pass
            # RubberNeck에서 해상도/fps를 바꿨다면 그 값을 이어받는다.
            try:
                self.width = int(old.width)
                self.height = int(old.height)
                self.fps = int(old.fps)
            except Exception:
                pass

        time.sleep(delay)
        if not self._running:
            return
        self._open()
        print(f"[SnowCamera:{self.camera_name}] 재시작 완료 (index={self.camera_index})")
