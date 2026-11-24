# -*- coding: utf-8 -*-
"""IMX219 / CamArray HAT (nvarguscamerasrc) 전용 카메라 매니저.

CameraManager(cam_manager.py)와 동일한 인터페이스(.get_frame() / .last_frame,
set_resolution, set_fps, save_frame, pubsub 연동)를 제공하되, 내부적으로
cv2.VideoCapture(camera_index) 대신 nvarguscamerasrc gstreamer 파이프라인을 사용한다.

CamArray HAT는 카메라 4개를 이미 하드웨어/드라이버 레벨에서 하나의 영상으로
합쳐서 sensor-id=0 하나로 출력하므로, 여기서는 "카메라 1개"로만 다루면 된다.

webrtc_robot.py의 다음 분기 덕분에 VIDEO_INFO["source"]에 이 클래스의 인스턴스를
그대로 넣으면 CameraManager를 거치지 않고 바로 사용된다:

    if type(cfg["source"]) != int:
        cam = cfg["source"]
    else:
        cam = CameraManager(camera_index=cfg["source"], ...)
"""

import cv2
import numpy as np
import time
import threading
import logging
from pubsub import pub
from raasdata.synclogger import get_global_mcap_logger

logger = logging.getLogger(__name__)


# label_quadrants.py --live로 확인한 사분면(0/1/2/3) -> 실제 방향 매핑.
# 카메라 배선이나 HAT 채널 순서가 바뀌면 이 값만 수정하면 된다.
#   0(좌상) = front, 1(우상) = back, 2(좌하) = left, 3(우하) = right
QUADRANT_MAP = {
    "front": 0,
    "back": 1,
    "left": 2,
    "right": 3,
}


def split_quadrants(frame):
    """2x2 합성 프레임을 4등분. 0:좌상, 1:우상, 2:좌하, 3:우하"""
    h, w = frame.shape[:2]
    hh, hw = h // 2, w // 2
    return {
        0: frame[0:hh, 0:hw],
        1: frame[0:hh, hw:w],
        2: frame[hh:h, 0:hw],
        3: frame[hh:h, hw:w],
    }


class AvmComposer:
    """CamArray HAT 2x2 합성 프레임을 어라운드뷰(AVM) 스타일로 재배치하는 합성기.

    가운데 직사각형(robot_w x robot_h)을 로봇 자리로 비워두고,
    전/후/좌/우 카메라 영상을 각각 사다리꼴(절두체 단면) 형태로 "원근 변환"해
    바깥 테두리까지 채운다. 원근 왜곡 보정이 들어가는 대신 warpPerspective를
    프레임마다 4번 호출해야 해서 그리드 스타일 중 가장 무겁다.

    캔버스는 width x height 직사각형(정사각형이 아니어도 됨)이다.

    관제(WebRTC 인코딩)와 CPU를 나눠 써야 하는 상황에서 프레임레이트가 떨어지면
    TrapezoidGridComposer(왜곡 보정 없이 같은 모양만 흉내) 또는
    SimpleGridComposer(사각 블록, 가장 가벼움)로 바꾸는 것을 고려할 것.
    """

    def __init__(self, width=800, height=800, robot_w=160, robot_h=280,
                 line_color=(60, 60, 60), line_thickness=2,
                 robot_color=(40, 40, 40), robot_outline=(0, 200, 255)):
        self.width = width
        self.height = height
        self.robot_w = robot_w
        self.robot_h = robot_h
        self.line_color = line_color
        self.line_thickness = line_thickness
        self.robot_color = robot_color
        self.robot_outline = robot_outline

        cx, cy = width // 2, height // 2
        self.rx0, self.rx1 = cx - robot_w // 2, cx + robot_w // 2
        self.ry0, self.ry1 = cy - robot_h // 2, cy + robot_h // 2

        specs = {
            # front: 위쪽 사다리꼴 (원본 이미지의 아래쪽 절반 정도를 씀 = 카메라 하단이 로봇에 가까움)
            "front": (
                [(0, height * 0.4), (width, height * 0.4), (width, height), (0, height)],
                [(0, 0), (width, 0), (self.rx1, self.ry0), (self.rx0, self.ry0)],
            ),
            # back: 아래쪽 사다리꼴
            "back": (
                [(0, 0), (width, 0), (width, height * 0.6), (0, height * 0.6)],
                [(self.rx0, self.ry1), (self.rx1, self.ry1), (width, height), (0, height)],
            ),
            # left: 왼쪽 사다리꼴
            "left": (
                [(width * 0.4, 0), (width, 0), (width, height), (width * 0.4, height)],
                [(0, 0), (self.rx0, self.ry0), (self.rx0, self.ry1), (0, height)],
            ),
            # right: 오른쪽 사다리꼴
            "right": (
                [(0, 0), (width * 0.6, 0), (width * 0.6, height), (0, height)],
                [(self.rx1, self.ry0), (width, 0), (width, height), (self.rx1, self.ry1)],
            ),
        }

        # 워프 행렬 M과 목적지 마스크를 미리 한 번만 계산해서 캐싱
        self._M = {}
        self._masks = {}
        for name, (src_quad, dst_quad) in specs.items():
            src_pts = np.float32(src_quad)
            dst_pts = np.float32(dst_quad)
            self._M[name] = cv2.getPerspectiveTransform(src_pts, dst_pts)
            mask = np.zeros((height, width), dtype=np.uint8)
            cv2.fillConvexPoly(mask, np.int32(dst_quad), 255)
            self._masks[name] = mask.astype(bool)

        # 정적인 테두리/로봇 아이콘 레이어도 미리 한 번만 그려서 캐싱
        self._overlay, self._overlay_mask = self._build_static_overlay()

    def _build_static_overlay(self):
        width, height, rx0, rx1, ry0, ry1 = self.width, self.height, self.rx0, self.rx1, self.ry0, self.ry1
        overlay = np.zeros((height, width, 3), dtype=np.uint8)

        # 로봇 박스 테두리 + 사분면 대각선 구분선만 (화면을 가로지르는 좌우 확장선은 그리지 않음)
        cv2.rectangle(overlay, (rx0, ry0), (rx1, ry1), self.line_color, self.line_thickness)
        cv2.line(overlay, (0, 0), (rx0, ry0), self.line_color, 1)
        cv2.line(overlay, (width, 0), (rx1, ry0), self.line_color, 1)
        cv2.line(overlay, (0, height), (rx0, ry1), self.line_color, 1)
        cv2.line(overlay, (width, height), (rx1, ry1), self.line_color, 1)

        # 중앙 로봇 아이콘 (탑뷰 사각형 + 전방 화살표)
        cv2.rectangle(overlay, (rx0, ry0), (rx1, ry1), self.robot_color, -1)
        cv2.rectangle(overlay, (rx0, ry0), (rx1, ry1), self.robot_outline, 2)
        cx = width // 2
        tip = (cx, ry0 + int(self.robot_h * 0.12))
        left_pt = (cx - self.robot_w // 4, ry0 + int(self.robot_h * 0.32))
        right_pt = (cx + self.robot_w // 4, ry0 + int(self.robot_h * 0.32))
        cv2.fillConvexPoly(overlay, np.array([tip, left_pt, right_pt], dtype=np.int32), self.robot_outline)

        # overlay에 실제로 그려진 픽셀만 마스크로 표시 (그 외 영역은 canvas 원본 유지)
        overlay_mask = np.any(overlay != 0, axis=2)
        return overlay, overlay_mask

    def compose(self, frame, quadrant_map=QUADRANT_MAP):
        quadrants = split_quadrants(frame)
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        resized = {k: cv2.resize(v, (self.width, self.height)) for k, v in quadrants.items()}

        for name in ("front", "back", "left", "right"):
            src_img = resized[quadrant_map[name]]
            warped = cv2.warpPerspective(src_img, self._M[name], (self.width, self.height))
            mask = self._masks[name]
            canvas[mask] = warped[mask]

        canvas[self._overlay_mask] = self._overlay[self._overlay_mask]
        return canvas


class TrapezoidGridComposer:
    """AvmComposer와 같은 사다리꼴(사각뿔대) 모양 레이아웃을 쓰되,
    warpPerspective(원근 변환)를 아예 하지 않는 경량 버전.

    각 카메라 사분면 영상을 "사다리꼴의 바운딩 박스 크기"로만 리사이즈한 뒤,
    그 박스 안에서 사다리꼴 모양의 마스크로 잘라 붙인다. 즉 이미지가 원근에
    맞게 늘어나지 않고 화면 가장자리가 잘려나갈 뿐이다(왜곡 보정 없음, 크롭만 발생).

    캔버스는 width x height 직사각형(정사각형이 아니어도 됨)이다. 정사각형
    캔버스에서는 네 사다리꼴이 캔버스를 다 덮지 못하고 귀퉁이가 비어 낭비되는
    느낌이 들 수 있는데, width를 height보다 크게(가로로 넓게) 잡으면 그 여백
    비율이 줄어들고 러버넥 타일(보통 가로가 긴 비율)에도 더 잘 맞는다.
    """

    def __init__(self, width=800, height=800, robot_w=160, robot_h=280,
                 line_color=(60, 60, 60), line_thickness=2,
                 robot_color=(40, 40, 40), robot_outline=(0, 200, 255)):
        self.width = width
        self.height = height
        self.robot_w = robot_w
        self.robot_h = robot_h
        self.line_color = line_color
        self.line_thickness = line_thickness
        self.robot_color = robot_color
        self.robot_outline = robot_outline

        cx, cy = width // 2, height // 2
        self.rx0, self.rx1 = cx - robot_w // 2, cx + robot_w // 2
        self.ry0, self.ry1 = cy - robot_h // 2, cy + robot_h // 2

        # AvmComposer와 동일한 사다리꼴 목적지 좌표 (원근 변환용 src_quad는 필요 없음)
        dst_quads = {
            "front": [(0, 0), (width, 0), (self.rx1, self.ry0), (self.rx0, self.ry0)],
            "back":  [(self.rx0, self.ry1), (self.rx1, self.ry1), (width, height), (0, height)],
            "left":  [(0, 0), (self.rx0, self.ry0), (self.rx0, self.ry1), (0, height)],
            "right": [(self.rx1, self.ry0), (width, 0), (width, height), (self.rx1, self.ry1)],
        }

        # 각 사다리꼴의 바운딩 박스(bx, by, bw, bh)와, 그 박스 안 좌표계 기준
        # 마스크를 미리 계산해서 캐싱. resize/마스크 적용을 전부 이 작은 박스
        # 크기로만 수행해서 불필요한 연산을 없앤다.
        self._bbox = {}
        self._masks = {}
        for name, dst_quad in dst_quads.items():
            pts = np.int32(dst_quad)
            bx, by, bw, bh = cv2.boundingRect(pts)
            # cv2.boundingRect가 캔버스 경계(width/height)를 살짝 넘는 폭/높이를
            # 돌려줄 수 있어서(예: bw가 width+1), canvas 슬라이싱과 마스크 크기가
            # 어긋나 "boolean index did not match" 에러가 났었다. 캔버스 안으로 clamp.
            bx = max(0, bx)
            by = max(0, by)
            bw = min(bw, width - bx)
            bh = min(bh, height - by)
            self._bbox[name] = (bx, by, bw, bh)
            local_pts = pts - np.array([bx, by])
            mask = np.zeros((bh, bw), dtype=np.uint8)
            cv2.fillConvexPoly(mask, local_pts, 255)
            self._masks[name] = mask.astype(bool)

        self._overlay, self._overlay_mask = self._build_static_overlay()

    def _build_static_overlay(self):
        width, height, rx0, rx1, ry0, ry1 = self.width, self.height, self.rx0, self.rx1, self.ry0, self.ry1
        overlay = np.zeros((height, width, 3), dtype=np.uint8)

        cv2.rectangle(overlay, (rx0, ry0), (rx1, ry1), self.line_color, self.line_thickness)
        cv2.line(overlay, (0, 0), (rx0, ry0), self.line_color, 1)
        cv2.line(overlay, (width, 0), (rx1, ry0), self.line_color, 1)
        cv2.line(overlay, (0, height), (rx0, ry1), self.line_color, 1)
        cv2.line(overlay, (width, height), (rx1, ry1), self.line_color, 1)

        cv2.rectangle(overlay, (rx0, ry0), (rx1, ry1), self.robot_color, -1)
        cv2.rectangle(overlay, (rx0, ry0), (rx1, ry1), self.robot_outline, 2)
        cx = width // 2
        tip = (cx, ry0 + int(self.robot_h * 0.12))
        left_pt = (cx - self.robot_w // 4, ry0 + int(self.robot_h * 0.32))
        right_pt = (cx + self.robot_w // 4, ry0 + int(self.robot_h * 0.32))
        cv2.fillConvexPoly(overlay, np.array([tip, left_pt, right_pt], dtype=np.int32), self.robot_outline)

        overlay_mask = np.any(overlay != 0, axis=2)
        return overlay, overlay_mask

    def compose(self, frame, quadrant_map=QUADRANT_MAP):
        quadrants = split_quadrants(frame)
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        for name in ("front", "back", "left", "right"):
            bx, by, bw, bh = self._bbox[name]
            # 사분면 원본을 캔버스 전체가 아니라 바운딩 박스 크기로만 리사이즈
            src_img = cv2.resize(quadrants[quadrant_map[name]], (bw, bh))
            mask = self._masks[name]
            region = canvas[by:by + bh, bx:bx + bw]
            region[mask] = src_img[mask]

        canvas[self._overlay_mask] = self._overlay[self._overlay_mask]
        return canvas


class SimpleGridComposer:
    """warpPerspective 없이 resize + 붙여넣기만으로 구성하는 경량 그리드 합성기.

    각 카메라 영상이 사각 블록(위/아래는 가로 전체 폭, 좌/우는 중간 높이만)으로
    배치된다. TrapezoidGridComposer보다도 더 단순한 형태이며 연산 비용은 동급으로 가볍다.
    캔버스는 width x height 직사각형(정사각형이 아니어도 됨)이다.

    레이아웃:
        ┌───────────────┐
        │     front      │
        ├────┬─────┬─────┤
        │left│robot│right│
        ├────┴─────┴─────┤
        │      back       │
        └───────────────┘
    """

    def __init__(self, width=600, height=600, robot_w=160, robot_h=160,
                 line_color=(60, 60, 60), line_thickness=2,
                 robot_color=(40, 40, 40), robot_outline=(0, 200, 255)):
        self.width = width
        self.height = height
        self.robot_w = robot_w
        self.robot_h = robot_h

        cx, cy = width // 2, height // 2
        self.rx0, self.rx1 = cx - robot_w // 2, cx + robot_w // 2
        self.ry0, self.ry1 = cy - robot_h // 2, cy + robot_h // 2

        # 상/하/좌/우 4개 영역의 크기를 미리 계산 (프레임마다 재계산하지 않도록 캐싱)
        self.top_h = self.ry0
        self.bottom_h = height - self.ry1
        self.side_w = self.rx0
        self.mid_h = self.ry1 - self.ry0

        self._overlay, self._overlay_mask = self._build_static_overlay(
            line_color, line_thickness, robot_color, robot_outline
        )

    def _build_static_overlay(self, line_color, line_thickness, robot_color, robot_outline):
        width, height, rx0, rx1, ry0, ry1 = self.width, self.height, self.rx0, self.rx1, self.ry0, self.ry1
        overlay = np.zeros((height, width, 3), dtype=np.uint8)

        cv2.rectangle(overlay, (rx0, ry0), (rx1, ry1), line_color, line_thickness)
        cv2.rectangle(overlay, (rx0, ry0), (rx1, ry1), robot_color, -1)
        cv2.rectangle(overlay, (rx0, ry0), (rx1, ry1), robot_outline, 2)

        cx = width // 2
        tip = (cx, ry0 + int(self.robot_h * 0.12))
        left_pt = (cx - self.robot_w // 4, ry0 + int(self.robot_h * 0.32))
        right_pt = (cx + self.robot_w // 4, ry0 + int(self.robot_h * 0.32))
        cv2.fillConvexPoly(overlay, np.array([tip, left_pt, right_pt], dtype=np.int32), robot_outline)

        overlay_mask = np.any(overlay != 0, axis=2)
        return overlay, overlay_mask

    def compose(self, frame, quadrant_map=QUADRANT_MAP):
        quadrants = split_quadrants(frame)
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # front: 맨 위, 가로 전체 폭
        front = cv2.resize(quadrants[quadrant_map["front"]], (self.width, self.top_h))
        canvas[0:self.top_h, :] = front

        # back: 맨 아래, 가로 전체 폭
        back = cv2.resize(quadrants[quadrant_map["back"]], (self.width, self.bottom_h))
        canvas[self.height - self.bottom_h:self.height, :] = back

        # left: 중간 왼쪽
        left = cv2.resize(quadrants[quadrant_map["left"]], (self.side_w, self.mid_h))
        canvas[self.ry0:self.ry1, 0:self.side_w] = left

        # right: 중간 오른쪽
        right = cv2.resize(quadrants[quadrant_map["right"]], (self.side_w, self.mid_h))
        canvas[self.ry0:self.ry1, self.width - self.side_w:self.width] = right

        canvas[self._overlay_mask] = self._overlay[self._overlay_mask]
        return canvas


def build_nvargus_pipeline(sensor_id=0, capture_width=1920, capture_height=1080,
                            capture_fps=30, display_width=None, display_height=None,
                            flip_method=0):
    """nvarguscamerasrc -> BGR appsink gstreamer 파이프라인 문자열 생성."""
    display_width = display_width or capture_width
    display_height = display_height or capture_height
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"framerate=(fraction){capture_fps}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        f"videoconvert ! video/x-raw, format=(string)BGR ! "
        f"appsink drop=1 max-buffers=1 sync=false"
    )


class NvArgusCameraManager:
    """CameraManager와 동일한 사용법의 nvarguscamerasrc 기반 카메라 매니저."""

    def __init__(self, sensor_id=0, capture_width=1920, capture_height=1080,
                 capture_fps=30, display_width=None, display_height=None,
                 flip_method=0, camera_name='imx219', auto_start=True,
                 x_grid=False, grid_width=800, grid_height=800,
                 robot_w=160, robot_h=280,
                 grid_style="avm"):
        """
        :param x_grid: True면 CamArray HAT의 2x2(+) 합성 프레임을 그리드 스타일로
                        변환해서 get_frame()/last_frame에 담는다.
        :param grid_width: 그리드 출력 캔버스 가로(px)
        :param grid_height: 그리드 출력 캔버스 세로(px). grid_width와 달라도 됨
                        (예: grid_width=960, grid_height=540 처럼 가로로 넓게 잡으면
                        정사각형일 때보다 사다리꼴 사이 귀퉁이 여백 비율이 줄어든다).
        :param robot_w: 중앙 로봇 아이콘 영역 가로(px)
        :param robot_h: 중앙 로봇 아이콘 영역 세로(px)
        :param grid_style: 그리드 합성 방식.
            "avm"       - 사다리꼴 + 원근 변환(warpPerspective). 화질/원근감 가장 좋지만
                          프레임당 warpPerspective 4회로 CPU 비용이 가장 크다.
            "trapezoid" - avm과 같은 사다리꼴 모양이지만 원근 변환 없이 크롭만 한다.
                          모양은 동일, 연산은 SimpleGridComposer급으로 가볍다.
            "simple"    - 사각 블록(위/아래 전체 폭, 좌/우 중간 높이) 레이아웃. 가장 단순하고 가볍다.
        """
        self.x_grid = x_grid
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.robot_w = robot_w
        self.robot_h = robot_h
        self.grid_style = grid_style
        self.camera_name = camera_name
        self.sensor_id = sensor_id
        self.width = display_width or capture_width
        self.height = display_height or capture_height
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.fps = capture_fps
        self.flip_method = flip_method

        # 그리드 합성기는 무거운 계산(마스크/워프행렬 등)을 한 번만 하도록
        # 인스턴스 생성 시점에 캐싱해서 만든다.
        self._avm = None
        if x_grid:
            if grid_style == "simple":
                self._avm = SimpleGridComposer(width=grid_width, height=grid_height,
                                                robot_w=robot_w, robot_h=robot_h)
            elif grid_style == "trapezoid":
                self._avm = TrapezoidGridComposer(width=grid_width, height=grid_height,
                                                   robot_w=robot_w, robot_h=robot_h)
            else:
                self._avm = AvmComposer(width=grid_width, height=grid_height,
                                         robot_w=robot_w, robot_h=robot_h)

        self.running = False
        self.last_frame = None
        self.last_frame_time = None
        self.cap = None
        self.thread = None
        self._lock = threading.Lock()
        self.publish_frame = False

        pub.subscribe(self.set_publish_frame, "set_publish_frame")
        pub.subscribe(self.save_frame, "save_frame")

        if auto_start:
            self.start()

    def _open_capture(self):
        pipeline = build_nvargus_pipeline(
            sensor_id=self.sensor_id,
            capture_width=self.capture_width,
            capture_height=self.capture_height,
            capture_fps=self.fps,
            display_width=self.width,
            display_height=self.height,
            flip_method=self.flip_method,
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            raise RuntimeError(
                f"❌ nvarguscamerasrc(sensor-id={self.sensor_id}) 열기 실패. "
                f"nvargus-daemon 상태 또는 다른 프로세스의 카메라 점유 여부를 확인하세요."
            )
        return cap

    def start(self):
        if self.running:
            logger.warning("📷 이미 실행 중")
            return
        self.running = True
        self.cap = self._open_capture()
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        logger.info(f"📷 IMX219(sensor-id={self.sensor_id}) 시작됨 ({self.width}x{self.height}@{self.fps})")

    def _capture_loop(self):
        while self.running:
            try:
                with self._lock:
                    ret, frame = self.cap.read()
                if not ret:
                    raise RuntimeError("❌ 프레임 읽기 실패")
                if self.x_grid:
                    frame = self._avm.compose(frame, QUADRANT_MAP)
                self.last_frame = frame
                self.last_frame_time = time.time()
                if self.publish_frame:
                    pub.sendMessage("video_stream", cam_id=self.camera_name, frame=frame)
            except Exception as e:
                logger.error(f"⚠️ 캡처 오류 발생: {e}")
                self.running = False
                if self.cap:
                    self.cap.release()
                threading.Thread(target=self._delayed_restart, daemon=True).start()
                return  # 현재 스레드는 여기서 종료

    def _delayed_restart(self):
        time.sleep(2)
        self.start()

    def set_publish_frame(self, flag=True):
        self.publish_frame = flag
        print(f"publish_frame : {self.publish_frame}")

    def get_frame(self):
        return self.last_frame

    def restart(self):
        self.stop()
        time.sleep(2)
        self.start()

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()
        if self.thread:
            self.thread.join()
        logger.info("🛑 IMX219 카메라 중지됨")

    def set_resolution(self, width: int, height: int):
        # nvarguscamerasrc 파이프라인은 런타임 해상도 변경을 지원하지 않으므로
        # 파이프라인을 새 해상도로 재구성해서 재시작한다.
        self.width = width
        self.height = height
        if self.running:
            self.restart()
        logger.info(f'IMX219 해상도를 {self.width}x{self.height}로 변경(재시작)')
        return int(self.width), int(self.height)

    def set_fps(self, fps: int):
        self.fps = fps
        if self.running:
            self.restart()
        logger.info(f'IMX219 fps를 {self.fps}로 변경(재시작)')
        return int(self.fps)

    def save_frame(self, file_path, file_name):
        if self.last_frame is None:
            logger.warning("🚨 저장할 프레임이 없습니다!")
            return
        mcap_logger = get_global_mcap_logger("DEFAULT")
        if mcap_logger and mcap_logger.is_running:
            cam_topic_name = f"/sensors/rgb_camera/{self.camera_name}"
            log_camera = mcap_logger.get_channel_logger(cam_topic_name)
            log_camera(cv2.cvtColor(self.last_frame, cv2.COLOR_BGR2RGB))
