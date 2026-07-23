### ROBOT
# Defines robot identity, type and classification

ROBOT_INFO = {
    "id": "camping_test_01",
    "robot_client_id": "camping_test_01",
    "model": "camping_mate",
    "category": "mobile",
    "api_key": "31870cb7bc4e2a064b4eb9d649fba210"
}

# Camera settings: device index and resolution
VIDEO_INFO = {
    "front_cam": {"source": 0, "width": 640, "height": 480},
    "rear": {"source": 2, "width": 640, "height": 480},
}

# ✅ 후방카메라: by-id → 실제 /dev/videoX 인덱스로 자동 변환
import os
import re

REAR_BYID = "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_0EEEF5BF-video-index0"

def rear_index_from_byid():
    dev = os.path.realpath(REAR_BYID)  # 예: /dev/video2
    m = re.search(r"/dev/video(\d+)$", dev)
    return int(m.group(1)) if m else VIDEO_INFO["rear"]["source"]

# 실행 시점에 rear source를 최신 인덱스로 갱신
VIDEO_INFO["rear"]["source"] = rear_index_from_byid()

# Audio I/O devices
AUDIO_INFO = {
    # "audio": {"input": "NM-CSP01", "output": "NM-CSP01"},
    "audio": {"input": "default", "output": "default"},
}
