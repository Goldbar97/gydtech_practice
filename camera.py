import cv2
import mediapipe as mp
import numpy as np

# ---------- mediapipe 설정 ----------
mp_hands = mp.solutions.hands
mp_face_mesh = mp.solutions.face_mesh
mp_draw = mp.solutions.drawing_utils

hand_detector = mp_hands.Hands(min_detection_confidence=0.7, max_num_hands=1)
face_detector = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, max_num_faces=1)

# 손 윤곽 순서 (손가락 사이 홈 포함)
HAND_OUTLINE_IDX = [0, 1, 2, 3, 4, 2,
                     8, 7, 6, 5,
                     12, 11, 10, 9,
                     16, 15, 14, 13,
                     20, 19, 18, 17,
                     0]

# 얼굴 바깥 테두리(윤곽) 관절 번호
FACE_OVAL_IDX = [10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288,
                  397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136,
                  172, 58, 132, 93, 234, 127, 162, 21, 54, 103, 67, 109, 10]

# ---------- 메인 루프 ----------
cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # ---- 손 인식 ----
        hand_results = hand_detector.process(rgb)

        if hand_results.multi_hand_landmarks:
            for hand_landmarks in hand_results.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                points = []
                for lm in hand_landmarks.landmark:
                    x, y = int(lm.x * w), int(lm.y * h)
                    points.append([x, y])

                hand_outline = np.array([points[i] for i in HAND_OUTLINE_IDX], dtype=np.int32)
                cv2.polylines(frame, [hand_outline], isClosed=True, color=(0, 255, 255), thickness=2)

        # ---- 얼굴 인식 ----
        face_results = face_detector.process(rgb)

        if face_results.multi_face_landmarks:
            for face_landmarks in face_results.multi_face_landmarks:
                face_points = []
                for lm in face_landmarks.landmark:
                    x, y = int(lm.x * w), int(lm.y * h)
                    face_points.append([x, y])

                face_outline = np.array([face_points[i] for i in FACE_OVAL_IDX], dtype=np.int32)
                cv2.polylines(frame, [face_outline], isClosed=True, color=(255, 0, 255), thickness=2)

        # ---- 화면 표시 ----
        cv2.imshow("Hand & Face Contour", frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    cap.release()
    cv2.destroyAllWindows()