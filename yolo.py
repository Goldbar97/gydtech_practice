import cv2
from ultralytics import YOLO
import numpy as np

model = YOLO('yolov8n-seg.pt')

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, verbose=False)

    for result in results:
        if result.masks is not None:
            for mask, cls in zip(result.masks.xy, result.boxes.cls):
                class_name = model.names[int(cls)]

                if class_name == 'person':
                    points = np.array(mask, dtype=np.int32)
                    cv2.polylines(frame, [points], isClosed=True, color=(0, 255, 0), thickness=2)

    cv2.imshow("YOLO Person Contour", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()