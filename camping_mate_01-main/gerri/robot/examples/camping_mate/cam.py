import cv2

cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow("frame",frame)
        cv2.waitKey(33)

