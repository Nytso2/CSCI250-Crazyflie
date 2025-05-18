import cv2
import numpy as np
import time

# ——— Configuration ———
STREAM_URL = "http://192.168.142.85:81/stream"

# Neon green detection — bright and saturated
JUST_GREEN_LOWER_HSV = np.array([45, 100, 100])
JUST_GREEN_UPPER_HSV = np.array([75, 255, 255])


TARGET_DIAM      = 100
K_DIST           = 0.02
MAX_FORWARD_SPD  = 1.0
START_DELAY      = 4.0
MIN_CONTOUR_AREA = 500
BLUR_KERNEL      = (7, 7)
MORPH_KERNEL     = np.ones((5, 5), np.uint8)
# ————————————

cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)
if not cap.isOpened():
    print(f"Error: couldn’t open stream at {STREAM_URL}")
    exit(1)

start_time = time.time()
prev_time  = start_time

while True:
    now = time.time()
    dt  = now - prev_time
    prev_time = now

    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    h, w = frame.shape[:2]

    # Smooth image to reduce noise
    blurred = cv2.GaussianBlur(frame, BLUR_KERNEL, 0)

    # Convert to HSV and apply mask
    hsv  = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, JUST_GREEN_LOWER_HSV, JUST_GREEN_UPPER_HSV)

    # Morphological cleanup
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)

    # Find contours
    cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) >= MIN_CONTOUR_AREA]

    if cnts:
        largest = max(cnts, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

            if now - start_time > START_DELAY:
                xDiff = cX - w // 2
                yDiff = cY - h // 2
                # yaw and height control logic here...

            (cx, cy), radius = cv2.minEnclosingCircle(largest)
            diam = 2 * radius

            if diam < TARGET_DIAM and now - start_time > START_DELAY:
                forward_des = min((TARGET_DIAM - diam) * K_DIST, MAX_FORWARD_SPD)

            cv2.circle(frame, (int(cx), int(cy)), int(radius), (255, 0, 0), 2)
            cv2.putText(frame, f"D={int(diam)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Crosshairs
    cv2.line(frame, (0, h // 2), (w, h // 2), (0, 255, 0), 1)
    cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)

    cv2.imshow("Neon Green Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
