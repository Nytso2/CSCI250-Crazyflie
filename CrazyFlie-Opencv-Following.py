"""
File name: CrazyFlieAI.py

Authors: Luis Coronel , Philip Kwan, Saad Ahmed Khan Ghori , Kidus Tegene

Description : This program will get data from a webserver from a ESP-32 Camera, and through the
crazyflie dongle will communicate with a Crazyflie. Uses Opencv for green object detection (can
be changed) to follow by going forward and yawing. It will also open OpenCv GUI to allow user 
to see green ball tracking.
"""

#!/usr/bin/env python2
from os import altsep
import cv2
import numpy as np
import time
import threading
import logging
import signal
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# ——— Configuration —————————————————————————
STREAM_URL       = "http://192.168.142.4:81/stream" # camera http: port 81/stream
URI              = "radio://0/80/2M/E7E7E7E7E7" # use your radio transmition transmition for your 
                                                # crazyflie (use Cfclient to obtain it)

# HSV neon-green
JUST_GREEN_LOWER = np.array([40,100,50])
JUST_GREEN_UPPER = np.array([80,255,255])

TARGET_DIAM      = 100     # px
K_DIST           = 0.02    # m/s per pixel deficit
MAX_FORWARD_SPD  = 0.8     # m/s cap
START_DELAY      = 4.0     # s
MIN_CONTOUR_AREA = 500     # px²

BLUR_KERNEL      = (7,7)
MORPH_KERNEL     = np.ones((5,5), np.uint8)

TAKEOFF_Z        = 0.5     # m
TAKEOFF_THRUST   = 30000   # raw units for manual burst
ALT_P            = 0.5     # P-gain for altitude (m → m/s)
LOG_PERIOD_MS    = 100     # 10 Hz logging
# ——————————————————————————————————————————————

last_z = TAKEOFF_Z
stop_flag = False
frame_lock = threading.Lock()
latest_frame = None


def signal_handler(sig, frame):
    global stop_flag
    stop_flag = True

signal.signal(signal.SIGINT, signal_handler)


def init_logging(cf):
    global last_z
    log_conf = LogConfig(name="cf_logs", period_in_ms=LOG_PERIOD_MS)
    log_conf.add_variable("stateEstimate.z", "float")
    log_conf.add_variable("stabilizer.thrust", "float")
    log_conf.add_variable("pm.vbat", "float")

    def _cb(timestamp, data, _):
        global last_z
        last_z = data["stateEstimate.z"]
        logging.info(f"[{timestamp:6}] z={last_z:.2f} m  thrust={data['stabilizer.thrust']:.0f}  vbat={data['pm.vbat']:.2f} V")

    cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(_cb)
    log_conf.start()


def frame_reader_thread(cap):
    global latest_frame
    while not stop_flag:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                latest_frame = frame
        time.sleep(0.01)


def main():
    global stop_flag, last_z, latest_frame

    cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print(f"Error: can’t open {STREAM_URL}")
        return

    threading.Thread(target=frame_reader_thread, args=(cap,), daemon=True).start()

    logging.basicConfig(level=logging.INFO)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf

        try:
            init_logging(cf)
        except KeyError as e:
            logging.warning(f"Logging init failed: {e}")

        with MotionCommander(scf, default_height=TAKEOFF_Z) as mc:
            time.sleep(4.0)
            
            print("Take off success!")

            time.sleep(1.0)
            start_time = time.time()
            prev_time = start_time

            height_desired = TAKEOFF_Z
            logging.info(">>> ENTERING CONTROL LOOP")


            while not stop_flag:

                status_message = ""
                forward_desired = 0.0
                sideways_desired = 0.0
                yaw_desired = 0.0
                height_diff_desired = 0.0

                now = time.time()
                dt = now - prev_time
                prev_time = now

                with frame_lock:
                    frame = latest_frame.copy() if latest_frame is not None else None

                if frame is None:
                    time.sleep(0.01)
                    continue

                h, w = frame.shape[:2]
                blur = cv2.GaussianBlur(frame, BLUR_KERNEL, 0)
                hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, JUST_GREEN_LOWER, JUST_GREEN_UPPER)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)

                cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cnts = [c for c in cnts if cv2.contourArea(c) >= MIN_CONTOUR_AREA]

                if cnts and now - start_time > START_DELAY:
                    c = max(cnts, key=cv2.contourArea)
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cX = int(M["m10"]/M["m00"])
                        cY = int(M["m01"]/M["m00"])
                        cv2.circle(frame, (cX, cY), 5, (0,0,255), -1)

                        xDiff = cX - w//2
                        yDiff = cY - h//2

                        if abs(xDiff) > 10:
                            yaw_desired = xDiff * 0.05
                            if xDiff > 0:
                                logging.info("Yawing right")
                                status_message += "Yawing right "
                                
                            else:
                                logging.info("Yawing left")
                                status_message += "Yawing left "
                                
                        if abs(yDiff) > 10:
                            height_desired = yDiff * 0.05 
                            if yDiff > 0:
                                logging.info("Moving up")
                                status_message += "Moving up "
                            else:
                                logging.info("Moving down")
                                status_message += "Moving down "                        
                        (cx, cy), r = cv2.minEnclosingCircle(c)
                        diam = 2*r
                        if diam < TARGET_DIAM:
                            forward_desired = (TARGET_DIAM - diam) * K_DIST
                            forward_desired = min(forward_desired, MAX_FORWARD_SPD)
                            

                        cv2.circle(frame, (int(cx),int(cy)), int(r), (255,0,0), 2)
                        cv2.putText(frame, f"D={int(diam)}", (10,30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

                cv2.line(frame, (0,h//2),(w,h//2), (0,255,0),1)
                cv2.line(frame, (w//2,0),(w//2,h), (255,0,0),1)
                cv2.putText(frame, status_message, (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.imshow("Green Follow", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_flag = True
                print(forward_desired, sideways_desired, height_desired, yaw_desired)
                mc.start_linear_motion(forward_desired, sideways_desired, 0, yaw_desired)

            logging.info(">>> LANDING")
            mc.land()
            time.sleep(2.0)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
