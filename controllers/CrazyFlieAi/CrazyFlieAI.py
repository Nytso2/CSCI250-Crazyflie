"""
File name: CrazyFlieAI.py

Authors: 

Description: 

"""


from controller import Robot, Keyboard
import cv2
import numpy as np
from math import cos, sin
from pid_controller import pid_velocity_fixed_height_controller
from time import time

FLYING_ATTITUDE = 0.8

# new tuning params
TARGET_DIAM   = 75    # desired diameter in pixels
K_DIST        = 0.007  # m/s per pixel deficit
MAX_FORWARD_SPEED = 2.0    # clamp at 1 m/s

start = time()
if __name__ == '__main__':
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Initialize motors
    motors = []
    for name, sign in zip(["m1_motor","m2_motor","m3_motor","m4_motor"], [-1,1,-1,1]):
        m = robot.getDevice(name)
        m.setPosition(float('inf'))
        m.setVelocity(sign)
        motors.append(m)

    # Initialize sensors
    imu    = robot.getDevice("inertial_unit"); imu.enable(timestep)
    gps    = robot.getDevice("gps");           gps.enable(timestep)
    gyro   = robot.getDevice("gyro");          gyro.enable(timestep)
    camera = robot.getDevice("camera");        camera.enable(timestep)
    for rng in ("range_front","range_left","range_back","range_right"):
        s = robot.getDevice(rng); s.enable(timestep)

    # Keyboard
    keyboard = Keyboard(); keyboard.enable(timestep)

    # State
    past_x_global = past_y_global = past_time = 0
    first_time = True
    height_desired = FLYING_ATTITUDE
    PID = pid_velocity_fixed_height_controller()

    print("\n====== Controls for ball =======")
    print("Use ↑ ↓ ← → or W/S for movement")
    print("Q/E to rotate yaw")
    print("ESC to stop\n")

    while robot.step(timestep) != -1:
        t = robot.getTime()
        dt = t - past_time
        past_time = t

        # First‐time GPS init
        if first_time:
            past_x_global, past_y_global = gps.getValues()[0:2]
            first_time = False

        # Read sensors
        roll, pitch, yaw = imu.getRollPitchYaw()
        yaw_rate = gyro.getValues()[2]
        xg, yg, altitude = gps.getValues()
        v_xg = (xg - past_x_global) / dt
        v_yg = (yg - past_y_global) / dt
        past_x_global, past_y_global = xg, yg

        cos_y = cos(yaw); sin_y = sin(yaw)
        v_x =  v_xg * cos_y + v_yg * sin_y
        v_y = -v_xg * sin_y + v_yg * cos_y

        # Desired motion defaults
        forward_desired = 0.0
        sideways_desired = 0.0
        yaw_desired = 0.0
        height_diff_desired = 0.0

        # Keyboard overrides
        key = keyboard.getKey()
        # while key > 0:
        #     if key == Keyboard.UP:    forward_desired += 0.5
        #     elif key == Keyboard.DOWN:forward_desired -= 0.5
        #     elif key == Keyboard.RIGHT: sideways_desired -= 0.5
        #     elif key == Keyboard.LEFT:  sideways_desired += 0.5
        #     elif key == ord('Q'):      yaw_desired =  +1
        #     elif key == ord('E'):      yaw_desired =  -1
        #     elif key == ord('W'):      height_diff_desired =  0.1
        #     elif key == ord('S'):      height_diff_desired = -0.1
        #     key = keyboard.getKey()
        height_desired += height_diff_desired * dt

        # Capture image
        
        w = camera.getWidth(); h = camera.getHeight()
        data = camera.getImage()
        img = np.frombuffer(data, np.uint8).reshape((h, w, 4))
        img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        # === GREEN DETECTION & DIAMETER-BASED FORWARD MOTION ===
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([40,70,70]), np.array([80,255,255]))
        cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if cnts:
            largest = max(cnts, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] != 0:
                # center point
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])
                cv2.circle(img_bgr, (cX,cY), 5, (0,0,255), -1)

                # x/y offset→ yaw/height as before
                if time() - start > 4:
                    xDiff = cX - w//2
                    yDiff = cY - h//2
                    height_diff_desired -= yDiff * 0.002
                    yaw_desired         -= xDiff * 0.02
                    # height_desired      -= yDiff * 0.0001
                    # height_desired     += height_diff_desired * 0.001
                    height_desired     += height_diff_desired * dt
    
                # compute circle diameter
                (cx,cy), radius = cv2.minEnclosingCircle(largest)
                diam = 2 * radius

                #move forward until diam >= TARGET_DIAM
                if diam < TARGET_DIAM and time() - start > 4:
                    forward_desired = (TARGET_DIAM - diam) * K_DIST
                    #upper bound so we don't go too fast:
                    forward_desired = min(forward_desired, MAX_FORWARD_SPEED)
                else:
                    forward_desired = 0.0

                cv2.circle(img_bgr, (int(cx),int(cy)), int(radius), (255,0,0), 2)
                cv2.putText(img_bgr, f"D={int(diam)}", (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                
                cv2.line(img_bgr, (0, w // 2), (w, w // 2), (0, 255, 0), 2)
                cv2.line(img_bgr, (w // 2, 0), (w // 2, w), (255, 0, 0), 2)


        # show image
        cv2.imshow("Green Ball Tracking", img_bgr)
        if cv2.waitKey(1) == 27: # press esc if you want to
            break

        # PID → motors
        motor_pw = PID.pid(dt,
                           forward_desired,
                           sideways_desired,
                           yaw_desired,
                           height_desired,
                           roll, pitch, yaw_rate,
                           altitude, v_x, v_y)

        motors[0].setVelocity(-motor_pw[0])
        motors[1].setVelocity( motor_pw[1])
        motors[2].setVelocity(-motor_pw[2])
        motors[3].setVelocity( motor_pw[3])

    cv2.destroyAllWindows()
