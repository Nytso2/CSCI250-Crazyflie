from controller import Robot, Keyboard
import cv2
import numpy as np
from math import cos, sin
from pid_controller import pid_velocity_fixed_height_controller
from time import time

FLYING_ATTITUDE = 1

start = time()
if __name__ == '__main__':
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Initialize motors
    motors = []
    for name, sign in zip(["m1_motor", "m2_motor", "m3_motor", "m4_motor"], [-1, 1, -1, 1]):
        motor = robot.getDevice(name)
        motor.setPosition(float('inf'))
        motor.setVelocity(sign)
        motors.append(motor)

    # Initialize sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    range_front = robot.getDevice("range_front")
    range_left = robot.getDevice("range_left")
    range_back = robot.getDevice("range_back")
    range_right = robot.getDevice("range_right")
    for sensor in [range_front, range_left, range_back, range_right]:
        sensor.enable(timestep)

    # Enable keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Variables
    past_x_global = past_y_global = past_time = 0
    first_time = True
    height_desired = FLYING_ATTITUDE
    PID_crazyflie = pid_velocity_fixed_height_controller()

    print("\n====== Controls =======\n")
    print("Use ↑ ↓ ← → or W/S for movement")
    print("Q/E to rotate yaw")
    print("ESC to close OpenCV window and stop controller\n")

    while robot.step(timestep) != -1:
        dt = robot.getTime() - past_time

        if first_time:
            past_x_global = gps.getValues()[0]
            past_y_global = gps.getValues()[1]
            past_time = robot.getTime()
            first_time = False

        # Sensor values
        roll, pitch, yaw = imu.getRollPitchYaw()
        yaw_rate = gyro.getValues()[2]
        x_global, y_global, altitude = gps.getValues()
        v_x_global = (x_global - past_x_global) / dt
        v_y_global = (y_global - past_y_global) / dt
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
        v_y = -v_x_global * sin_yaw + v_y_global * cos_yaw

        # Desired motion
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0

        # Handle keyboard input
        key = keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP:
                forward_desired += 0.5
            elif key == Keyboard.DOWN:
                forward_desired -= 0.5
            elif key == Keyboard.RIGHT:
                sideways_desired -= 0.5
            elif key == Keyboard.LEFT:
                sideways_desired += 0.5
            elif key == ord('Q'):
                yaw_desired = +1
            elif key == ord('E'):
                yaw_desired = -1
            elif key == ord('W'):
                height_diff_desired = 0.1
            elif key == ord('S'):
                height_diff_desired = -0.1
            key = keyboard.getKey()

        height_desired += height_diff_desired * dt

        

        # Display camera
        width = camera.getWidth()
        height = camera.getHeight()
        camera_data = camera.getImage()
        img = np.frombuffer(camera_data, np.uint8).reshape((height, width, 4))
        img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        # === GREEN DETECTION WITH CENTER POINT ===
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 70, 70])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                # Draw a red dot at the center
                cv2.circle(img_bgr, (cX, cY), 5, (0, 0, 255), -1)
                cv2.putText(img_bgr, "Center", (cX - 20, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                
                xDiff = cX - width // 2
                yDiff = cY - height // 2
                print(xDiff, yDiff)
                
                height_diff_desired -= yDiff * 0.002
                height_desired += height_diff_desired * dt
                
                sideways_desired -= xDiff * 0.002
                
                # Draw a horizontal line (from x=50 to x=450, at y=250)
                cv2.line(img_bgr, (0, width // 2), (width, width // 2), (0, 255, 0), 2)
        
                # Draw a vertical line (from y=50 to y=450, at x=250)
                cv2.line(img_bgr, (width // 2, 0), (width // 2, width), (255, 0, 0), 2)
                
        # Show the final image
        cv2.imshow("Green Object Center", img_bgr)
        
        # PID control
        motor_power = PID_crazyflie.pid(dt, forward_desired, sideways_desired,
                                        yaw_desired, height_desired,
                                        roll, pitch, yaw_rate,
                                        altitude, v_x, v_y)

        # Apply motor power
        motors[0].setVelocity(-motor_power[0])
        motors[1].setVelocity(motor_power[1])
        motors[2].setVelocity(-motor_power[2])
        motors[3].setVelocity(motor_power[3])

        # Save state
        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global
        
        # ESC key to stop
        key_val = cv2.waitKey(1)
        if key_val == 27:  # ESC
            cv2.destroyAllWindows()
            break
