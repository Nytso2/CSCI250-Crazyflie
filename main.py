"""
Crazyflie 2.1 Keyboard Control Script (Windows Compatible)

Controls:
- Arrow keys: Forward/Backward/Left/Right
- W/S: Up/Down
- A/D: Yaw left/right
- Esc: Stop and land

Requirements:
- pip install cflib keyboard
- Crazyradio PA USB dongle plugged in
"""

import time
import threading
import cflib.crtp  # Initialize the low-level drivers
from cflib.crazyflie import Crazyflie
import keyboard  # Windows-friendly keyboard input

# Connection URI (change if needed)
URI = 'radio://0/80/2M'

# Movement parameters
THRUST = 37000
ROLL = 0
PITCH = 0
YAW = 0

running = True
cf = Crazyflie()

def connect_callback(link_uri):
    print(f'Connected to {link_uri}')
    # Start sending setpoints every 0.1s
    threading.Thread(target=send_setpoints).start()

def disconnect_callback(link_uri):
    print(f'Disconnected from {link_uri}')
    global running
    running = False

def send_setpoints():
    global running
    while running:
        cf.commander.send_setpoint(ROLL, PITCH, YAW, THRUST)
        time.sleep(0.1)
    # On exit, send zero thrust to land
    for _ in range(20):
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)
    cf.commander.send_stop_setpoint()
    cf.close_link()

def keyboard_listener():
    global ROLL, PITCH, YAW, THRUST, running

    print("Use arrow keys to move, W/S to go up/down, A/D to yaw. Esc to exit.")

    while running:
        ROLL = 0
        PITCH = 0
        YAW = 0

        if keyboard.is_pressed('up'):
            PITCH = 10
        if keyboard.is_pressed('down'):
            PITCH = -10
        if keyboard.is_pressed('left'):
            ROLL = -10
        if keyboard.is_pressed('right'):
            ROLL = 10
        if keyboard.is_pressed('w'):
            THRUST = min(60000, THRUST + 500)
        if keyboard.is_pressed('s'):
            THRUST = max(20000, THRUST - 500)
        if keyboard.is_pressed('a'):
            YAW = -200
        if keyboard.is_pressed('d'):
            YAW = 200
        if keyboard.is_pressed('esc'):
            print("Landing and exiting...")
            running = False

        time.sleep(0.05)

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    print("Scanning interfaces...")
    try:
        cf.open_link(URI)
        cf.connected.add_callback(connect_callback)
        cf.disconnected.add_callback(disconnect_callback)

        keyboard_listener()

    except KeyboardInterrupt:
        print("Interrupted by user")
        running = False
