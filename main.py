import logging
import time
import threading
import curses

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# URI to the Crazyflie to connect to
URI = 'radio://0/80/2M/E7E7E7E7E7'

# Set up logging
logging.basicConfig(level=logging.ERROR)

# Initialize the low-level drivers
cflib.crtp.init_drivers()

# Flag to indicate when to stop the control loop
stop_flag = False
STEP = 0.1  # movement increment (m or m/s)


def keyboard_control(stdscr, mc):
    """
    Capture keyboard input and send continuous movement commands using start/stop methods.
    Arrows = move in XY plane; W/S = up/down; Q = quit & land
    """
    global stop_flag
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr(0, 0, "Arrows: move, W/S: up/down, Q: quit & land")
    stdscr.refresh()

    last_dir = None
    while not stop_flag:
        key = stdscr.getch()
        # Horizontal movement
        if key == curses.KEY_UP:
            if last_dir != 'forward':
                mc.start_forward(STEP)
                last_dir = 'forward'
        elif key == curses.KEY_DOWN:
            if last_dir != 'back':
                mc.start_back(STEP)
                last_dir = 'back'
        elif key == curses.KEY_LEFT:
            if last_dir != 'left':
                mc.start_left(STEP)
                last_dir = 'left'
        elif key == curses.KEY_RIGHT:
            if last_dir != 'right':
                mc.start_right(STEP)
                mc.turn_right(STEP)
                last_dir = 'right'
        # Vertical movement (momentary)
        elif key in (ord('w'), ord('W')):
            mc.up(STEP)
        elif key in (ord('s'), ord('S')):
            mc.down(STEP)
        # Quit
        elif key in (ord('q'), ord('Q')):
            stop_flag = True
        # No arrow key pressed -> stop horizontal motion
        else:
            if last_dir is not None:
                mc.stop()
                last_dir = None

        time.sleep(0.5)


def main():
    global stop_flag
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        with MotionCommander(scf) as mc:
            # Safe takeoff
            try:
                mc.take_off(0.5)
                time.sleep(2)
            except Exception:
                pass

            # Start keyboard control in a separate daemon thread
            kb_thread = threading.Thread(
                target=lambda: curses.wrapper(lambda s: keyboard_control(s, mc)),
                daemon=True
            )
            kb_thread.start()

            # Keep alive until user quits
            while not stop_flag:
                time.sleep(0.1)

            # Land safely
            try:
                mc.land()
                time.sleep(2)
            except Exception:
                pass


if __name__ == '__main__':
    main()

