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

# Initialize the low-level drivers (don't list the debug drivers)
cflib.crtp.init_drivers()

# Flag to indicate when to stop the control loop
stop_flag = False

def keyboard_control(stdscr, mc):
    """
    Capture keyboard input and send movement commands to the Crazyflie.
    """
    global stop_flag
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr(0, 0, "Use arrow keys to move. Press 'q' to quit.")
    stdscr.refresh()

    while not stop_flag:
        key = stdscr.getch()
        if key == curses.KEY_UP:
            mc.forward(0.1)
        elif key == curses.KEY_DOWN:
            mc.back(0.1)
        elif key == curses.KEY_LEFT:
            mc.left(0.1)
        elif key == curses.KEY_RIGHT:
            mc.right(0.1)
        elif key == ord('q'):
            stop_flag = True
        time.sleep(0.1)

def main():
    global stop_flag
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        with MotionCommander(scf) as mc:
            # Start the keyboard control in a separate thread
            keyboard_thread = threading.Thread(target=curses.wrapper, args=(keyboard_control, mc))
            keyboard_thread.start()

            # Keep the main thread alive until stop_flag is set
            while not stop_flag:
                time.sleep(0.1)

if __name__ == '__main__':
    main()
