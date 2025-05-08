"""
FileName: ball_supervisor.py
Author : Luis Coronel
Description: Managable ball with ↑ ↓ ← → or W/S for movement; Q/E to rotate yaw
"""

from controller import Supervisor, Keyboard

if __name__ == "__main__":

    supervisor = Supervisor()
    keyboard = Keyboard()
    keyboard.enable(int(supervisor.getBasicTimeStep()))
    timestep = int(supervisor.getBasicTimeStep())

    # Get the Ball node
    ball = supervisor.getFromDef("Ball")
    if ball is None:
        print("Ball node not found! Check DEF name.")
        exit()

    position_field = ball.getField("translation")
    position = position_field.getSFVec3f()

    speed = 0.02  # Adjust speed that the ball moves at

    while supervisor.step(timestep) != -1:
        key = keyboard.getKey()

        # Check which key is pressed
        if key == Keyboard.UP:
            position[1] += speed
        elif key == Keyboard.DOWN:
            position[1] -= speed
        elif key == Keyboard.LEFT:
            position[0] -= speed
        elif key == Keyboard.RIGHT:
            position[0] += speed
        elif key == ord('W'):
            position[2] += speed
        elif key == ord('S'):
            position[2] -= speed

        position_field.setSFVec3f(position)
