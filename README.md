# Crazyflie Real-World Green Ball Tracking with ESP32-CAM + OpenCV

[![Watch the demo](https://img.youtube.com/vi/9ssjhqC-uFs/0.jpg)](https://www.youtube.com/watch?v=9ssjhqC-uFs)


## 📌 Overview

This project demonstrates a real-world implementation of green object tracking using a Crazyflie 2.1 drone and an ESP32-CAM module. We capture video using the ESP32-CAM, process it in real-time with OpenCV on a computer, and send flight commands to the drone via the Crazyradio PA USB dongle. This setup mimics vision-based autonomous tracking without heavy onboard computation.

---

## 🧠 How It Works

1. **ESP32-CAM** streams live video over Wi-Fi using a lightweight web server (`WebServerCamera.ino`).
2. **Python/OpenCV Script** (`CrazyFlie-Opencv-Following.py`) receives the video stream, identifies the green object (e.g., a ball), and calculates the required motion.
3. **Crazyflie 2.1 Drone** receives movement commands wirelessly via the Crazyradio dongle and follows the object by adjusting yaw, forward motion, and altitude.

---

## 📁 Files Included

- `WebServerCamera.ino`: Arduino code for ESP32-CAM to start a video stream on `http://<ip>:81/stream`.
- `CrazyFlie-Opencv-Following.py`: Python script that connects to both the ESP32-CAM stream and the Crazyflie. It performs object detection and sends motion commands based on the target's position and size.
- `CrazyFlieAI.py`: (Optional) Webots simulation script for developing and testing the same tracking logic in a virtual environment.

---

## 🛠️ Requirements

### Hardware
- Crazyflie 2.1 drone
- Crazyradio PA (USB)
- ESP32-CAM module
- A Wi-Fi router that both the ESP32 and computer can connect to
- Computer with Python 3 and OpenCV installed

### Software
- Python 3.x
- `opencv-python`
- `numpy`
- `cflib` (Bitcraze Python library)

Install Python dependencies:

```bash
pip install opencv-python numpy cflib
```
To correctly install cflib refer to : https://github.com/bitcraze/crazyflie-lib-python

Other dependencies needed:
```bash
time, threading, logging, signal, sys
```
### Setup Guide
- Upload WebServerCamera.ino to your ESP32-CAM using Arduino IDE, and change name & password on credentials, to get your address, use Serial Monitor.
- In CrazyFlie-Opencv-Following.py, set:
```bash
STREAM_URL = "http://<esp32-ip>:81/stream
```
Replace <esp32-ip> with your address provided from your WebServerCamera.ino
- Run your python controller after chaning your URI:
```bash
URI = "radio://0/80/2M/E7E7E7E7E7"
```
Run :
```bash
python CrazyFlie-Opencv-Following.py

```
