#!/usr/bin/python3
# -*- coding: UTF-8 -*-
from raspbot.Raspbot_Lib import Raspbot
import time, math, socket, threading
import cv2
import os
from datetime import datetime
from lib.McBetter_Wheel_Sports import (
    move_forward,
    move_backward,
    move_left,
    move_right,
    rotate_left,
    rotate_right,
    stop_robot,
)

SAVE_PATH = "/home/lab-tuesday/raspbot_programming/IMAGE/"
os.makedirs(SAVE_PATH, exist_ok=True)

# ── Config ────────────────────────────────────────────────────────────────────

HOST = "0.0.0.0"
PORT = 5005
SPEED = 50        # 0–255

CAM_MOVE_DELAY  = 0.02 # seconds between each 1° step (lower = faster)
CAM_PAN_MIN     = 45
CAM_PAN_MAX     = 135
CAM_TILT_MIN    = 0
CAM_TILT_MAX    = 90
CAM_PAN_CENTER  = 90
CAM_TILT_CENTER = 0

# ── Bot & Camera State ────────────────────────────────────────────────────────

bot = Raspbot()

cam_pan  = CAM_PAN_CENTER
cam_tilt = CAM_TILT_CENTER

_pan_thread  = None
_tilt_thread = None
_pan_active  = False
_tilt_active = False


# ── Smooth Servo Movement ─────────────────────────────────────────────────────

def _smooth_move(axis, target, step_delay):
    global cam_pan, cam_tilt, _pan_active, _tilt_active

    if axis == "pan":
        while _pan_active:
            if cam_pan < target:
                cam_pan = min(cam_pan + 1, target)
            elif cam_pan > target:
                cam_pan = max(cam_pan - 1, target)
            bot.Ctrl_Servo(1, cam_pan)
            if cam_pan == target:
                break
            time.sleep(step_delay)
        _pan_active = False

    elif axis == "tilt":
        while _tilt_active:
            if cam_tilt < target:
                cam_tilt = min(cam_tilt + 1, target)
            elif cam_tilt > target:
                cam_tilt = max(cam_tilt - 1, target)
            bot.Ctrl_Servo(2, cam_tilt)
            if cam_tilt == target:
                break
            time.sleep(step_delay)
        _tilt_active = False

def _start_pan(target):
    global _pan_thread, _pan_active
    _pan_active = False
    if _pan_thread and _pan_thread.is_alive():
        _pan_thread.join(timeout=0.1)
    _pan_active = True
    _pan_thread = threading.Thread(
        target=_smooth_move,
        args=("pan", target, CAM_MOVE_DELAY),
        daemon=True
    )
    _pan_thread.start()

def _start_tilt(target):
    global _tilt_thread, _tilt_active
    _tilt_active = False
    if _tilt_thread and _tilt_thread.is_alive():
        _tilt_thread.join(timeout=0.1)
    _tilt_active = True
    _tilt_thread = threading.Thread(
        target=_smooth_move,
        args=("tilt", target, CAM_MOVE_DELAY),
        daemon=True
    )
    _tilt_thread.start()

# -- Camera Continuous Move ----------------------------------------------------

_pan_active  = False
_tilt_active = False
_pan_thread  = None
_tilt_thread = None

def _continuous_pan(direction):
    """direction: +1 = left, -1 = right"""
    global cam_pan, _pan_active
    while _pan_active:
        cam_pan = max(CAM_PAN_MIN, min(CAM_PAN_MAX, cam_pan + direction))
        bot.Ctrl_Servo(1, cam_pan)
        time.sleep(CAM_MOVE_DELAY)

def _continuous_tilt(direction):
    """direction: +1 = up, -1 = down"""
    global cam_tilt, _tilt_active
    while _tilt_active:
        cam_tilt = max(CAM_TILT_MIN, min(CAM_TILT_MAX, cam_tilt + direction))
        bot.Ctrl_Servo(2, cam_tilt)
        time.sleep(CAM_MOVE_DELAY)

def _start_pan(direction):
    global _pan_thread, _pan_active
    _pan_active = False
    if _pan_thread and _pan_thread.is_alive():
        _pan_thread.join(timeout=0.1)
    _pan_active = True
    _pan_thread = threading.Thread(target=_continuous_pan, args=(direction,), daemon=True)
    _pan_thread.start()

def _start_tilt(direction):
    global _tilt_thread, _tilt_active
    _tilt_active = False
    if _tilt_thread and _tilt_thread.is_alive():
        _tilt_thread.join(timeout=0.1)
    _tilt_active = True
    _tilt_thread = threading.Thread(target=_continuous_tilt, args=(direction,), daemon=True)
    _tilt_thread.start()

def cam_up():
    _start_tilt(+1)

def cam_down():
    _start_tilt(-1)

def cam_left():
    _start_pan(+1)

def cam_right():
    _start_pan(-1)

def cam_center():
    global cam_pan, cam_tilt
    cam_stop()
    cam_pan  = CAM_PAN_CENTER
    cam_tilt = CAM_TILT_CENTER
    bot.Ctrl_Servo(1, cam_pan)
    bot.Ctrl_Servo(2, cam_tilt)

def cam_stop():
    global _pan_active, _tilt_active
    _pan_active  = False
    _tilt_active = False
# ── Command Map ───────────────────────────────────────────────────────────────

COMMANDS = {
    "FORWARD":      lambda: move_forward(SPEED),
    "BACKWARD":     lambda: move_backward(SPEED),
    "LEFT":         lambda: move_left(SPEED),
    "RIGHT":        lambda: move_right(SPEED),
    "ROTATE_LEFT":  lambda: rotate_left(SPEED),
    "ROTATE_RIGHT": lambda: rotate_right(SPEED),
    "STOP":         stop_robot,
    "CAM_UP":       cam_up,
    "CAM_DOWN":     cam_down,
    "CAM_LEFT":     cam_left,
    "CAM_RIGHT":    cam_right,
    "CAM_CENTER":   cam_center,
    "CAM_STOP":     cam_stop,
}

# -- Camera Thread -------------------------------------------------------------

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # prevents frame buildup lag

_cam_running = True

capture_counter=0

def _camera_loop():
    global capture_counter
    while _cam_running:
        ret, frame = cap.read()
        if not ret:
            continue
        cv2.imshow("Camera", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            filename = f"capture_{capture_counter}.jpg"
            cv2.imwrite(os.path.join(SAVE_PATbH, filename), gray)
            print(f"Image saved: {filename}")
            capture_counter += 1
        elif key == ord('q'):
            break

cam_thread = threading.Thread(target=_camera_loop, daemon=True)
cam_thread.start()

# ── Server ────────────────────────────────────────────────────────────────────

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)

print(f"Waiting for connection on port {PORT}...")

conn, addr = server.accept()
print(f"Connected to controller at {addr}\n")

# center servos on startup
bot.Ctrl_Servo(1, CAM_PAN_CENTER)
bot.Ctrl_Servo(2, CAM_TILT_CENTER)

buffer = ""

try:
    while True:
        data = conn.recv(1024).decode()
        if not data:
            print("Connection lost.")
            break

        buffer += data
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            command = line.strip().upper()
            if command in COMMANDS:
                print(f"Executing: {command}")
                COMMANDS[command]()
            else:
                print(f"Unknown command: {command}")


except KeyboardInterrupt:
    print("\nShutting down.")

finally:
    _cam_running = False
    cam_stop()
    stop_robot()
    conn.close()
    server.close()
    cap.release()
    cv2.destroyAllWindows()
