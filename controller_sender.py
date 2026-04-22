#!/usr/bin/env python3

import socket
from inputs import get_gamepad

# ── Config ────────────────────────────────────────────────────────────────────

PI_HOST = "10.121.32.241"  # e.g. "192.168.1.42"
PI_PORT = 5005

# ── Connect ───────────────────────────────────────────────────────────────────

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((PI_HOST, PI_PORT))
print(f"Connected to Pi at {PI_HOST}:{PI_PORT}")

print("Xbox Controller Ready.")
print("  D-pad Up/Down/Left/Right -> Move")
print("  RB -> Turn Right  |  LB -> Turn Left")
print(" A -> Cam Down | B -> Cam Right | X -> Cam Left | Y -> Cam Up")
print("  Ctrl-C to quit\n")

def send(command: str):
    sock.sendall((command + "\n").encode())
    print(f"Sent: {command}")

# ── Input Loop ────────────────────────────────────────────────────────────────

while True:
    try:
        events = get_gamepad()
        for event in events:
            ev_type = event.ev_type
            code    = event.code
            value   = event.state

            # D-pad Up / Down
            if ev_type == "Absolute" and code == "ABS_HAT0Y":
                if value == -1:
                    send("FORWARD")
                elif value == 1:
                    send("BACKWARD")
                elif value == 0:
                    send("STOP")

            # D-pad Left / Right
            elif ev_type == "Absolute" and code == "ABS_HAT0X":
                if value == 1:
                    send("RIGHT")
                elif value == -1:
                    send("LEFT")
                elif value == 0:
                    send("STOP")

            # Bumpers
            elif ev_type == "Key" and code == "BTN_TR":   # RB
                if value == 1:
                    send("ROTATE_RIGHT")
                elif value == 0:
                    send("STOP")

            elif ev_type == "Key" and code == "BTN_TL":   # LB
                if value == 1:
                    send("ROTATE_LEFT")
                elif value == 0:
                    send("STOP")

             # Camera controls (ABXY)
            elif ev_type == "Key" and code == "BTN_NORTH":  # Y
                if value == 1:
                    send("CAM_UP")
                elif value == 0:
                    send("CAM_STOP")
            elif ev_type == "Key" and code == "BTN_SOUTH":  # A
                if value == 1:
                    send("CAM_DOWN")
                elif value == 0:
                    send("CAM_STOP")
            elif ev_type == "Key" and code == "BTN_WEST":   # X
                if value == 1:
                    send("CAM_LEFT")
                elif value == 0:
                    send("CAM_STOP")
            elif ev_type == "Key" and code == "BTN_EAST":   # B
                if value == 1:
                    send("CAM_RIGHT")
                elif value == 0:
                    send("CAM_STOP")
            # camera pitch and roll# Menu/Start -> Center camera
            elif ev_type == "Key" and code == "BTN_START":
                if value == 1:
                    send("CAM_CENTER")

            


    except KeyboardInterrupt:
        send("CAM_STOP")
        send("STOP")
        print("\nExiting.")
        sock.close()
        break