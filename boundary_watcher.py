"""
boundary_watcher.py
-------------------
Standalone boundary detection module.

Usage — import and call start() to launch the watcher as a background daemon
thread. Pass in the shared stop_event so the caller can shut it down cleanly:

    import threading
    from boundary_watcher import start as start_boundary

    stop_event = threading.Event()
    start_boundary(bot, stop_event)
    # ... rest of your program ...
    stop_event.set()   # stops the watcher thread
"""

import time
import threading
from lib.McBetter_Wheel_Sports import move_backward, rotate_left, rotate_right, stop

# ── Tuning constants ──────────────────────────────────────────────────────────
SAMPLE_COUNT  = 20    # sensor readings collected per cycle
SLEEP_CLEAR   = 0.05  # seconds between polls when no boundary detected
SPEED         = 10


def _read_sensors(bot):
    """Return (L1, L2, R1, R2) — 0 means black (boundary), 1 means clear."""
    track_data = bot.read_data_array(0x0a, 1)
    track = int(track_data[0])
    x1 = (track >> 3) & 0x01
    x2 = (track >> 2) & 0x01
    x3 = (track >> 1) & 0x01
    x4 =  track       & 0x01
    return x2, x1, x3, x4  # L1, L2, R1, R2


def _watcher_loop(bot, stop_event):
    """
    Main loop — polls sensors and reacts to boundary detections.

    Priority logic (high to low):
      1. L2 + R1 both triggered  → boundary straight ahead  → back + right turn
      2. L1 or L2 triggered      → boundary on the left     → back + right turn
      3. R1 or R2 triggered      → boundary on the right    → back + left turn
    """
    print("[boundary] watcher started")
    while not stop_event.is_set():
        readings = [_read_sensors(bot) for _ in range(SAMPLE_COUNT)]

        # A sensor is triggered if it fired in *any* sample in the batch
        L1 = 0 if any(r[0] == 0 for r in readings) else 1
        L2 = 0 if any(r[1] == 0 for r in readings) else 1
        R1 = 0 if any(r[2] == 0 for r in readings) else 1
        R2 = 0 if any(r[3] == 0 for r in readings) else 1

        if L1 == 0 or L2 == 0 or R1 == 0 or R2 == 0:
            print(f"[boundary] triggered — L1={L1} L2={L2} R1={R1} R2={R2}")
            stop()

            if L2 == 0 and R1 == 0:          # straight ahead
                print("[boundary] straight ahead — backing up and turning right")
                move_backward(SPEED)
                time.sleep(1.0)
                stop()
                rotate_right(SPEED)
                time.sleep(1.0)
                stop()

            elif L1 == 0 or L2 == 0:         # left boundary
                print("[boundary] left — backing up and turning right")
                move_backward(SPEED)
                time.sleep(0.5)
                stop()
                rotate_right(SPEED)
                time.sleep(0.5)
                stop()

            elif R1 == 0 or R2 == 0:         # right boundary
                print("[boundary] right — backing up and turning left")
                move_backward(SPEED)
                time.sleep(0.5)
                stop()
                rotate_left(SPEED)
                time.sleep(0.5)
                stop()

            time.sleep(0.3)   # brief cooldown before re-checking
        else:
            time.sleep(SLEEP_CLEAR)

    print("[boundary] watcher stopped")


def start(bot, stop_event):
    """
    Launch the boundary watcher as a background daemon thread.

    Args:
        bot:        Raspbot instance (already initialised by the caller)
        stop_event: threading.Event — set it to stop the watcher thread

    Returns:
        The running Thread object (daemon=True, so it exits with the process).
    """
    thread = threading.Thread(
        target=_watcher_loop,
        args=(bot, stop_event),
        daemon=True,
        name="boundary-watcher",
    )
    thread.start()
    return thread
