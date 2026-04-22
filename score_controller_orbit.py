"""
score_controller_orbit.py
-------------------------
Scoring logic with a simple, explicit control loop.

Process:
  1. FIND BALL
       Spin right until ball appears in frame.

  2. ALIGN WITH POSTS (orbit phase)
       Centre ball by strafing.
       Strafe left, turn right, rotate to re-centre ball.
       Check if ball is between posts — if not, repeat.

  3. CONFIRM AND FINE-TUNE (final phase, once between posts)
       Rotate to centre ball → check posts → strafe to centre →
       check posts → rotate to centre → confirm both → score.

  4. STRIKE
       Charge forward at the ball.

Key rule: the robot is always fully stopped before any image is processed.
No detections happen while the robot is moving.
"""

from lib.McBetter_Wheel_Sports import *
from image_detection.detection import *
from raspbot.Raspbot_Lib import Raspbot
import boundary_watcher
import cv2
import time
import threading
import os

os.environ.setdefault("QT_QPA_PLATFORM", "xcb")   # use X11 backend — avoids Wayland plugin warning

# ── Hardware init ─────────────────────────────────────────────────────────────
bot = Raspbot()

CAM_PAN_CENTER  = 90    # degrees — straight ahead
CAM_TILT_CENTER = 25

bot.Ctrl_Servo(1, CAM_PAN_CENTER)
bot.Ctrl_Servo(2, CAM_TILT_CENTER)

# ── Camera ────────────────────────────────────────────────────────────────────
cam = cv2.VideoCapture(0)

# ── Shared stop event (passed to boundary watcher) ────────────────────────────
stop_event = threading.Event()

# ── Shared camera frame ───────────────────────────────────────────────────────
# The camera feed thread writes here continuously. _capture_gray() reads from
# here so that cam.read() is only ever called from one thread, avoiding
# OpenCV thread-safety issues.
_frame_lock    = threading.Lock()
_latest_frame  = None   # most recent raw BGR frame from the camera


def _camera_feed_thread():
    """
    Runs as a daemon thread for the entire program lifetime.
    Continuously reads frames from the camera, updates _latest_frame for use
    by detection functions, and displays a live window via cv2.imshow.

    The window is titled "Robot Camera Feed" and can be closed with 'q'.
    Pressing 'q' does not stop the robot — it only closes the display window.
    The thread exits cleanly when stop_event is set.
    """
    global _latest_frame
    print("[camera] feed started")

    while not stop_event.is_set():
        ret, frame = cam.read()
        if not ret:
            continue

        with _frame_lock:
            _latest_frame = frame.copy()

        # Display the live feed — label shows the feed is active
        display = frame.copy()
        cv2.putText(display, "Robot Camera Feed  |  q to close",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("Robot Camera Feed", display)

        # cv2.waitKey must be called from the same thread as imshow
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

    cv2.destroyAllWindows()
    print("[camera] feed stopped")

# ══════════════════════════════════════════════════════════════════════════════
# TUNING PARAMETERS — edit these to adjust robot behaviour
# ══════════════════════════════════════════════════════════════════════════════

# — Spin (used when searching for ball and posts) ——————————————————————————————
SPIN_SPEED    = 5      # motor speed for spin bursts (0–100)
SPIN_DURATION = 0.1  # seconds per spin burst — keep short to avoid overshooting

# — Centre strafe (used to centre the ball in frame) ——————————————————————————
CENTRE_SPEED    = 15    # motor speed for centring strafe (0–100)
CENTRE_DURATION = 0.03  # seconds per centre strafe burst

# — Align strafe (used to move ball between posts) ————————————————————————————
ALIGN_SPEED    = 25     # motor speed for alignment strafe (0–100)
ALIGN_DURATION = 0.1    # seconds per alignment strafe burst

# — Strike (scoring charge) ————————————————————————————————————————————————————
STRIKE_SPEED    = 40    # motor speed for strike (0–100)
STRIKE_DURATION = 1.2   # seconds to drive forward during strike

# — Detection thresholds ———————————————————————————————————————————————————————
BALL_CENTRE_THRESH = 0.15   # ball must be within this fraction of centre to count as centred
GOAL_THRESH        = 1.1    # fraction of goal width — 1.0 means anywhere between the posts
RUN_TIME           = 100    # global timeout in seconds for any single phase

LEFT_THRESH  = 0.5 - BALL_CENTRE_THRESH / 2
RIGHT_THRESH = 0.5 + BALL_CENTRE_THRESH / 2

# — Settle delay ———————————————————————————————————————————————————————————————
SETTLE = 0.5  # seconds to wait after stopping before capturing a frame


# ══════════════════════════════════════════════════════════════════════════════
# CAMERA HELPER
# ══════════════════════════════════════════════════════════════════════════════

def _reset_camera():
    """Return the camera to centre (90°) before any forward movement or strike."""
    bot.Ctrl_Servo(1, CAM_PAN_CENTER)
    time.sleep(0.12)


# ══════════════════════════════════════════════════════════════════════════════
# IMAGE DETECTION
# ══════════════════════════════════════════════════════════════════════════════

def _capture_gray():
    """
    Returns the most recent camera frame converted to gray→BGR, or None on failure.
    Reads from _latest_frame written by the camera feed thread, so cam.read()
    is never called from more than one thread simultaneously.
    """
    with _frame_lock:
        if _latest_frame is None:
            return None
        frame = _latest_frame.copy()

    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image


def _check_ball():
    """
    Robot must be fully stopped before calling this.
    Captures a fresh frame and returns ball detection results.

    Returns (ball_in_frame, ball_pos, ball_centered, ball_width).
    Returns False if the camera read fails.
    """
    image = _capture_gray()
    if image is None:
        return False

    num_balls, ball_detections, ball_widths = detect_ball(image)
    ball_in_frame = (num_balls == 1)

    if ball_in_frame:
        ball_pos   = ball_detections[0]
        ball_width = ball_widths[0]
    else:
        ball_pos   = -1
        ball_width = -1

    ball_centered = LEFT_THRESH < ball_pos < RIGHT_THRESH
    return ball_in_frame, ball_pos, ball_centered, ball_width


def _check_ball_and_posts():
    """
    Robot must be fully stopped before calling this.
    Captures a single fresh frame and returns ball and post detections together.
    Using one frame guarantees both positions are consistent with each other.

    Returns (ball_in_frame, ball_pos, ball_centered, post_pos, posts_found, num_posts).
    Returns False if the camera read fails.
    """
    image = _capture_gray()
    if image is None:
        return False

    num_balls, ball_detections, num_posts, post_detections = get_detections(image)

    ball_in_frame = (num_balls == 1)
    ball_pos      = ball_detections[0][0] if ball_in_frame else -1
    ball_centered = LEFT_THRESH < ball_pos < RIGHT_THRESH

    posts_found = (num_posts == 2)
    if posts_found:
        post_pos = (post_detections[0][0], post_detections[1][0])
    elif num_posts == 1:
        post_pos = (post_detections[0][0], -1)
    else:
        post_pos = (-1, -1)

    return ball_in_frame, ball_pos, ball_centered, post_pos, posts_found, num_posts


def _ball_between_posts(ball_pos, post_pos, posts_found):
    """
    Returns True if both posts are visible and the ball is anywhere between
    them within the GOAL_THRESH window (1.0 = full goal width).
    """
    if not posts_found or -1 in post_pos or ball_pos < 0:
        return False

    goal_w = abs(post_pos[0] - post_pos[1])
    goal_c = (post_pos[0] + post_pos[1]) / 2
    left   = goal_c - (GOAL_THRESH * goal_w) / 2
    right  = goal_c + (GOAL_THRESH * goal_w) / 2
    return left <= ball_pos <= right


# ══════════════════════════════════════════════════════════════════════════════
# STEP 1 — FIND BALL
# ══════════════════════════════════════════════════════════════════════════════

def find_ball():
    """
    Stop and check if ball is in frame.
    If not, spin right a small amount, stop, check again.
    Repeat until ball is found or timeout.

    The robot is always fully stopped and settled before each check.
    Returns True when ball is found, False on timeout.
    """
    print("[find] looking for ball")
    start_time = time.time()

    while not stop_event.is_set():
        if time.time() - start_time > RUN_TIME:
            print("[find] timed out")
            return False

        # Robot is stopped — capture and analyze
        result = _check_ball()
        if result is False:
            continue
        ball_in_frame, ball_pos, ball_centered, ball_width = result

        if ball_in_frame:
            print(f"[find] ball found at pos={ball_pos:.2f}")
            return True

        # Ball not found — spin right a small amount, stop, then check again
        print("[find] ball not in frame — spinning right")
        rotate_right(SPIN_SPEED)
        time.sleep(SPIN_DURATION)
        stop()
        time.sleep(SETTLE)   # wait until fully stationary before next check

    return False


# ══════════════════════════════════════════════════════════════════════════════
# STEP 2 — CENTRE BALL IN FRAME
# ══════════════════════════════════════════════════════════════════════════════

def centre_ball():
    """
    Strafe left or right a small amount to centre the ball in frame.
    Stop and re-check after each strafe. Re-runs find_ball() if ball is lost.
    Repeat until ball is centred or timeout.

    ball_pos convention:
      ball_pos < LEFT_THRESH  → ball is left in image  → strafe left
      ball_pos > RIGHT_THRESH → ball is right in image → strafe right

    The robot is always fully stopped and settled before each check.
    Returns True when centred, False on timeout.
    """
    print("[centre] centring ball in frame")
    start_time = time.time()

    while not stop_event.is_set():
        if time.time() - start_time > RUN_TIME:
            stop(); return False

        # Robot is stopped — capture and analyze
        result = _check_ball()
        if result is False:
            continue
        ball_in_frame, ball_pos, ball_centered, ball_width = result

        if not ball_in_frame:
            print("[centre] ball lost — re-finding")
            if not find_ball():
                return False
            continue   # re-check after find_ball returns

        if ball_centered:
            print(f"[centre] ball centred at pos={ball_pos:.2f}")
            return True

        # Strafe toward the ball to centre it
        if 0 <= ball_pos < LEFT_THRESH:
            print(f"[centre] ball left pos={ball_pos:.2f} — strafing left")
            move_left(CENTRE_SPEED)
        elif ball_pos > RIGHT_THRESH:
            print(f"[centre] ball right pos={ball_pos:.2f} — strafing right")
            move_right(CENTRE_SPEED)

        time.sleep(CENTRE_DURATION)
        stop()
        time.sleep(SETTLE)   # wait until fully stationary before next check

    stop()
    return False


def rotate_to_centre_ball():
    """
    Rotates the robot left or right to centre the ball in frame.
    Used during alignment — after a strafe the ball drifts, and rotating
    re-centres it without changing the robot's lateral position relative
    to the goal, which strafing would undo.

    ball_pos convention:
      ball_pos < LEFT_THRESH  → ball is left in image  → rotate left
      ball_pos > RIGHT_THRESH → ball is right in image → rotate right

    The robot is always fully stopped and settled before each check.
    Returns True when centred, False on timeout.
    """
    print("[rotate-centre] rotating to centre ball in frame")
    start_time = time.time()

    while not stop_event.is_set():
        if time.time() - start_time > RUN_TIME:
            stop(); return False

        # Robot is stopped — capture and analyze
        result = _check_ball()
        if result is False:
            continue
        ball_in_frame, ball_pos, ball_centered, ball_width = result

        if not ball_in_frame:
            print("[rotate-centre] ball lost — re-finding")
            if not find_ball():
                return False
            continue

        if ball_centered:
            print(f"[rotate-centre] ball centred at pos={ball_pos:.2f}")
            return True

        # Rotate toward the ball to centre it
        if 0 <= ball_pos < LEFT_THRESH:
            print(f"[rotate-centre] ball left pos={ball_pos:.2f} — rotating left")
            rotate_left(SPIN_SPEED)
        elif ball_pos > RIGHT_THRESH:
            print(f"[rotate-centre] ball right pos={ball_pos:.2f} — rotating right")
            rotate_right(SPIN_SPEED)

        time.sleep(SPIN_DURATION)
        stop()
        time.sleep(SETTLE)

    stop()
    return False

def align_with_posts():
    """
    Aligns the ball between the goalposts in two phases.

    Phase 1 — orbit until ball is between posts:
      a. Find ball
      b. Strafe to centre ball in frame
      c. Strafe left one burst
      d. Turn right one burst
      e. Rotate to centre ball in frame
      f. Check posts — if ball between posts → Phase 2, else repeat from (c)
      If ball is lost at any point → restart from (a)

    Phase 2 — confirm and fine-tune before scoring:
      a. Rotate to centre ball
      b. Check posts — if no longer between posts → back to Phase 1
      c. Strafe to centre ball
      d. Check posts again
      e. Rotate to centre ball
      f. If ball centred and between posts → return True and strike

    Robot is always fully stopped before any image is captured.
    Returns True when ready to strike, False on timeout.
    """
    print("[align] starting alignment")
    start_time = time.time()

    while not stop_event.is_set():
        if time.time() - start_time > RUN_TIME:
            print("[align] timed out")
            stop(); return False

        # ── Phase 1 — orbit until ball is between posts ───────────────────────

        # (a) Find ball
        if not find_ball():
            return False

        # (b) Strafe to centre ball in frame
        if not centre_ball():
            return False

        # (c) Strafe left one burst
        print("[align] orbit — strafing left")
        move_left(ALIGN_SPEED)
        time.sleep(ALIGN_DURATION)
        stop()
        time.sleep(SETTLE)

        # (d) Turn right one burst
        print("[align] orbit — turning right")
        rotate_right(SPIN_SPEED)
        time.sleep(SPIN_DURATION)
        stop()
        time.sleep(SETTLE)

        # (e) Rotate to centre ball in frame
        if not rotate_to_centre_ball():
            continue   # ball lost — restart from (a)

        # (f) Check if ball is between posts
        result = _check_ball_and_posts()
        if result is False:
            continue
        ball_in_frame, ball_pos, ball_centered, post_pos, posts_found, num_posts = result

        if not ball_in_frame:
            print("[align] ball lost after orbit step — restarting")
            continue

        if not _ball_between_posts(ball_pos, post_pos, posts_found):
            if posts_found:
                print(f"[align] ball not between posts (ball={ball_pos:.2f} posts={post_pos}) — continuing orbit")
            else:
                print(f"[align] posts not visible — continuing orbit")
            continue   # back to (a)

        # ── Phase 2 — ball is between posts: confirm and fine-tune ───────────
        print(f"[align] ball between posts — ball={ball_pos:.2f} posts={post_pos}")

        # (a) Rotate to centre ball
        if not rotate_to_centre_ball():
            continue

        # (b) Re-check posts — rotation may have shifted things
        result = _check_ball_and_posts()
        if result is False:
            continue
        ball_in_frame, ball_pos, ball_centered, post_pos, posts_found, num_posts = result

        if not ball_in_frame or not _ball_between_posts(ball_pos, post_pos, posts_found):
            print("[align] lost alignment after rotate — returning to orbit")
            continue   # back to Phase 1

        # (c) Strafe to centre ball in frame
        if not centre_ball():
            continue

        # (d) Re-check posts after strafe
        result = _check_ball_and_posts()
        if result is False:
            continue
        ball_in_frame, ball_pos, ball_centered, post_pos, posts_found, num_posts = result

        if not ball_in_frame or not _ball_between_posts(ball_pos, post_pos, posts_found):
            print("[align] lost alignment after strafe — returning to orbit")
            continue   # back to Phase 1

        # (e) Rotate to centre ball one final time
        if not rotate_to_centre_ball():
            continue

        # (f) Final check — ball centred and between posts
        result = _check_ball_and_posts()
        if result is False:
            continue
        ball_in_frame, ball_pos, ball_centered, post_pos, posts_found, num_posts = result

        if ball_in_frame and ball_centered and _ball_between_posts(ball_pos, post_pos, posts_found):
            print(f"[align] confirmed — ball centred and between posts at pos={ball_pos:.2f}")
            return True

        print("[align] final check failed — returning to orbit")

    stop()
    return False


# ══════════════════════════════════════════════════════════════════════════════
# STEP 4 — STRIKE
# ══════════════════════════════════════════════════════════════════════════════

def strike_ball():
    """
    Reset camera to centre (90°) then charge at STRIKE_SPEED for STRIKE_DURATION.
    Camera must be straight ahead so the robot drives directly at the ball.
    """
    _reset_camera()
    print(f"[strike] charging at speed={STRIKE_SPEED} for {STRIKE_DURATION}s")
    move_forward(STRIKE_SPEED)
    time.sleep(STRIKE_DURATION)
    stop()


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    """
    Full scoring algorithm:
      1. find_ball()        — spin right until ball appears in frame
      2. align_with_posts() — orbit: centre → strafe left → turn right → rotate to centre → check posts
                              confirm: rotate to centre → check → strafe to centre → check → rotate → confirm
      3. strike_ball()      — charge forward
    """
    # Start live camera feed — runs for the entire lifetime of the program
    feed = threading.Thread(target=_camera_feed_thread, daemon=True, name="camera-feed")
    feed.start()

    # Start boundary watcher — runs for the entire lifetime of the program
    boundary_watcher.start(bot, stop_event)

    try:
        # Step 1 — find ball
        if not find_ball():
            print("[main] failed to find ball")
            return

        # Step 2 — align with posts (centres ball as part of final condition)
        if not align_with_posts():
            print("[main] failed to align with posts")
            return

        # Step 4 — strike
        strike_ball()
        print("[main] success!")

    except KeyboardInterrupt:
        print("[main] interrupted")
    finally:
        stop_event.set()
        stop()
        _reset_camera()
        cam.release()
        print("[main] shut down cleanly")


if __name__ == "__main__":
    main()
