"""
Code version: 8.0
TURRET CONTROL – Python (Windows)
==================================
Requirements: pip install opencv-python pyserial pynput numpy

KEYS (work globally – camera window does not need to be focused):
  Arrow LEFT / RIGHT  → X axis  (hold = move, release = stop)
  Arrow UP   / DOWN   → Y axis  (hold = move, release = stop)
  X  – toggle X motor on/off
  Y  – toggle Y motor on/off
  A  – switch MANUAL / AUTO-AIM mode
  S  – servo toggle (0° / 160°)
  L  – laser toggle
  Q  – quit
"""

import cv2
import numpy as np
import serial
import time
import threading
from pynput import keyboard

# ============================================================
#  CONFIGURATION  ← edit these values
# ============================================================
PORT_COM  = 'COM8'   # e.g. 'COM3', 'COM7' – check Device Manager
BAUD_RATE = 115200
CAMERA_ID = 0        # 0 = built-in webcam, 1 = external USB camera

CAM_W, CAM_H = 1024, 600   # WSVGA – camera resolution before rotation

# After 90° rotation the frame becomes 600×1024 (w×h).
# We crop the center 600×600 px – no scaling.
CROP    = 600
CENTER_X = CROP // 2   # 300
CENTER_Y = CROP // 2   # 300

# HSV range for blue / navy blue
# Hue 100–130: navy (100-110), blue (110-125), royal blue (125-130)
# Low saturation floor (60) catches dark/faded navy shades
HSV_LOWER = np.array([100,  60,  30])
HSV_UPPER = np.array([130, 255, 255])
MIN_CONTOUR_AREA = 2000   # ignore noise smaller than this (px²)

# ============================================================
#  ESP32 CONNECTION
# ============================================================
try:
    esp32 = serial.Serial(PORT_COM, BAUD_RATE, timeout=0.1)
    time.sleep(2)
    print(f"[OK] Connected to ESP32 on {PORT_COM}")
except Exception as e:
    esp32 = None
    print(f"[WARNING] Could not connect to ESP32: {e}")
    print("          Running in camera-preview-only mode.")

_serial_lock = threading.Lock()

def send(cmd: str):
    """Send a newline-terminated command to the ESP32."""
    if esp32:
        with _serial_lock:
            try:
                esp32.write((cmd + '\n').encode())
            except Exception as ex:
                print(f"[SERIAL ERROR] {ex}")

# ============================================================
#  APPLICATION STATE  (shared between threads)
# ============================================================
auto_aim = False   # True = auto-tracking mode
running  = True    # False = shut down
servo_held = False # True = S is currently physically held down

# Set of currently held movement directions
held_dirs = set()
held_lock = threading.Lock()

# Last sent movement commands – avoid flooding the ESP32 with duplicates
prev_cmd_x = None
prev_cmd_y = None

# ============================================================
#  KEYBOARD HANDLER  (pynput – runs in a background thread)
# ============================================================
TOGGLE_KEYS = {
    keyboard.KeyCode.from_char('x'): 'CMD:MOTOR_X',
    keyboard.KeyCode.from_char('X'): 'CMD:MOTOR_X',
    keyboard.KeyCode.from_char('y'): 'CMD:MOTOR_Y',
    keyboard.KeyCode.from_char('Y'): 'CMD:MOTOR_Y',
    keyboard.KeyCode.from_char('a'): 'CMD:MODE',
    keyboard.KeyCode.from_char('A'): 'CMD:MODE',
    # S handled separately as press/release – not in TOGGLE_KEYS
    keyboard.KeyCode.from_char('l'): 'CMD:LASER',
    keyboard.KeyCode.from_char('L'): 'CMD:LASER',
}

ARROW_KEYS = {
    keyboard.Key.left:  'LEFT',
    keyboard.Key.right: 'RIGHT',
    keyboard.Key.up:    'UP',
    keyboard.Key.down:  'DOWN',
}

def on_press(key):
    global auto_aim, running

    # Quit
    if key in (keyboard.KeyCode.from_char('q'), keyboard.KeyCode.from_char('Q')):
        running = False
        return False   # stops the listener

    # Servo – press = move to 0°, release = return to 160°
    # servo_held blocks autorepeat from firing multiple toggles.
    if key in (keyboard.KeyCode.from_char('s'), keyboard.KeyCode.from_char('S')):
        global servo_held
        if not servo_held:
            servo_held = True
            send('CMD:SERVO')
        return

    # One-shot toggle commands
    if key in TOGGLE_KEYS:
        cmd = TOGGLE_KEYS[key]
        send(cmd)
        if 'MODE' in cmd:
            auto_aim = not auto_aim
            print(f"[MODE] {'AUTO-AIM' if auto_aim else 'MANUAL'}")
        return

    # Arrow keys – add direction to held set (manual mode only)
    if key in ARROW_KEYS and not auto_aim:
        with held_lock:
            held_dirs.add(ARROW_KEYS[key])

def on_release(key):
    global servo_held
    # Servo release – send toggle again to return to 160°
    if key in (keyboard.KeyCode.from_char('s'), keyboard.KeyCode.from_char('S')):
        servo_held = False
        send('CMD:SERVO')
        return
    # Always remove direction on key release, regardless of mode
    if key in ARROW_KEYS:
        with held_lock:
            held_dirs.discard(ARROW_KEYS[key])

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.daemon = True
listener.start()

# ============================================================
#  CAMERA
# ============================================================
cap = cv2.VideoCapture(CAMERA_ID)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)

if not cap.isOpened():
    print("[ERROR] Cannot open camera!")
    if esp32:
        esp32.close()
    exit(1)

# ============================================================
#  ON-SCREEN DISPLAY
# ============================================================
def draw_osd(frame, mode_auto, target=None):
    h, w = frame.shape[:2]

    # Mode label (bottom-left)
    col   = (0, 200, 0) if mode_auto else (0, 180, 255)
    label = "MODE: AUTO-AIM" if mode_auto else "MODE: MANUAL"
    cv2.putText(frame, label, (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

    # Crosshair at frame center
    cx, cy = CENTER_X, CENTER_Y
    cv2.line(frame,  (cx - 22, cy), (cx + 22, cy), (255, 255, 255), 2)
    cv2.line(frame,  (cx, cy - 22), (cx, cy + 22), (255, 255, 255), 2)
    cv2.circle(frame, (cx, cy), 4,  (255, 255, 255), -1)

    # Target overlay
    if target:
        tx, ty, tw, th = target
        sx, sy = tx + tw // 2, ty + th // 2
        cv2.rectangle(frame, (tx, ty), (tx + tw, ty + th), (255, 255, 0), 2)
        cv2.circle(frame, (sx, sy), 6, (0, 0, 255), -1)
        cv2.line(frame, (cx, cy), (sx, sy), (0, 255, 255), 2)
        ex, ey = sx - cx, sy - cy
        cv2.putText(frame, f"Err X:{ex:+d}  Y:{ey:+d}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # Key legend (top-right)
    legend = ["X/Y - motors", "A - auto/manual", "S - servo",
              "L - laser", "Arrows - move"]
    for i, ln in enumerate(legend):
        cv2.putText(frame, ln, (w - 145, 18 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.37, (200, 200, 200), 1)

# ============================================================
#  MAIN LOOP
# ============================================================
print("\nKeys (work globally – camera window does not need to be focused):")
print("  Arrows    – move X/Y axes  (hold = move, release = stop)")
print("  X / Y     – motor X / Y  on/off")
print("  A         – manual / auto-aim")
print("  S         – servo  |  L – laser  |  Q – quit\n")

while running:
    ok, frame = cap.read()
    if not ok:
        print("[ERROR] No frame from camera.")
        break

    # Rotate 90° clockwise + horizontal flip
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    frame = cv2.flip(frame, 1)

    # Crop center 600×600 from the 600×1024 rotated frame
    fh, fw = frame.shape[:2]          # fw=600, fh=1024
    y0 = (fh - CROP) // 2            # (1024-600)//2 = 212
    frame = frame[y0 : y0 + CROP, 0 : CROP]

    # ----------------------------------------------------------
    #  BLUE OBJECT DETECTION
    # ----------------------------------------------------------
    target_rect = None
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > MIN_CONTOUR_AREA:
            x, y, w, h = cv2.boundingRect(largest)
            target_rect = (x, y, w, h)
            sx, sy = x + w // 2, y + h // 2
            if auto_aim:
                send(f"X:{sx},Y:{sy}")

    # ----------------------------------------------------------
    #  SEND MOVEMENT COMMANDS  (every frame, only on state change)
    # ----------------------------------------------------------
    if not auto_aim:
        with held_lock:
            dirs = set(held_dirs)

        cmd_x = 'CMD:LEFT'  if 'LEFT'  in dirs else \
                'CMD:RIGHT' if 'RIGHT' in dirs else 'CMD:STOP_X'

        cmd_y = 'CMD:UP'   if 'UP'   in dirs else \
                'CMD:DOWN' if 'DOWN' in dirs else 'CMD:STOP_Y'

        if cmd_x != prev_cmd_x:
            send(cmd_x)
            prev_cmd_x = cmd_x
        if cmd_y != prev_cmd_y:
            send(cmd_y)
            prev_cmd_y = cmd_y
    else:
        # Reset cached commands when entering auto mode so that
        # returning to manual doesn't leave a stale state
        prev_cmd_x = None
        prev_cmd_y = None

    # ----------------------------------------------------------
    #  DISPLAY
    # ----------------------------------------------------------
    draw_osd(frame, auto_aim, target_rect)
    cv2.imshow("Turret Control", frame)

    # waitKey is only here to keep the OpenCV window responsive
    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False
        break

# ============================================================
#  CLEANUP
# ============================================================
send("CMD:STOP_X")
send("CMD:STOP_Y")
cap.release()
if esp32:
    esp32.close()
cv2.destroyAllWindows()
listener.stop()
print("Closed.")