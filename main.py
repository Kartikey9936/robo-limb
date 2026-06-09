"""
ROBOLIMB - Complete Pipeline
Flow: Webcam → YOLOv8 Detection → 3D Coordinates → Inverse Kinematics → Pick Object
"""

import cv2
import numpy as np
import math
import time
import serial  # pip install pyserial  (for Arduino/servo communication)
from ultralytics import YOLO

# ═══════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════

# Camera settings
CAMERA_INDEX       = 0
FRAME_WIDTH        = 640
FRAME_HEIGHT       = 480
FOCAL_LENGTH_PX    = 800          # calibrate with checkerboard for best results
KNOWN_OBJECT_WIDTH = 0.07         # meters — average object width (e.g. small cup ~7cm)

# Workspace (meters) — adjust to your arm's reach
WORKSPACE_X_RANGE  = (-0.3, 0.3)
WORKSPACE_Y_RANGE  = (0.0,  0.5)
WORKSPACE_Z_RANGE  = (0.0,  0.4)

# Robotic arm link lengths (meters) — CHANGE to your robot's actual values
L1 = 0.15    # base to shoulder
L2 = 0.15    # shoulder to elbow
L3 = 0.10    # elbow to wrist
L4 = 0.06    # wrist to gripper tip

# Serial port for Arduino/servo controller (set to None to run in simulation mode)
SERIAL_PORT = None          # e.g. "COM5" on Windows, "/dev/ttyUSB0" on Linux
SERIAL_BAUD = 115200

# Detection confidence threshold
CONF_THRESHOLD = 0.55

# Gripper servo values
GRIPPER_OPEN   = 0
GRIPPER_CLOSED = 90


# ═══════════════════════════════════════════════════════
#  CAMERA CALIBRATION  (replace with your values)
# ═══════════════════════════════════════════════════════

camera_matrix = np.array([
    [FOCAL_LENGTH_PX, 0,               FRAME_WIDTH  / 2],
    [0,               FOCAL_LENGTH_PX, FRAME_HEIGHT / 2],
    [0,               0,               1               ]
], dtype=np.float64)

dist_coeffs = np.zeros((5, 1), dtype=np.float64)


# ═══════════════════════════════════════════════════════
#  SERIAL COMMUNICATION  (Arduino / servo board)
# ═══════════════════════════════════════════════════════

class SerialController:
    def __init__(self, port, baud):
        self.connected = False
        if port:
            try:
                self.ser = serial.Serial(port, baud, timeout=1)
                time.sleep(2)
                self.connected = True
                print(f"[SERIAL] Connected to {port}")
            except Exception as e:
                print(f"[SERIAL] Could not connect: {e} — running in simulation mode")
        else:
            print("[SERIAL] No port specified — simulation mode")

    def send_angles(self, angles: dict):
        """angles = {'base': θ, 'shoulder': θ, 'elbow': θ, 'wrist': θ, 'gripper': θ}"""
        if self.connected:
            cmd = "MOVE"
            for joint, angle in angles.items():
                cmd += f",{joint}:{int(angle)}"
            cmd += "\n"
            self.ser.write(cmd.encode())
            print(f"[SERIAL] Sent → {cmd.strip()}")
        else:
            print(f"[SIM]    Angles → {angles}")

    def close(self):
        if self.connected:
            self.ser.close()


# ═══════════════════════════════════════════════════════
#  INVERSE KINEMATICS  (4-DOF planar + base rotation)
# ═══════════════════════════════════════════════════════

class InverseKinematics:
    def __init__(self, l1, l2, l3, l4):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4

    def solve(self, x: float, y: float, z: float, pitch_deg: float = -90.0):
        """
        Solve IK for target position (x, y, z) in meters.
        Returns dict of joint angles in degrees, or None if unreachable.

        Convention:
          x — left/right from robot base
          y — forward from robot base
          z — height above base
          pitch_deg — desired end-effector pitch (default -90 = pointing down)
        """
        # ── Base rotation (yaw) ──────────────────────────────
        base_angle = math.degrees(math.atan2(x, y))

        # ── Planar reach in the arm's vertical plane ─────────
        r_horiz = math.sqrt(x**2 + y**2)    # horizontal distance from base
        dz      = z - 0                      # height relative to base joint

        # Account for gripper offset (end-effector points downward by default)
        pitch_rad = math.radians(pitch_deg)
        wx = r_horiz - self.l4 * math.cos(pitch_rad)
        wz = dz      - self.l4 * math.sin(pitch_rad)

        # Effective reach to wrist (l2 + l3 arm)
        dist = math.sqrt(wx**2 + wz**2)
        arm_reach = self.l2 + self.l3

        if dist > arm_reach:
            print(f"[IK] Target unreachable  dist={dist:.3f}m  max={arm_reach:.3f}m")
            return None

        # ── Elbow angle (cosine rule) ────────────────────────
        cos_elbow = (dist**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))        # clamp for numerical safety
        elbow_angle = math.degrees(math.acos(cos_elbow))  # elbow-up solution

        # ── Shoulder angle ───────────────────────────────────
        alpha = math.atan2(wz, wx)
        beta  = math.atan2(
            self.l3 * math.sin(math.radians(elbow_angle)),
            self.l2 + self.l3 * math.cos(math.radians(elbow_angle))
        )
        shoulder_angle = math.degrees(alpha - beta)

        # ── Wrist pitch to maintain desired end-effector pitch ─
        wrist_angle = pitch_deg - shoulder_angle - elbow_angle

        angles = {
            "base":     round(base_angle,     2),
            "shoulder": round(shoulder_angle, 2),
            "elbow":    round(elbow_angle,    2),
            "wrist":    round(wrist_angle,    2),
            "gripper":  GRIPPER_OPEN
        }
        return angles

    def clamp_angles(self, angles: dict) -> dict:
        """Clamp to safe servo limits — adjust per your robot."""
        limits = {
            "base":     (-90, 90),
            "shoulder": (-30, 150),
            "elbow":    (0,   150),
            "wrist":    (-90, 90),
            "gripper":  (0,   90)
        }
        for joint, (lo, hi) in limits.items():
            if joint in angles:
                angles[joint] = max(lo, min(hi, angles[joint]))
        return angles


# ═══════════════════════════════════════════════════════
#  3D COORDINATE ESTIMATION  (monocular webcam)
# ═══════════════════════════════════════════════════════

def estimate_3d_position(bbox, frame_w, frame_h, known_width_m, focal_px):
    """
    Estimate real-world (X, Y, Z) from bounding box using pinhole model.
    Z = distance along camera optical axis (depth).
    X, Y = lateral offsets from camera centre.
    """
    x1, y1, x2, y2 = bbox
    obj_px_width = x2 - x1
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2

    if obj_px_width < 5:
        return None

    Z = (known_width_m * focal_px) / obj_px_width          # depth in metres
    X = (cx - frame_w / 2) * Z / focal_px                  # lateral X
    Y = (cy - frame_h / 2) * Z / focal_px                  # vertical Y (camera frame)

    # Convert camera frame → robot frame:
    # camera Y (down)  →  robot Z (height, inverted)
    # camera Z (depth) →  robot Y (forward)
    robot_x =  X
    robot_y =  Z
    robot_z = -Y + 0.30     # offset so object rests at positive height

    return (robot_x, robot_y, robot_z)


# ═══════════════════════════════════════════════════════
#  PICK SEQUENCE
# ═══════════════════════════════════════════════════════

def pick_object(controller: SerialController, ik: InverseKinematics,
                target_xyz: tuple, label: str):
    """Full pick routine: move above → descend → grip → lift."""

    x, y, z = target_xyz
    print(f"\n[PICK] Target '{label}'  x={x:.3f}  y={y:.3f}  z={z:.3f}")

    steps = [
        ("HOME",        (0.0, 0.20, 0.25), GRIPPER_OPEN),    # safe home position
        ("ABOVE",       (x,   y,    z + 0.08), GRIPPER_OPEN),  # 8 cm above object
        ("DESCEND",     (x,   y,    z + 0.01), GRIPPER_OPEN),  # just above object
        ("GRIP",        (x,   y,    z + 0.01), GRIPPER_CLOSED),# close gripper
        ("LIFT",        (x,   y,    z + 0.12), GRIPPER_CLOSED),# lift object
        ("DELIVER",     (0.0, 0.15, 0.20),     GRIPPER_CLOSED),# carry to drop zone
        ("RELEASE",     (0.0, 0.15, 0.20),     GRIPPER_OPEN),  # open gripper
        ("HOME",        (0.0, 0.20, 0.25), GRIPPER_OPEN),    # return home
    ]

    for step_name, pos, gripper in steps:
        px, py, pz = pos
        angles = ik.solve(px, py, pz)
        if angles is None:
            print(f"[PICK] Step '{step_name}' unreachable — aborting pick.")
            return False

        angles = ik.clamp_angles(angles)
        angles["gripper"] = gripper
        print(f"  [{step_name}]  {angles}")
        controller.send_angles(angles)
        time.sleep(0.8)     # wait for servos to reach position

    print(f"[PICK] '{label}' picked and delivered ✓\n")
    return True


# ═══════════════════════════════════════════════════════
#  MAIN LOOP
# ═══════════════════════════════════════════════════════

def main():
    print("=" * 55)
    print("       ROBOLIMB — Object Detection + Pick System")
    print("=" * 55)

    # Load YOLO model
    print("[INIT] Loading YOLOv8 model...")
    model = YOLO("yolov8s.pt")          # downloads automatically on first run
    print("[INIT] Model loaded.")

    # Init subsystems
    controller = SerialController(SERIAL_PORT, SERIAL_BAUD)
    ik         = InverseKinematics(L1, L2, L3, L4)

    # Open webcam
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print("[ERROR] Cannot open webcam.")
        return

    print("\n[INFO] Press  SPACE  to pick the highest-confidence object")
    print("[INFO] Press  Q      to quit\n")

    last_detections = []
    picking = False

    while True:
        ret, raw_frame = cap.read()
        if not ret:
            print("[ERROR] Frame grab failed.")
            break

        frame = cv2.undistort(raw_frame, camera_matrix, dist_coeffs)
        h, w  = frame.shape[:2]

        # ── Run YOLO ────────────────────────────────────────
        results      = model(frame, conf=CONF_THRESHOLD, verbose=False)
        detections   = []

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf  = float(box.conf[0])
            cls   = int(box.cls[0])
            label = model.names[cls]

            pos3d = estimate_3d_position(
                (x1, y1, x2, y2), w, h, KNOWN_OBJECT_WIDTH, FOCAL_LENGTH_PX
            )
            if pos3d is None:
                continue

            detections.append({
                "label": label, "conf": conf,
                "bbox":  (x1, y1, x2, y2), "pos3d": pos3d
            })

            # ── Draw bounding box ──────────────────────────
            rx, ry, rz = pos3d
            color = (0, 255, 80)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{label}  {conf:.2f}",
                        (x1, y1 - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cv2.putText(frame,
                        f"X:{rx:.2f}m  Y:{ry:.2f}m  Z:{rz:.2f}m",
                        (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 220, 0), 1)

        last_detections = detections

        # ── Status overlay ───────────────────────────────
        status = f"Objects: {len(detections)}   [SPACE]=Pick  [Q]=Quit"
        cv2.putText(frame, status, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("ROBOLIMB — Detection", frame)

        key = cv2.waitKey(1) & 0xFF

        # ── PICK on SPACE ────────────────────────────────
        if key == ord(' ') and last_detections and not picking:
            # Pick the detection with highest confidence
            best = max(last_detections, key=lambda d: d["conf"])
            print(f"\n[USER] Pick triggered for: {best['label']} (conf={best['conf']:.2f})")
            picking = True
            pick_object(controller, ik, best["pos3d"], best["label"])
            picking = False

        elif key == ord('q'):
            print("[INFO] Quit.")
            break

    cap.release()
    cv2.destroyAllWindows()
    controller.close()


if __name__ == "__main__":
    main()