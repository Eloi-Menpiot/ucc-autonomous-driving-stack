#!/usr/bin/env python3

import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

# ── tunables ──────────────────────────────────────────────────────────────────
CAMERA_URL      = 'tcp://192.168.137.25:34808'
DRIVE_TOPIC     = '/drive'          # ackermann_mux nav input (priority 10)
PUBLISH_HZ      = 10.0              # must be > 1/mux_timeout (mux timeout = 0.2 s)
VISION_HZ       = 2.0               # camera processing rate (slow is fine)

SPEED_STRAIGHT  = 0.2               # m/s when path is clear
SPEED_TURN      = 0.15              # m/s while steering
STEER_MILD      = 0.25              # rad — gentle correction
STEER_HARD      = 0.40              # rad — full turn away from obstacle

# How much of each column (0–255 mean) must look like floor to be "clear"
FLOOR_THRESH_CLEAR = 100            # center column: go straight
FLOOR_THRESH_TURN  = 60             # side column: can steer that way
FLOOR_COLOR_TOL    = 55.0           # Euclidean BGR distance for floor match

# ROI: analyse the band between these fractional heights of the frame
ROI_TOP_FRAC    = 0.40              # ignore top (ceiling, far wall)
ROI_BOT_FRAC    = 0.92              # ignore very bottom strip (used for sampling)
SAMPLE_TOP_FRAC = 0.93              # floor colour sample band (always floor)
# ─────────────────────────────────────────────────────────────────────────────


class CameraNav(Node):
    def __init__(self):
        super().__init__('camera_nav_node')

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)

        # Cached command — updated by vision thread, published by fast timer
        self._lock = threading.Lock()
        self._speed = 0.0
        self._steer = 0.0
        self._frame_ok = False      # stays False until first good camera frame

        # Open camera in background thread — don't block __init__
        self._cap = None
        self._cap_thread = threading.Thread(target=self._open_camera, daemon=True)
        self._cap_thread.start()

        self.create_timer(1.0 / PUBLISH_HZ, self._publish_cmd)
        self.create_timer(1.0 / VISION_HZ,  self._vision_tick)

        self.get_logger().info(
            f'camera_nav started  camera={CAMERA_URL}  topic={DRIVE_TOPIC}')

    # ── camera open ───────────────────────────────────────────────────────────

    def _open_camera(self):
        self.get_logger().info('Opening camera stream…')
        cap = cv2.VideoCapture(CAMERA_URL)
        if cap.isOpened():
            self._cap = cap
            self.get_logger().info('Camera stream opened.')
        else:
            self.get_logger().error(f'Could not open camera: {CAMERA_URL}')

    # ── fast publish timer (10 Hz) ────────────────────────────────────────────

    def _publish_cmd(self):
        with self._lock:
            speed = self._speed
            steer = self._steer

        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)

    # ── slow vision timer (2 Hz) ──────────────────────────────────────────────

    def _vision_tick(self):
        if self._cap is None or not self._cap.isOpened():
            return

        ret, frame = self._cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Camera read failed — holding last command.')
            return

        speed, steer = self._compute_drive(frame)
        with self._lock:
            self._speed = speed
            self._steer = steer
            self._frame_ok = True

        self.get_logger().info(
            f'vision → speed={speed:.2f}  steer={steer:+.2f} rad')

    # ── vision ────────────────────────────────────────────────────────────────

    def _compute_drive(self, frame):
        h, w = frame.shape[:2]

        # -- 1. sample floor colour from bottom strip (always floor) -----------
        sample = frame[int(h * SAMPLE_TOP_FRAC):h, w // 4: 3 * w // 4]
        floor_bgr = np.mean(sample.reshape(-1, 3), axis=0)  # [B, G, R]

        # -- 2. build floor mask in the analysis ROI ---------------------------
        roi = frame[int(h * ROI_TOP_FRAC): int(h * ROI_BOT_FRAC), :]
        diff = roi.astype(np.float32) - floor_bgr           # broadcast over pixels
        dist = np.sqrt(np.sum(diff ** 2, axis=2))           # per-pixel colour distance
        floor_mask = (dist < FLOOR_COLOR_TOL).astype(np.uint8)  # 1 = floor, 0 = obstacle

        # morphological clean-up
        kernel = np.ones((5, 5), np.uint8)
        floor_mask = cv2.morphologyEx(floor_mask, cv2.MORPH_OPEN, kernel)

        # -- 3. score left / centre / right columns (mean of floor pixels) -----
        third = w // 3
        left_score   = np.mean(floor_mask[:, :third])        * 255
        centre_score = np.mean(floor_mask[:, third:2*third]) * 255
        right_score  = np.mean(floor_mask[:, 2*third:])      * 255

        self.get_logger().debug(
            f'floor scores  L={left_score:.0f}  C={centre_score:.0f}  R={right_score:.0f}')

        # -- 4. decision logic -------------------------------------------------
        centre_clear = centre_score >= FLOOR_THRESH_CLEAR
        left_ok      = left_score   >= FLOOR_THRESH_TURN
        right_ok     = right_score  >= FLOOR_THRESH_TURN

        if centre_clear:
            # Path ahead is open — go straight, with a gentle bias toward the
            # clearer side to avoid drifting toward walls
            if left_score > right_score + 30:
                return SPEED_STRAIGHT, -STEER_MILD   # nudge left (more floor left)
            elif right_score > left_score + 30:
                return SPEED_STRAIGHT,  STEER_MILD   # nudge right
            else:
                return SPEED_STRAIGHT, 0.0

        elif left_ok or right_ok:
            # Centre blocked — turn toward whichever side has more floor
            if left_score >= right_score and left_ok:
                steer = -STEER_HARD                  # negative = left on F1Tenth
            elif right_ok:
                steer = STEER_HARD
            else:
                steer = -STEER_HARD if left_score > right_score else STEER_HARD
            return SPEED_TURN, steer

        else:
            # All paths appear blocked — stop
            self.get_logger().warn('All paths blocked — stopping.')
            return 0.0, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = CameraNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
