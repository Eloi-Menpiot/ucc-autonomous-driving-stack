#!/usr/bin/env python3
"""
claude_nav_node.py

Gemini pilots the F1Tenth car by analysing camera frames via the Google Gemini API.

Pipeline:
  Camera → frame capture (slow timer) → Gemini vision API → drive command
                                                           ↓
  /drive ← keepalive publisher (10 Hz) ← cached command ──┘

Requires: GEMINI_API_KEY environment variable (free at aistudio.google.com)
Camera:   GStreamer TCP stream (start tcp_server.py first) OR /tmp/camframes/frame.jpg
"""

import os
import base64
import json
import threading

import cv2
import numpy as np
import requests

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

# ── config ────────────────────────────────────────────────────────────────────
# Primary: GStreamer TCP stream. Run first:
#   python3 /home/sample-apps-for-robotics-platforms/RB5/linux_kernel_4_x/ \
#           GStreamer-apps/python/gst_streaming/tcp_server.py 0 34808 192.168.137.25
CAMERA_TCP   = 'tcp://192.168.137.25:34808'
CAMERA_FILE  = '/tmp/camframes/frame.jpg'   # fallback if TCP unavailable

DRIVE_TOPIC  = '/drive'       # ackermann_mux nav input, priority 10
PUBLISH_HZ   = 10.0           # keepalive rate — must be > 1/mux_timeout (0.2 s)
API_HZ       = 0.4            # API call rate (1 call per 2.5 s)
MODEL        = 'gemini-2.0-flash'
FRAME_W      = 640            # resize before sending to API (saves tokens/latency)
FRAME_H      = 480

MAX_SPEED    = 0.25           # m/s hard cap applied after API response
MAX_STEER    = 0.40           # rad hard cap
# ─────────────────────────────────────────────────────────────────────────────

SYSTEM_PROMPT = """You are the AI brain of an F1Tenth autonomous car (wheelbase 0.25 m) \
driving in a university robotics lab. The camera is mounted at the front of the car, \
slightly elevated, pointing forward.

Your only goal is to keep driving without hitting walls, furniture, people, or any obstacle.

Output format — respond with ONLY a valid JSON object, nothing else:
{"speed": <float>, "steering_angle": <float>, "reason": "<short string>"}

Parameter ranges and meaning:
- speed: 0.0 (stopped) to 0.25 (fast). Use 0.0 to stop, 0.1 for cautious, 0.2 for confident.
- steering_angle: negative = turn LEFT, positive = turn RIGHT, in radians. Max ±0.40 rad.
- reason: one short phrase explaining your decision (for logging only).

Decision guidelines:
- Open corridor or room ahead → speed 0.15–0.20, steer gently toward the centre of free space.
- Wall or obstacle filling the centre of the frame → steer toward the clearer side, reduce speed.
- Obstacle very close (large in frame) → speed 0.0, steer away before moving.
- Turning corner → speed 0.10, steer toward the opening.
- Uncertain or ambiguous scene → speed 0.0, steering_angle 0.0.
- Prefer gentle corrections over sharp swerves.
- You are driving slowly — do not over-steer."""


class GeminiNav(Node):
    def __init__(self):
        super().__init__('claude_nav_node')

        api_key = os.environ.get('GEMINI_API_KEY')
        if not api_key:
            self.get_logger().error('GEMINI_API_KEY not set — exiting.')
            raise RuntimeError('GEMINI_API_KEY not set')

        self._api_key = api_key
        self._api_url = (
            f'https://generativelanguage.googleapis.com/v1beta/models/'
            f'{MODEL}:generateContent?key={api_key}'
        )

        self._drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)

        # Cached command — updated by API thread, published by keepalive timer
        self._lock = threading.Lock()
        self._speed = 0.0
        self._steer = 0.0
        self._api_active = False    # True while an API call is in flight

        # Camera — opened in background so __init__ doesn't block
        self._cap = None
        self._use_file = False
        threading.Thread(target=self._open_camera, daemon=True).start()

        self.create_timer(1.0 / PUBLISH_HZ, self._publish_cmd)
        self.create_timer(1.0 / API_HZ,     self._api_tick)

        self.get_logger().info(
            f'gemini_nav started | model={MODEL} | topic={DRIVE_TOPIC}')

    # ── camera ────────────────────────────────────────────────────────────────

    def _open_camera(self):
        self.get_logger().info(f'Trying TCP stream: {CAMERA_TCP}')
        cap = cv2.VideoCapture(CAMERA_TCP)
        if cap.isOpened():
            self._cap = cap
            self.get_logger().info('TCP stream opened.')
            return
        self.get_logger().warn('TCP stream failed — using file fallback.')
        self._use_file = True   # _get_frame will read CAMERA_FILE directly

    def _get_frame(self):
        """Return a BGR numpy frame, or None on failure."""
        if self._use_file:
            if not os.path.exists(CAMERA_FILE):
                return None
            frame = cv2.imread(CAMERA_FILE)
            return frame   # may be None if file unreadable

        if self._cap is None or not self._cap.isOpened():
            return None
        ret, frame = self._cap.read()
        return frame if ret else None

    # ── keepalive publisher (10 Hz) ───────────────────────────────────────────

    def _publish_cmd(self):
        with self._lock:
            speed = self._speed
            steer = self._steer
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self._drive_pub.publish(msg)

    # ── API tick (0.4 Hz) ─────────────────────────────────────────────────────

    def _api_tick(self):
        with self._lock:
            if self._api_active:
                return          # previous call still running — skip this tick
            self._api_active = True

        # Run API call in a thread so the ROS executor isn't blocked
        threading.Thread(target=self._run_api_call, daemon=True).start()

    def _run_api_call(self):
        try:
            frame = self._get_frame()
            if frame is None:
                self.get_logger().warn('No camera frame — stopping.')
                with self._lock:
                    self._speed = 0.0
                    self._steer = 0.0
                return

            speed, steer, reason = self._query_gemini(frame)

            with self._lock:
                self._speed = speed
                self._steer = steer

            self.get_logger().info(
                f'Gemini → speed={speed:.2f}  steer={steer:+.2f} rad  "{reason}"')

        except Exception as e:
            self.get_logger().error(f'API call failed: {e}')
            with self._lock:
                self._speed = 0.0   # safe stop on any error
        finally:
            with self._lock:
                self._api_active = False

    # ── Gemini vision API ─────────────────────────────────────────────────────

    def _query_gemini(self, frame):
        """Send frame to Gemini REST API, return (speed, steering_angle, reason)."""
        small = cv2.resize(frame, (FRAME_W, FRAME_H))
        _, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 80])
        b64 = base64.standard_b64encode(buf.tobytes()).decode('utf-8')

        payload = {
            'system_instruction': {'parts': [{'text': SYSTEM_PROMPT}]},
            'contents': [{
                'parts': [
                    {'inline_data': {'mime_type': 'image/jpeg', 'data': b64}},
                    {'text': 'Analyse this frame and respond with your drive command JSON.'},
                ]
            }],
            'generationConfig': {'maxOutputTokens': 150},
        }

        resp = requests.post(self._api_url, json=payload, timeout=10)
        resp.raise_for_status()
        raw = resp.json()['candidates'][0]['content']['parts'][0]['text'].strip()

        # Strip markdown code fences if present
        if '```' in raw:
            raw = raw.split('```')[1]
            if raw.startswith('json'):
                raw = raw[4:]
        data = json.loads(raw)

        speed = float(np.clip(data['speed'],          0.0,       MAX_SPEED))
        steer = float(np.clip(data['steering_angle'], -MAX_STEER, MAX_STEER))
        reason = str(data.get('reason', ''))
        return speed, steer, reason


def main(args=None):
    rclpy.init(args=args)
    node = GeminiNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
