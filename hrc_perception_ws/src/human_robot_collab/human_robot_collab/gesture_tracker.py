#!/usr/bin/env python3
"""
Gesture tracker — MediaPipe Hands → /gesture_pause + /gesture_cmd

Lets the operator pause and resume the robot's waypoint task with hand
gestures, no teach pendant needed:

    OPEN PALM (all 4 fingers extended)  → PAUSE the task
    FIST      (all 4 fingers curled)    → RESUME the task

Classification is pure landmark geometry (no ML beyond MediaPipe itself):
a finger counts as extended when its TIP is farther from the wrist than its
PIP joint by a margin, and curled when it is closer. A gesture must be held
for GESTURE_HOLD_FRAMES consecutive frames before it toggles the state, so
a hand sweeping through the frame never accidentally pauses the robot.

Published topics:
    /gesture_pause  (std_msgs/Bool)   — latched pause state, republished at
                                        10 Hz so late-joining BT gates sync.
    /gesture_cmd    (std_msgs/String) — debounced raw gesture on each change
                                        ("open_palm" / "fist" / "none"),
                                        useful for debugging and demo HUDs.

The BT side: GesturePauseGate nodes in bt_action.xml return RUNNING while
/gesture_pause is true, holding the tree between waypoints. If this node is
not running at all, no message is ever published and the gates default to
not-paused — the pipeline behaves exactly as before this feature existed.

Input source (ROS parameters, same scheme as body_tracker):
    image_topic     (string, default "") — subscribe to this sensor_msgs/Image
                                           topic. Empty = open the webcam.
    camera_index    (int, default 0)     — cv2.VideoCapture index.
    show_preview    (bool, default True) — OpenCV window with overlay.
    flip_horizontal (bool, default False)

NOTE (webcam runs): two nodes cannot stream the same webcam. If
body_tracker is also running from the webcam, start it with
-p publish_frames:=true and point this node at the republished topic:
    ros2 run human_robot_collab gesture_tracker --ros-args -p image_topic:=/webcam/image_raw

Requires: pip install mediapipe opencv-python
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from collections import deque

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from human_robot_collab.mp_compat import (
    HandDetector, HAND_CONNECTIONS, draw_landmarks)

CAPTURE_PERIOD_S    = 1.0 / 30.0
REPUBLISH_PERIOD_S  = 0.1     # 10 Hz pause-state heartbeat
GESTURE_HOLD_FRAMES = 8       # ~0.27 s at 30 FPS before a gesture takes effect

# MediaPipe Hands landmark indices: (PIP joint, TIP) per finger.
# Thumb is excluded — its PIP/TIP geometry is unreliable for this test.
FINGER_JOINTS = [
    (6, 8),     # index
    (10, 12),   # middle
    (14, 16),   # ring
    (18, 20),   # pinky
]
WRIST = 0

EXTENDED_MARGIN = 1.10   # tip > pip * margin  → finger extended
CURLED_MARGIN   = 0.95   # tip < pip * margin  → finger curled


def classify_gesture(landmarks):
    """Classify one hand's 21 landmarks: 'open_palm', 'fist' or 'none'."""
    w = np.array([landmarks[WRIST].x, landmarks[WRIST].y])

    def dist(i):
        return float(np.hypot(landmarks[i].x - w[0], landmarks[i].y - w[1]))

    extended = 0
    curled   = 0
    for pip, tip in FINGER_JOINTS:
        d_pip, d_tip = dist(pip), dist(tip)
        if d_tip > d_pip * EXTENDED_MARGIN:
            extended += 1
        elif d_tip < d_pip * CURLED_MARGIN:
            curled += 1

    if extended == len(FINGER_JOINTS):
        return "open_palm"
    if curled == len(FINGER_JOINTS):
        return "fist"
    return "none"


class GestureTracker(Node):

    def __init__(self):
        super().__init__("gesture_tracker")

        self.declare_parameter("image_topic", "")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("show_preview", True)
        self.declare_parameter("flip_horizontal", False)

        self.image_topic  = self.get_parameter("image_topic") \
                                .get_parameter_value().string_value.strip()
        self.camera_index = self.get_parameter("camera_index") \
                                .get_parameter_value().integer_value
        self.show_preview = self.get_parameter("show_preview") \
                                .get_parameter_value().bool_value
        self.flip_h       = self.get_parameter("flip_horizontal") \
                                .get_parameter_value().bool_value

        self.bridge = CvBridge()
        # Works with both the legacy mp.solutions API and the new Tasks API
        # (auto-downloads the model file on first run with new MediaPipe).
        self.hands = HandDetector()

        self.paused  = False
        self._stable = "none"                      # last debounced gesture
        self._window = deque(maxlen=GESTURE_HOLD_FRAMES)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.pause_pub = self.create_publisher(Bool, "/gesture_pause", reliable_qos)
        self.cmd_pub   = self.create_publisher(String, "/gesture_cmd", reliable_qos)

        self.cap = None
        if self.image_topic:
            self.create_subscription(Image, self.image_topic, self._image_cb, 10)
            source_str = f"topic {self.image_topic}"
        else:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                raise RuntimeError(
                    f"Cannot open webcam index {self.camera_index}")
            self.create_timer(CAPTURE_PERIOD_S, self._capture_tick)
            source_str = f"webcam index {self.camera_index}"

        self.create_timer(REPUBLISH_PERIOD_S, self._heartbeat)

        self.get_logger().info(
            f"GestureTracker ready — source: {source_str} — "
            f"OPEN PALM pauses, FIST resumes.")

    # ─────────────────────────────────────────────────────────────────────────
    #  FRAME SOURCES
    # ─────────────────────────────────────────────────────────────────────────

    def _capture_tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Webcam frame grab failed.",
                                   throttle_duration_sec=2.0)
            return
        self._process_frame(frame)

    def _image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._process_frame(frame)

    # ─────────────────────────────────────────────────────────────────────────
    #  DETECTION + STATE MACHINE
    # ─────────────────────────────────────────────────────────────────────────

    def _process_frame(self, frame_bgr):
        if self.flip_h:
            frame_bgr = cv2.flip(frame_bgr, 1)

        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        landmarks = self.hands.detect(rgb)

        gesture = "none"
        if landmarks is not None:
            gesture = classify_gesture(landmarks)

        self._window.append(gesture)
        if (len(self._window) == GESTURE_HOLD_FRAMES
                and len(set(self._window)) == 1
                and gesture != self._stable):
            self._stable = gesture
            self.cmd_pub.publish(String(data=gesture))
            self._apply_gesture(gesture)

        if self.show_preview:
            self._draw_preview(frame_bgr, landmarks, gesture)

    def _apply_gesture(self, gesture):
        if gesture == "open_palm" and not self.paused:
            self.paused = True
            self._publish_pause()
            self.get_logger().info("OPEN PALM detected — task PAUSED. "
                                   "Show a fist to resume.")
        elif gesture == "fist" and self.paused:
            self.paused = False
            self._publish_pause()
            self.get_logger().info("FIST detected — task RESUMED.")

    def _publish_pause(self):
        self.pause_pub.publish(Bool(data=self.paused))

    def _heartbeat(self):
        # Periodic republish so BT gates that start late learn the state.
        self._publish_pause()

    # ─────────────────────────────────────────────────────────────────────────
    #  PREVIEW
    # ─────────────────────────────────────────────────────────────────────────

    def _draw_preview(self, frame, landmarks, gesture):
        preview = frame.copy()
        if landmarks is not None:
            draw_landmarks(preview, landmarks, HAND_CONNECTIONS)

        if self.paused:
            status, colour = "PAUSED — fist to resume", (0, 0, 255)
        else:
            status, colour = "RUNNING — palm to pause", (0, 200, 0)
        cv2.putText(preview, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
        cv2.putText(preview, f"gesture: {gesture}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("gesture_tracker — MediaPipe Hands", preview)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        if self.show_preview:
            cv2.destroyAllWindows()
        self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
