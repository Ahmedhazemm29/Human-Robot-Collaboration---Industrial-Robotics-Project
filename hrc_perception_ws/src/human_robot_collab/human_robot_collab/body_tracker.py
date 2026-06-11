#!/usr/bin/env python3
"""
Body tracker — MediaPipe Pose → /body_landmarks

Python-side counterpart of the C++ MediaPipe hand-tracking node, used when
hand_to_collision runs with tracking_mode:=body. Publishes all 33 MediaPipe
Pose landmarks as a PoseArray in PIXEL coordinates (normalised landmark
coords scaled to the 640x480 frame the rest of the pipeline assumes):

    position.x = u  (px, 0..640)
    position.y = v  (px, 0..480)
    position.z = landmark visibility (0..1)

Nothing is published while no person is detected, so the collision node's
STALE_TIMEOUT removes the body model exactly like the hand box.

Input source (ROS parameters):
    image_topic   (string, default "")  — subscribe to this sensor_msgs/Image
                                          topic (e.g. /image_raw from the
                                          Kinect driver). Empty = use webcam.
    camera_index  (int, default 0)      — cv2.VideoCapture device index,
                                          only used when image_topic is "".
    show_preview  (bool, default True)  — OpenCV window with the skeleton
                                          overlay (disable on headless runs).
    flip_horizontal (bool, default False) — mirror the frame before
                                          detection (some webcams pre-mirror).
    publish_frames (bool, default False) — republish captured frames on
                                          /webcam/image_raw so other nodes
                                          (e.g. gesture_tracker) can consume
                                          the same webcam without opening it.

Run (webcam, matches WEBCAM_MODE=True in hand_to_collision.py):
    ros2 run human_robot_collab body_tracker
Run (Kinect RGB stream):
    ros2 run human_robot_collab body_tracker --ros-args -p image_topic:=/image_raw

Requires: pip install mediapipe opencv-python
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import cv2
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image

from human_robot_collab.mp_compat import (
    PoseDetector, POSE_CONNECTIONS, draw_landmarks)

# Frame size the downstream pipeline (hand_to_collision.py) assumes.
IMG_W = 640
IMG_H = 480

CAPTURE_PERIOD_S = 1.0 / 30.0   # webcam polling rate


class BodyTracker(Node):

    def __init__(self):
        super().__init__("body_tracker")

        self.declare_parameter("image_topic", "")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("show_preview", True)
        self.declare_parameter("flip_horizontal", False)
        self.declare_parameter("publish_frames", False)

        self.image_topic  = self.get_parameter("image_topic") \
                                .get_parameter_value().string_value.strip()
        self.camera_index = self.get_parameter("camera_index") \
                                .get_parameter_value().integer_value
        self.show_preview = self.get_parameter("show_preview") \
                                .get_parameter_value().bool_value
        self.flip_h       = self.get_parameter("flip_horizontal") \
                                .get_parameter_value().bool_value
        self.publish_frames = self.get_parameter("publish_frames") \
                                  .get_parameter_value().bool_value

        self.bridge = CvBridge()
        # Works with both the legacy mp.solutions API and the new Tasks API
        # (auto-downloads the model file on first run with new MediaPipe).
        self.pose = PoseDetector()

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.landmarks_pub = self.create_publisher(
            PoseArray, "/body_landmarks", reliable_qos)

        self.frame_pub = None
        if self.publish_frames:
            self.frame_pub = self.create_publisher(
                Image, "/webcam/image_raw", 10)

        self.cap = None
        if self.image_topic:
            self.create_subscription(
                Image, self.image_topic, self._image_cb, 10)
            source_str = f"topic {self.image_topic}"
        else:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                raise RuntimeError(
                    f"Cannot open webcam index {self.camera_index}")
            self.create_timer(CAPTURE_PERIOD_S, self._capture_tick)
            source_str = f"webcam index {self.camera_index}"

        self.get_logger().info(
            f"BodyTracker ready — source: {source_str} — "
            f"publishing /body_landmarks (33 pose landmarks, px coords).")

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
    #  DETECTION + PUBLISH
    # ─────────────────────────────────────────────────────────────────────────

    def _process_frame(self, frame_bgr):
        if self.flip_h:
            frame_bgr = cv2.flip(frame_bgr, 1)

        if self.frame_pub is not None:
            img_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.frame_pub.publish(img_msg)

        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        landmarks = self.pose.detect(rgb)

        if landmarks is not None:
            msg                 = PoseArray()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = "pixel"
            for lm in landmarks:
                p = Pose()
                p.position.x = float(lm.x * IMG_W)
                p.position.y = float(lm.y * IMG_H)
                p.position.z = float(lm.visibility)
                msg.poses.append(p)
            self.landmarks_pub.publish(msg)

        if self.show_preview:
            preview = frame_bgr.copy()
            if landmarks is not None:
                draw_landmarks(preview, landmarks, POSE_CONNECTIONS)
            cv2.imshow("body_tracker — MediaPipe Pose", preview)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        if self.show_preview:
            cv2.destroyAllWindows()
        self.pose.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BodyTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
