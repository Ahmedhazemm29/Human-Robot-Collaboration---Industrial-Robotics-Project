#!/usr/bin/env python3
"""
Version-agnostic MediaPipe wrappers for pose and hand landmark detection.

MediaPipe removed the legacy ``mp.solutions`` API in recent releases
(0.10.3x): a fresh ``pip install mediapipe`` only provides the new
``mp.tasks`` API, which needs downloadable .task model files. Older
installs only have ``mp.solutions``. This module hides the difference:

    from human_robot_collab.mp_compat import PoseDetector, HandDetector

    detector = PoseDetector()
    landmarks = detector.detect(rgb_frame)   # list of 33, or None

Each landmark exposes .x, .y, .z (normalised image coords) and
.visibility (0..1; hands have no visibility concept → fixed 1.0), which
is exactly what body_tracker / gesture_tracker / hand_to_collision use.

Tasks-API model files are auto-downloaded on first use to
``~/.cache/hrc_models`` (override with the HRC_MODEL_DIR env var):
    pose_landmarker_lite.task  (~5 MB)
    hand_landmarker.task       (~8 MB)
"""

import os
import pathlib
import urllib.request

import numpy as np
import mediapipe as mp

LEGACY_API = hasattr(mp, "solutions")

_MODEL_URLS = {
    "pose": ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
             "pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"),
    "hand": ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
             "hand_landmarker/float16/latest/hand_landmarker.task"),
}

# Minimal connection sets for preview drawing (work with both APIs).
POSE_CONNECTIONS = (
    (11, 12),                      # shoulders
    (11, 13), (13, 15),            # left arm
    (12, 14), (14, 16),            # right arm
    (11, 23), (12, 24), (23, 24),  # torso
)
HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),          # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),          # index
    (5, 9), (9, 10), (10, 11), (11, 12),     # middle
    (9, 13), (13, 14), (14, 15), (15, 16),   # ring
    (13, 17), (17, 18), (18, 19), (19, 20),  # pinky
    (0, 17),                                 # palm edge
)

_FRAME_STEP_MS = 33  # nominal 30 FPS timestamp step for VIDEO running mode


class Landmark:
    __slots__ = ("x", "y", "z", "visibility")

    def __init__(self, x, y, z=0.0, visibility=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.visibility = visibility


def _model_path(name):
    cache = pathlib.Path(os.environ.get(
        "HRC_MODEL_DIR", pathlib.Path.home() / ".cache" / "hrc_models"))
    cache.mkdir(parents=True, exist_ok=True)
    path = cache / f"{name}_landmarker.task"
    if not path.exists():
        url = _MODEL_URLS[name]
        print(f"[mp_compat] downloading {name} model -> {path}")
        tmp = path.with_suffix(".part")
        urllib.request.urlretrieve(url, tmp)
        tmp.rename(path)
    return str(path)


class PoseDetector:
    """One person, 33 landmarks with real visibility scores."""

    def __init__(self):
        if LEGACY_API:
            self._impl = mp.solutions.pose.Pose(
                model_complexity=1,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5)
        else:
            from mediapipe.tasks.python import BaseOptions, vision
            options = vision.PoseLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=_model_path("pose")),
                running_mode=vision.RunningMode.VIDEO,
                num_poses=1)
            self._impl = vision.PoseLandmarker.create_from_options(options)
            self._ts_ms = 0

    def detect(self, rgb):
        """rgb: HxWx3 uint8. Returns list of 33 Landmark or None."""
        if LEGACY_API:
            rgb.flags.writeable = False
            res = self._impl.process(rgb)
            if res.pose_landmarks is None:
                return None
            return [Landmark(lm.x, lm.y, lm.z, lm.visibility)
                    for lm in res.pose_landmarks.landmark]

        image = mp.Image(image_format=mp.ImageFormat.SRGB,
                         data=np.ascontiguousarray(rgb))
        self._ts_ms += _FRAME_STEP_MS
        res = self._impl.detect_for_video(image, self._ts_ms)
        if not res.pose_landmarks:
            return None
        return [Landmark(lm.x, lm.y, lm.z, lm.visibility)
                for lm in res.pose_landmarks[0]]

    def close(self):
        self._impl.close()


class HandDetector:
    """One hand, 21 landmarks (visibility fixed at 1.0 — the hand model
    has no per-landmark visibility; absence of a hand returns None)."""

    def __init__(self):
        if LEGACY_API:
            self._impl = mp.solutions.hands.Hands(
                max_num_hands=1,
                model_complexity=0,
                min_detection_confidence=0.6,
                min_tracking_confidence=0.5)
        else:
            from mediapipe.tasks.python import BaseOptions, vision
            options = vision.HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=_model_path("hand")),
                running_mode=vision.RunningMode.VIDEO,
                num_hands=1,
                min_hand_detection_confidence=0.6,
                min_tracking_confidence=0.5)
            self._impl = vision.HandLandmarker.create_from_options(options)
            self._ts_ms = 0

    def detect(self, rgb):
        """rgb: HxWx3 uint8. Returns list of 21 Landmark or None."""
        if LEGACY_API:
            rgb.flags.writeable = False
            res = self._impl.process(rgb)
            if not res.multi_hand_landmarks:
                return None
            return [Landmark(lm.x, lm.y, lm.z)
                    for lm in res.multi_hand_landmarks[0].landmark]

        image = mp.Image(image_format=mp.ImageFormat.SRGB,
                         data=np.ascontiguousarray(rgb))
        self._ts_ms += _FRAME_STEP_MS
        res = self._impl.detect_for_video(image, self._ts_ms)
        if not res.hand_landmarks:
            return None
        return [Landmark(lm.x, lm.y, lm.z) for lm in res.hand_landmarks[0]]

    def close(self):
        self._impl.close()


def draw_landmarks(frame_bgr, landmarks, connections,
                   colour=(0, 255, 0), joint_colour=(0, 0, 255)):
    """API-independent skeleton overlay for preview windows."""
    import cv2
    h, w = frame_bgr.shape[:2]
    pts = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
    for a, b in connections:
        if a < len(pts) and b < len(pts):
            cv2.line(frame_bgr, pts[a], pts[b], colour, 2)
    for p in pts:
        cv2.circle(frame_bgr, p, 3, joint_colour, -1)
