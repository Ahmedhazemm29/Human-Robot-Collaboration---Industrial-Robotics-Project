#!/usr/bin/env python3
"""
Logic tests for the body-collision mode and gesture classification.

These tests exercise the pure-logic parts of the perception package —
gesture geometry, quaternion alignment, pixel→world mapping, body model
assembly, and the atomic planning-scene diff — WITHOUT requiring a ROS2
installation. Any ROS module that is not importable is replaced by a
minimal stub before the package modules are loaded, so the suite runs:

    python test_feature_logic.py          # plain script, any OS
    pytest test_feature_logic.py          # or under pytest/colcon on the robot PC

On a machine with ROS2 sourced, the real message classes are used.
"""

import math
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
#  STUB MISSING ROS / VISION MODULES (no-ops where ROS2 is not installed)
# ─────────────────────────────────────────────────────────────────────────────


def _ensure_module(name):
    if name in sys.modules:
        return sys.modules[name]
    mod = types.ModuleType(name)
    sys.modules[name] = mod
    return mod


def _try_import(name):
    try:
        __import__(name)
        return True
    except Exception:
        return False


class _KwInit:
    """Base for message stubs: keyword-args constructor with class defaults."""
    _defaults = {}

    def __init__(self, **kwargs):
        for k, v in self._defaults.items():
            setattr(self, k, v() if callable(v) else v)
        for k, v in kwargs.items():
            setattr(self, k, v)


def _install_stubs():
    # rclpy ----------------------------------------------------------------
    if not _try_import("rclpy"):
        rclpy = _ensure_module("rclpy")
        node_mod = _ensure_module("rclpy.node")

        class Node:
            def __init__(self, *_a, **_k):
                pass

        node_mod.Node = Node
        rclpy.node = node_mod

        qos = _ensure_module("rclpy.qos")
        qos.QoSProfile = lambda **_k: SimpleNamespace(**_k)
        qos.ReliabilityPolicy = SimpleNamespace(RELIABLE=1, BEST_EFFORT=2)
        qos.DurabilityPolicy = SimpleNamespace(VOLATILE=1, TRANSIENT_LOCAL=2)
        rclpy.qos = qos

        time_mod = _ensure_module("rclpy.time")

        class _Time:
            def to_msg(self):
                return SimpleNamespace(sec=0, nanosec=0)

        time_mod.Time = _Time
        rclpy.time = time_mod

        dur_mod = _ensure_module("rclpy.duration")
        dur_mod.Duration = lambda **_k: SimpleNamespace(**_k)
        rclpy.duration = dur_mod

    # cv_bridge / tf2 / cv2 / mediapipe -------------------------------------
    if not _try_import("cv_bridge"):
        _ensure_module("cv_bridge").CvBridge = lambda: SimpleNamespace()

    if not _try_import("tf2_ros"):
        tf2 = _ensure_module("tf2_ros")

        class _TfErr(Exception):
            pass

        tf2.Buffer = lambda *a, **k: SimpleNamespace()
        tf2.TransformListener = lambda *a, **k: SimpleNamespace()
        tf2.LookupException = _TfErr
        tf2.ExtrapolationException = _TfErr

    if not _try_import("tf2_geometry_msgs"):
        _ensure_module("tf2_geometry_msgs")

    if not _try_import("cv2"):
        _ensure_module("cv2")

    if not _try_import("mediapipe"):
        _ensure_module("mediapipe")

    # geometry_msgs ----------------------------------------------------------
    if not _try_import("geometry_msgs.msg"):
        _ensure_module("geometry_msgs")
        gm = _ensure_module("geometry_msgs.msg")

        class Point(_KwInit):
            _defaults = {"x": 0.0, "y": 0.0, "z": 0.0}

        class Quaternion(_KwInit):
            _defaults = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}

        class Pose(_KwInit):
            _defaults = {"position": Point, "orientation": Quaternion}

        class PoseArray(_KwInit):
            _defaults = {"header": lambda: SimpleNamespace(), "poses": list}

        class PointStamped(_KwInit):
            _defaults = {"header": lambda: SimpleNamespace(frame_id="", stamp=None),
                         "point": Point}

        class PolygonStamped(_KwInit):
            _defaults = {"header": lambda: SimpleNamespace(),
                         "polygon": lambda: SimpleNamespace(points=[])}

        gm.Point = Point
        gm.Quaternion = Quaternion
        gm.Pose = Pose
        gm.PoseArray = PoseArray
        gm.PointStamped = PointStamped
        gm.PolygonStamped = PolygonStamped

    # sensor_msgs / shape_msgs / moveit_msgs / std_msgs -----------------------
    if not _try_import("sensor_msgs.msg"):
        _ensure_module("sensor_msgs")
        sm = _ensure_module("sensor_msgs.msg")

        class Image(_KwInit):
            _defaults = {"header": lambda: SimpleNamespace()}

        sm.Image = Image

    if not _try_import("shape_msgs.msg"):
        _ensure_module("shape_msgs")
        shm = _ensure_module("shape_msgs.msg")

        class SolidPrimitive(_KwInit):
            BOX = 1
            SPHERE = 2
            CYLINDER = 3
            CONE = 4
            _defaults = {"type": 0, "dimensions": list}

        shm.SolidPrimitive = SolidPrimitive

    if not _try_import("moveit_msgs.msg"):
        _ensure_module("moveit_msgs")
        mm = _ensure_module("moveit_msgs.msg")

        class CollisionObject(_KwInit):
            ADD = 0
            REMOVE = 1
            APPEND = 2
            MOVE = 3
            _defaults = {"id": "", "operation": 0, "header": lambda: SimpleNamespace(),
                         "primitives": list, "primitive_poses": list}

        class PlanningScene(_KwInit):
            _defaults = {"is_diff": False,
                         "world": lambda: SimpleNamespace(collision_objects=[])}

        mm.CollisionObject = CollisionObject
        mm.PlanningScene = PlanningScene

    if not _try_import("std_msgs.msg"):
        _ensure_module("std_msgs")
        stm = _ensure_module("std_msgs.msg")

        class Header(_KwInit):
            _defaults = {"frame_id": "", "stamp": None}

        class Bool(_KwInit):
            _defaults = {"data": False}

        class String(_KwInit):
            _defaults = {"data": ""}

        stm.Header = Header
        stm.Bool = Bool
        stm.String = String


_install_stubs()

# Make the package importable when running straight from the source tree.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from human_robot_collab import hand_to_collision as htc          # noqa: E402
from human_robot_collab.gesture_tracker import classify_gesture  # noqa: E402
from geometry_msgs.msg import Pose, PoseArray                    # noqa: E402


# ─────────────────────────────────────────────────────────────────────────────
#  TEST HELPERS
# ─────────────────────────────────────────────────────────────────────────────


def _hand_landmarks(tip_over_pip_ratio):
    """21 synthetic hand landmarks. Each finger's TIP sits at
    ratio * (PIP distance from wrist) along the same ray from the wrist."""
    lms = [SimpleNamespace(x=0.5, y=0.5) for _ in range(21)]
    wrist = np.array([0.5, 0.9])
    lms[0] = SimpleNamespace(x=float(wrist[0]), y=float(wrist[1]))
    pip_dist = 0.15
    # one direction per finger, fanning upward from the wrist
    angles = {  # finger → angle from vertical
        (6, 8): -0.3, (10, 12): -0.1, (14, 16): 0.1, (18, 20): 0.3,
    }
    for (pip, tip), ang in angles.items():
        direction = np.array([math.sin(ang), -math.cos(ang)])
        p_pip = wrist + direction * pip_dist
        p_tip = wrist + direction * pip_dist * tip_over_pip_ratio
        lms[pip] = SimpleNamespace(x=float(p_pip[0]), y=float(p_pip[1]))
        lms[tip] = SimpleNamespace(x=float(p_tip[0]), y=float(p_tip[1]))
    return lms


def _quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _bare_node():
    """HandToCollision instance without running Node.__init__ (no ROS)."""
    obj = htc.HandToCollision.__new__(htc.HandToCollision)
    obj.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace()))
    obj.get_logger = lambda: SimpleNamespace(
        warn=lambda *a, **k: None, info=lambda *a, **k: None)
    return obj


def _body_msg(visible=frozenset(range(33)), px=None):
    """PoseArray of 33 landmarks. position.z carries visibility."""
    default_px = {
        htc.MP_NOSE: (320, 60),
        htc.MP_L_SHOULDER: (260, 140), htc.MP_R_SHOULDER: (380, 140),
        htc.MP_L_ELBOW: (220, 240), htc.MP_R_ELBOW: (420, 240),
        htc.MP_L_WRIST: (200, 340), htc.MP_R_WRIST: (440, 340),
    }
    if px:
        default_px.update(px)
    msg = PoseArray()
    for i in range(33):
        u, v = default_px.get(i, (320, 240))
        p = Pose()
        p.position.x = float(u)
        p.position.y = float(v)
        p.position.z = 0.95 if i in visible else 0.05
        msg.poses.append(p)
    return msg


class _PubRecorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


# ─────────────────────────────────────────────────────────────────────────────
#  GESTURE CLASSIFICATION
# ─────────────────────────────────────────────────────────────────────────────


def test_open_palm_detected():
    assert classify_gesture(_hand_landmarks(1.7)) == "open_palm"


def test_fist_detected():
    assert classify_gesture(_hand_landmarks(0.6)) == "fist"


def test_ambiguous_hand_is_none():
    # tip ≈ pip distance — inside the dead band between the margins
    assert classify_gesture(_hand_landmarks(1.02)) == "none"


def test_partial_hand_is_none():
    # three fingers extended, one curled — neither palm nor fist
    lms = _hand_landmarks(1.7)
    curled = _hand_landmarks(0.6)
    lms[18], lms[20] = curled[18], curled[20]   # curl the pinky
    assert classify_gesture(lms) == "none"


# ─────────────────────────────────────────────────────────────────────────────
#  QUATERNION ALIGNMENT
# ─────────────────────────────────────────────────────────────────────────────


def test_quat_align_z_various_directions():
    z = np.array([0.0, 0.0, 1.0])
    for d in [(1, 0, 0), (0, 1, 0), (0, 0, 1), (-1, 0, 0),
              (0.3, -0.5, 0.8), (0.7, 0.7, 0.0)]:
        d = np.array(d, dtype=float)
        d /= np.linalg.norm(d)
        q = htc.HandToCollision._quat_align_z(d)
        rotated = _quat_to_matrix(q) @ z
        assert np.allclose(rotated, d, atol=1e-6), f"failed for {d}"


def test_quat_align_z_antiparallel():
    q = htc.HandToCollision._quat_align_z(np.array([0.0, 0.0, -1.0]))
    rotated = _quat_to_matrix(q) @ np.array([0.0, 0.0, 1.0])
    assert np.allclose(rotated, [0.0, 0.0, -1.0], atol=1e-6)


# ─────────────────────────────────────────────────────────────────────────────
#  PIXEL → WORLD (webcam mapping)
# ─────────────────────────────────────────────────────────────────────────────


def test_webcam_mapping_centre_and_bounds():
    obj = _bare_node()
    centre = obj._pixel_to_world_webcam(htc.IMG_W // 2, htc.IMG_H // 2)
    assert abs(centre[0]) < 0.01 and abs(centre[1]) < 0.01
    assert abs(centre[2] - (htc.TABLE_Z_SURFACE + 0.30)) < 0.01

    for u, v in [(0, 0), (htc.IMG_W - 1, htc.IMG_H - 1),
                 (0, htc.IMG_H - 1), (htc.IMG_W - 1, 0)]:
        p = obj._pixel_to_world_webcam(u, v)
        assert htc.TABLE_X_MIN <= p[0] <= htc.TABLE_X_MAX
        assert htc.TABLE_Y_MIN <= p[1] <= htc.TABLE_Y_MAX
        assert p[2] >= htc.TABLE_Z_SURFACE   # never below the table


# ─────────────────────────────────────────────────────────────────────────────
#  PRIMITIVE BUILDERS
# ─────────────────────────────────────────────────────────────────────────────


def test_cylinder_geometry():
    obj = _bare_node()
    p_a, p_b = np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.3])
    co = obj._make_cylinder("seg", p_a, p_b, 0.06)
    assert co is not None and co.id == "seg"
    height, radius = co.primitives[0].dimensions
    assert abs(height - (0.3 + 2 * 0.06)) < 1e-6   # padded by r each end
    assert abs(radius - 0.06) < 1e-9
    pos = co.primitive_poses[0].position
    assert (abs(pos.x) < 1e-9 and abs(pos.y) < 1e-9
            and abs(pos.z - 1.15) < 1e-9)
    # vertical segment → identity orientation
    assert abs(co.primitive_poses[0].orientation.w - 1.0) < 1e-6


def test_cylinder_degenerate_returns_none():
    obj = _bare_node()
    p = np.array([0.1, 0.2, 0.9])
    assert obj._make_cylinder("seg", p, p + 1e-4, 0.06) is None


def test_sphere_geometry():
    obj = _bare_node()
    co = obj._make_sphere("ball", np.array([0.1, -0.2, 0.9]), 0.10)
    assert co.id == "ball"
    assert co.primitives[0].type == htc.SolidPrimitive.SPHERE
    assert abs(co.primitives[0].dimensions[0] - 0.10) < 1e-9


# ─────────────────────────────────────────────────────────────────────────────
#  BODY MODEL ASSEMBLY (webcam mode)
# ─────────────────────────────────────────────────────────────────────────────


def test_full_body_builds_seven_parts(monkeypatch=None):
    old = htc.WEBCAM_MODE
    htc.WEBCAM_MODE = True
    try:
        obj = _bare_node()
        cos = obj._build_body_objects(_body_msg())
        ids = sorted(c.id for c in cos)
        assert ids == sorted(htc.ALL_BODY_IDS), f"got {ids}"
    finally:
        htc.WEBCAM_MODE = old


def test_hidden_arm_is_skipped():
    old = htc.WEBCAM_MODE
    htc.WEBCAM_MODE = True
    try:
        obj = _bare_node()
        visible = set(range(33)) - {htc.MP_L_ELBOW, htc.MP_L_WRIST}
        cos = obj._build_body_objects(_body_msg(visible=frozenset(visible)))
        ids = {c.id for c in cos}
        assert ids == {"body_r_upper_arm", "body_r_forearm",
                       "body_r_hand", "body_head"}, f"got {ids}"
    finally:
        htc.WEBCAM_MODE = old


def test_short_landmark_message_rejected():
    obj = _bare_node()
    msg = PoseArray()
    msg.poses = [Pose() for _ in range(10)]
    assert obj._build_body_objects(msg) == []


# ─────────────────────────────────────────────────────────────────────────────
#  ATOMIC SCENE DIFF
# ─────────────────────────────────────────────────────────────────────────────


def _co(obj_id):
    c = htc.CollisionObject()
    c.id = obj_id
    c.operation = htc.CollisionObject.ADD
    return c


def test_publish_objects_adds_and_removes_atomically():
    obj = _bare_node()
    obj.scene_pub = _PubRecorder()
    obj._active_ids = {"a", "b"}

    obj._publish_objects([_co("a"), _co("c")])

    assert len(obj.scene_pub.messages) == 1
    scene = obj.scene_pub.messages[0]
    assert scene.is_diff is True
    ops = {c.id: c.operation for c in scene.world.collision_objects}
    assert ops == {"a": htc.CollisionObject.ADD,
                   "c": htc.CollisionObject.ADD,
                   "b": htc.CollisionObject.REMOVE}
    assert obj._active_ids == {"a", "c"}


def test_publish_remove_only_clears_everything():
    obj = _bare_node()
    obj.scene_pub = _PubRecorder()
    obj._active_ids = {"x", "y", "z"}

    obj._publish_remove_only()

    scene = obj.scene_pub.messages[0]
    assert all(c.operation == htc.CollisionObject.REMOVE
               for c in scene.world.collision_objects)
    assert {c.id for c in scene.world.collision_objects} == {"x", "y", "z"}
    assert obj._active_ids == set()


# ─────────────────────────────────────────────────────────────────────────────
#  GESTURE PAUSE/RESUME STATE MACHINE (through the node's real frame path)
# ─────────────────────────────────────────────────────────────────────────────


def _bare_gesture_node(detect_results):
    """GestureTracker instance without ROS; detector yields canned results."""
    from collections import deque
    from human_robot_collab import gesture_tracker as gt

    obj = gt.GestureTracker.__new__(gt.GestureTracker)
    obj.flip_h = False
    obj.show_preview = False
    obj.paused = False
    obj._stable = "none"
    obj._window = deque(maxlen=gt.GESTURE_HOLD_FRAMES)
    obj.pause_pub = _PubRecorder()
    obj.cmd_pub = _PubRecorder()
    obj.get_logger = lambda: SimpleNamespace(
        warn=lambda *a, **k: None, info=lambda *a, **k: None)
    it = iter(detect_results)
    obj.hands = SimpleNamespace(detect=lambda _rgb: next(it), close=lambda: None)
    return obj, gt


def test_gesture_state_machine_debounce_and_latch():
    import cv2
    if not hasattr(cv2, "cvtColor"):   # stubbed cv2 → skip (needs real OpenCV)
        return

    from human_robot_collab import gesture_tracker as gt
    hold = gt.GESTURE_HOLD_FRAMES
    palm = _hand_landmarks(1.7)
    fist = _hand_landmarks(0.6)

    # palm held → pause; brief noise → no change; fist held → resume
    seq = ([palm] * hold) + [None, fist] + ([fist] * hold) + [None]
    obj, _ = _bare_gesture_node(seq)
    frame = np.zeros((48, 64, 3), dtype=np.uint8)

    for i in range(hold):
        obj._process_frame(frame)
    assert obj.paused is True, "open palm held for HOLD frames must pause"
    assert obj.pause_pub.messages[-1].data is True
    assert obj.cmd_pub.messages[-1].data == "open_palm"

    obj._process_frame(frame)   # None (hand lost) — breaks the window
    obj._process_frame(frame)   # first fist frame
    assert obj.paused is True, "a single fist frame must NOT resume yet"

    for _ in range(hold):
        obj._process_frame(frame)
    assert obj.paused is False, "fist held for HOLD frames must resume"
    assert obj.pause_pub.messages[-1].data is False

    obj._process_frame(frame)   # trailing None — no state change
    assert obj.paused is False


def test_gesture_apply_is_idempotent():
    obj, _ = _bare_gesture_node([])
    obj._apply_gesture("fist")          # resume while not paused → no publish
    assert obj.pause_pub.messages == []
    obj._apply_gesture("open_palm")     # pause
    obj._apply_gesture("open_palm")     # already paused → no second publish
    assert [m.data for m in obj.pause_pub.messages] == [True]
    obj._apply_gesture("none")          # never changes state
    assert obj.paused is True


# ─────────────────────────────────────────────────────────────────────────────
#  PLAIN-SCRIPT RUNNER
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except AssertionError as e:
            failed += 1
            print(f"  FAIL  {name}: {e}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"  ERROR {name}: {type(e).__name__}: {e}")
    total = len(tests)
    print(f"\n{total - failed}/{total} tests passed")
    sys.exit(1 if failed else 0)
