#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PolygonStamped, Pose, Point, Quaternion, PointStamped
from sensor_msgs.msg import Image
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from std_msgs.msg import Header

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped transform support

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

BASE_FRAME     = "world"
CAMERA_FRAME   = "kinect_rgb_optical_frame"
COLLISION_ID   = "hand_exclusion_zone"
DEPTH_PADDING  = 0.0
BOX_MARGIN_M   = 0.0
BOX_MIN_DIM    = 0.01
MARGIN_PX      = 0
IMG_W          = 640
IMG_H          = 480
STALE_TIMEOUT  = 0.5
DEPTH_EMA_ALPHA = 0.25   # EMA weight on new depth sample (lower = smoother)

# Kinect vertical calibration offset — set this to compensate for the
# mismatch between the URDF kinect_link Z and the real mounting height.
# Procedure: put hand flat on table, read Z_observed from the log,
# set WORLD_Z_OFFSET_M = 1.0 - Z_observed  (negative = box was too high).
WORLD_Z_OFFSET_M = 0.0

# ─────────────────────────────────────────────────────────────────────────────
#  MODE SWITCH
#  WEBCAM_MODE = True  → webcam-only fallback (no depth, crude pixel mapping)
#  WEBCAM_MODE = False → Kinect: depth + TF → accurate 3-D world bounding box
# ─────────────────────────────────────────────────────────────────────────────
WEBCAM_MODE   = False
FIXED_DEPTH_M = 0.9

# ─────────────────────────────────────────────────────────────────────────────
#  TRACKING MODE (ROS parameter, not a constant — set at launch)
#    ros2 run human_robot_collab hand_to_collision                       → hand
#    ros2 run human_robot_collab hand_to_collision --ros-args -p tracking_mode:=body
#
#  "hand" → original pipeline: /hand_bbox (C++ MediaPipe node) → single box
#  "body" → /body_landmarks (body_tracker.py, MediaPipe Pose) → multi-part
#           model: cylinder per arm segment + sphere per hand + head sphere
# ─────────────────────────────────────────────────────────────────────────────

# MediaPipe Pose landmark indices used in body mode
MP_NOSE       = 0
MP_L_SHOULDER = 11
MP_R_SHOULDER = 12
MP_L_ELBOW    = 13
MP_R_ELBOW    = 14
MP_L_WRIST    = 15
MP_R_WRIST    = 16

VIS_THRESH    = 0.5    # min MediaPipe visibility to trust a landmark
UPPER_ARM_R   = 0.07   # m — cylinder radius shoulder→elbow
FOREARM_R     = 0.06   # m — cylinder radius elbow→wrist
HAND_R        = 0.10   # m — sphere radius at wrist (covers the hand)
HEAD_R        = 0.14   # m — sphere radius at nose
MIN_SEG_LEN   = 0.02   # m — skip degenerate segments shorter than this
LM_PATCH_PX   = 8      # px — half-size of depth patch sampled per landmark

# (segment_id, landmark_a, landmark_b, radius)
BODY_SEGMENTS = [
    ("body_l_upper_arm", MP_L_SHOULDER, MP_L_ELBOW, UPPER_ARM_R),
    ("body_l_forearm",   MP_L_ELBOW,    MP_L_WRIST, FOREARM_R),
    ("body_r_upper_arm", MP_R_SHOULDER, MP_R_ELBOW, UPPER_ARM_R),
    ("body_r_forearm",   MP_R_ELBOW,    MP_R_WRIST, FOREARM_R),
]
# (sphere_id, landmark, radius)
BODY_SPHERES = [
    ("body_l_hand", MP_L_WRIST, HAND_R),
    ("body_r_hand", MP_R_WRIST, HAND_R),
    ("body_head",   MP_NOSE,    HEAD_R),
]
ALL_BODY_IDS = [s[0] for s in BODY_SEGMENTS] + [s[0] for s in BODY_SPHERES]

# ─────────────────────────────────────────────────────────────────────────────
#  KINECT v1 FIXED INTRINSICS
#  Standard Kinect v1 factory values — no CameraInfo topic needed
# ─────────────────────────────────────────────────────────────────────────────
KINECT_FX = 525.0
KINECT_FY = 525.0
KINECT_CX = 319.5
KINECT_CY = 239.5

# ─────────────────────────────────────────────────────────────────────────────
#  TABLE BOUNDARIES IN WORLD FRAME
#  Derived from table.xacro:
#    Visual geometry : size="1.4 x 0.71 x 0.71",  origin xyz="0 0 0.375"
#    Joint           : xyz="0 0 0.7"  rpy="pi 0 pi/2"
#
#  Effect of rpy="pi 0 pi/2":
#    pi   around X → flips local Z,  table centre Z = 0.7 - 0.375 = 0.325 m
#    pi/2 around Z → swaps X↔Y axes
#      local X (1.4 m) → world Y  →  Y ∈ [-0.70,  +0.70]
#      local Y (0.71 m) → world X  →  X ∈ [-0.355, +0.355]
#
#  TABLE_Z_SURFACE = table centre Z + half-height = 0.325 + 0.355 = 0.68 m
#  The collision box may slide freely in XY within these bounds and move
#  up/down in Z, but its bottom face is floored at TABLE_Z_SURFACE so it
#  can never penetrate the table.
# ─────────────────────────────────────────────────────────────────────────────
TABLE_X_MIN     = -0.355
TABLE_X_MAX     =  0.355
TABLE_Y_MIN     = -0.70
TABLE_Y_MAX     =  0.70
TABLE_Z_SURFACE =  0.68   # m — top face of table in world frame

# ─────────────────────────────────────────────────────────────────────────────
#  HAND-BOX SIZING (webcam mode)
#  Width (X) and height (Z) are derived from the MediaPipe pixel bbox.
#  Depth (Y) cannot be recovered from a single 2-D image, so it is fixed.
# ─────────────────────────────────────────────────────────────────────────────
HAND_BASE_H  = 0.22   # m  — hand depth (Y, into scene) — fixed, not observable


class HandToCollision(Node):

    def __init__(self):
        super().__init__("hand_to_collision")

        self.declare_parameter("tracking_mode", "hand")
        self.mode = self.get_parameter("tracking_mode") \
                        .get_parameter_value().string_value.strip().lower()
        if self.mode not in ("hand", "body"):
            self.get_logger().warn(
                f"Unknown tracking_mode '{self.mode}' — falling back to 'hand'.")
            self.mode = "hand"

        self.bridge             = CvBridge()
        self.latest_landmarks   = None
        self.last_landmark_time = None
        self.latest_depth       = None

        self._z_smooth   = None   # EMA-smoothed depth value (hand mode)
        self._lm_z_smooth = {}    # per-landmark EMA depth (body mode)
        # IDs currently present in the planning scene. Seeded with every ID we
        # could ever publish so the first update atomically clears leftovers
        # from a previous run (e.g. hand box lingering after a mode switch).
        self._active_ids = set([COLLISION_ID] + ALL_BODY_IDS)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        if self.mode == "body":
            self.create_subscription(
                PoseArray,
                "/body_landmarks",
                self._landmarks_cb,
                reliable_qos
            )
        else:
            self.create_subscription(
                PolygonStamped,
                "/hand_bbox",
                self._bbox_cb,
                reliable_qos
            )

        if not WEBCAM_MODE:
            # ── Subscribe to Kinect depth topic (published by kinect_cpp) ────
            # Topic: /depth/image_raw  encoding: 32FC1 (metres)
            self.create_subscription(
                Image,
                "/depth/image_raw",
                self._depth_cb,
                10
            )

        self.scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )

        self.create_timer(0.05, self._update_scene)  # 20 Hz

        mode_str = "WEBCAM (fixed depth, no TF)" if WEBCAM_MODE else "KINECT (depth stream + TF)"
        self.get_logger().info(
            f"HandToCollision node ready — camera: {mode_str} — "
            f"tracking: {self.mode.upper()} — waiting for landmarks.")

    # ─────────────────────────────────────────────────────────────────────────
    #  SUBSCRIBERS
    # ─────────────────────────────────────────────────────────────────────────

    def _landmarks_cb(self, msg: PoseArray):
        self.latest_landmarks   = msg
        self.last_landmark_time = self.get_clock().now()

    def _bbox_cb(self, msg: PolygonStamped):
        from geometry_msgs.msg import PoseArray, Pose
        fake = PoseArray()
        fake.header = msg.header
        for pt in msg.polygon.points:
            p = Pose()
            p.position.x = pt.x
            p.position.y = pt.y
            p.position.z = pt.z
            fake.poses.append(p)
        self.latest_landmarks   = fake
        self.last_landmark_time = self.get_clock().now()

    def _depth_cb(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if depth.dtype == np.uint16:
            # 16UC1: millimetres → metres (standard for ROS2 depth cameras)
            self.latest_depth = depth.astype(np.float32) / 1000.0
        else:
            # 32FC1: already in metres
            self.latest_depth = depth.astype(np.float32)

    # ─────────────────────────────────────────────────────────────────────────
    #  10 Hz TIMER
    # ─────────────────────────────────────────────────────────────────────────

    def _update_scene(self):
        now = self.get_clock().now()

        if self.last_landmark_time is not None:
            age = (now - self.last_landmark_time).nanoseconds / 1e9
            if age > STALE_TIMEOUT:
                self._publish_remove_only()
                self.get_logger().info(
                    "Target lost — collision objects removed.",
                    throttle_duration_sec=1.0)
                return

        if self.latest_landmarks is None:
            return

        if not WEBCAM_MODE:
            if self.latest_depth is None:
                self.get_logger().warn(
                    "Waiting for Kinect depth image...",
                    throttle_duration_sec=2.0)
                return

        # ── BODY MODE ─────────────────────────────────────────────────────
        if self.mode == "body":
            cos = self._build_body_objects(self.latest_landmarks)
            if not cos:
                self._publish_remove_only()
                return
            self._publish_objects(cos)
            self.get_logger().info(
                f"Body model updated: {len(cos)} parts "
                f"[{', '.join(c.id.replace('body_', '') for c in cos)}]",
                throttle_duration_sec=0.5)
            return

        # ── HAND MODE (original pipeline) ─────────────────────────────────
        co = self._build_collision_object(self.latest_landmarks)
        if co is None:
            self._publish_remove_only()
            return

        self._publish_objects([co])

        self.get_logger().info(
            f"Collision box updated: centre=("
            f"{co.primitive_poses[0].position.x:.2f}, "
            f"{co.primitive_poses[0].position.y:.2f}, "
            f"{co.primitive_poses[0].position.z:.2f}) m  "
            f"size=({co.primitives[0].dimensions[0]:.2f} x "
            f"{co.primitives[0].dimensions[1]:.2f} x "
            f"{co.primitives[0].dimensions[2]:.2f}) m",
            throttle_duration_sec=0.5
        )

    # ─────────────────────────────────────────────────────────────────────────
    #  COLLISION OBJECT BUILDER
    # ─────────────────────────────────────────────────────────────────────────

    def _build_collision_object(self, msg: PoseArray):
        xs = [pose.position.x for pose in msg.poses]
        ys = [pose.position.y for pose in msg.poses]

        u_min = int(max(0,       min(xs) - MARGIN_PX))
        u_max = int(min(IMG_W-1, max(xs) + MARGIN_PX))
        v_min = int(max(0,       min(ys) - MARGIN_PX))
        v_max = int(min(IMG_H-1, max(ys) + MARGIN_PX))

        u_c = int((u_min + u_max) / 2)
        v_c = int((v_min + v_max) / 2)

        # ── WEBCAM MODE ───────────────────────────────────────────────────
        if WEBCAM_MODE:
            Z_RANGE = 0.60
            # Derive world-space size from the actual MediaPipe pixel bbox.
            box_w = float((u_max - u_min) * (TABLE_X_MAX - TABLE_X_MIN) / IMG_W)
            box_d = float((v_max - v_min) * Z_RANGE / IMG_H)
            box_h = HAND_BASE_H  # Y depth fixed — not observable from 2-D image

            # Map pixel centre -> table XY footprint, clamped to table edges.
            x_centre = TABLE_X_MIN + (u_c / IMG_W) * (TABLE_X_MAX - TABLE_X_MIN)
            y_centre = TABLE_Y_MIN + (v_c / IMG_H) * (TABLE_Y_MAX - TABLE_Y_MIN)
            x_centre = float(np.clip(x_centre, TABLE_X_MIN, TABLE_X_MAX))
            y_centre = float(np.clip(y_centre, TABLE_Y_MIN, TABLE_Y_MAX))

            # Map vertical pixel centre -> Z above table surface.
            # v=0 (top of frame)   = arm raised (TABLE_Z_SURFACE + Z_RANGE)
            # v=IMG_H (bottom)     = hand on table (TABLE_Z_SURFACE)
            z_raw    = TABLE_Z_SURFACE + (1.0 - v_c / IMG_H) * Z_RANGE
            z_centre = float(max(z_raw, TABLE_Z_SURFACE + box_d / 2.0))

            co                 = CollisionObject()
            co.header          = Header()
            co.header.frame_id = "world"
            co.header.stamp    = self.get_clock().now().to_msg()
            co.id              = COLLISION_ID
            co.operation       = CollisionObject.ADD

            box_prim            = SolidPrimitive()
            box_prim.type       = SolidPrimitive.BOX
            box_prim.dimensions = [box_w, box_h, box_d]

            pose             = Pose()
            pose.position    = Point(x=x_centre, y=y_centre, z=z_centre)
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            co.primitives      = [box_prim]
            co.primitive_poses = [pose]
            return co

        # ── KINECT MODE ───────────────────────────────────────────────────
        # Sample depth over the full hand bbox and take the 10th percentile
        # (the closest valid surface = the hand, not the table behind it).
        patch = self.latest_depth[
            max(0, v_min):min(IMG_H, v_max + 1),
            max(0, u_min):min(IMG_W, u_max + 1)
        ]
        valid = patch[patch > 0.1]
        if valid.size == 0:
            self.get_logger().warn(
                "No valid depth in hand bbox — skipping frame.",
                throttle_duration_sec=1.0)
            return None
        Z_raw = float(np.percentile(valid, 10))

        # EMA smoothing: damps per-frame depth noise / sudden jumps.
        if self._z_smooth is None:
            self._z_smooth = Z_raw
        else:
            self._z_smooth = DEPTH_EMA_ALPHA * Z_raw + (1.0 - DEPTH_EMA_ALPHA) * self._z_smooth
        Z = self._z_smooth

        self.get_logger().info(
            f"Depth at hand centre: {Z:.3f} m (raw {Z_raw:.3f})  bbox_px=({u_min},{v_min})->({u_max},{v_max})",
            throttle_duration_sec=0.5)

        # Deproject 5 pixel positions to 3-D in the optical frame.
        # Optical convention: Z forward (depth), X right, Y down.
        def deproject(u, v, z):
            return np.array([(u - KINECT_CX) * z / KINECT_FX,
                             (v - KINECT_CY) * z / KINECT_FY,
                             z], dtype=float)

        cam_pts = [
            deproject(u_c,   v_c,   Z),  # centre
            deproject(u_min, v_min, Z),  # top-left
            deproject(u_max, v_min, Z),  # top-right
            deproject(u_max, v_max, Z),  # bottom-right
            deproject(u_min, v_max, Z),  # bottom-left
        ]

        # Use Time() = latest available transform — correct for static frames.
        def to_world(xyz_cam):
            pt               = PointStamped()
            pt.header.frame_id = CAMERA_FRAME
            pt.header.stamp  = rclpy.time.Time().to_msg()
            pt.point.x       = float(xyz_cam[0])
            pt.point.y       = float(xyz_cam[1])
            pt.point.z       = float(xyz_cam[2])
            return self.tf_buffer.transform(
                pt, "world",
                timeout=rclpy.duration.Duration(seconds=0.1))

        try:
            world_pts = [to_world(p) for p in cam_pts]
        except tf2_ros.LookupException:
            self.get_logger().warn(
                "TF lookup failed — is the robot/camera TF tree running?",
                throttle_duration_sec=2.0)
            return None
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(
                f"TF extrapolation error: {e}", throttle_duration_sec=2.0)
            return None
        except Exception as e:
            self.get_logger().warn(
                f"TF transform failed: {e}", throttle_duration_sec=2.0)
            return None

        # Build world-frame AABB from the 5 transformed points.
        xs = [p.point.x for p in world_pts]
        ys = [p.point.y for p in world_pts]
        zs = [p.point.z for p in world_pts]

        cx_world = float((max(xs) + min(xs)) / 2.0)
        cy_world = float((max(ys) + min(ys)) / 2.0)
        cz_world = float((max(zs) + min(zs)) / 2.0) + WORLD_Z_OFFSET_M

        box_w = max(BOX_MIN_DIM, float(max(xs) - min(xs)) + 2 * BOX_MARGIN_M)
        box_h = max(BOX_MIN_DIM, float(max(ys) - min(ys)) + 2 * BOX_MARGIN_M)
        box_d = max(BOX_MIN_DIM, float(max(zs) - min(zs)) + DEPTH_PADDING + 2 * BOX_MARGIN_M)

        co                 = CollisionObject()
        co.header          = Header()
        co.header.frame_id = "world"
        co.header.stamp    = self.get_clock().now().to_msg()
        co.id              = COLLISION_ID
        co.operation       = CollisionObject.ADD

        box_prim            = SolidPrimitive()
        box_prim.type       = SolidPrimitive.BOX
        box_prim.dimensions = [box_w, box_h, box_d]

        pose             = Pose()
        pose.position    = Point(x=cx_world, y=cy_world, z=cz_world)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        co.primitives      = [box_prim]
        co.primitive_poses = [pose]
        return co

    # ─────────────────────────────────────────────────────────────────────────
    #  BODY-MODE BUILDER
    #  Input: PoseArray of 33 MediaPipe Pose landmarks from body_tracker.py
    #         position.x = u (px, 0..IMG_W), position.y = v (px, 0..IMG_H),
    #         position.z = landmark visibility (0..1)
    #  Output: list of CollisionObjects — one cylinder per arm segment,
    #          one sphere per hand, one sphere for the head.
    # ─────────────────────────────────────────────────────────────────────────

    def _build_body_objects(self, msg: PoseArray):
        if len(msg.poses) < 33:
            self.get_logger().warn(
                f"Body landmarks message has {len(msg.poses)} poses "
                f"(expected 33) — skipping frame.",
                throttle_duration_sec=2.0)
            return []

        # Resolve each needed landmark to a world-frame point (or None).
        world_pt = {}
        for idx in {i for _, a, b, _ in BODY_SEGMENTS for i in (a, b)} | \
                   {i for _, i, _ in BODY_SPHERES}:
            p   = msg.poses[idx]
            vis = p.position.z
            if vis < VIS_THRESH:
                world_pt[idx] = None
                continue
            u = int(np.clip(p.position.x, 0, IMG_W - 1))
            v = int(np.clip(p.position.y, 0, IMG_H - 1))
            if WEBCAM_MODE:
                world_pt[idx] = self._pixel_to_world_webcam(u, v)
            else:
                world_pt[idx] = self._pixel_to_world_kinect(u, v, idx)

        cos = []
        for seg_id, ia, ib, radius in BODY_SEGMENTS:
            pa, pb = world_pt.get(ia), world_pt.get(ib)
            if pa is None or pb is None:
                continue
            co = self._make_cylinder(seg_id, pa, pb, radius)
            if co is not None:
                cos.append(co)

        for sph_id, idx, radius in BODY_SPHERES:
            pc = world_pt.get(idx)
            if pc is None:
                continue
            cos.append(self._make_sphere(sph_id, pc, radius))

        return cos

    # ─────────────────────────────────────────────────────────────────────────
    #  PIXEL → WORLD HELPERS (body mode)
    # ─────────────────────────────────────────────────────────────────────────

    def _pixel_to_world_webcam(self, u, v):
        """Same crude overhead mapping as the webcam hand box: u → world X,
        v → world Y, and v also drives height above the table surface."""
        Z_RANGE = 0.60
        x = TABLE_X_MIN + (u / IMG_W) * (TABLE_X_MAX - TABLE_X_MIN)
        y = TABLE_Y_MIN + (v / IMG_H) * (TABLE_Y_MAX - TABLE_Y_MIN)
        z = TABLE_Z_SURFACE + (1.0 - v / IMG_H) * Z_RANGE
        x = float(np.clip(x, TABLE_X_MIN, TABLE_X_MAX))
        y = float(np.clip(y, TABLE_Y_MIN, TABLE_Y_MAX))
        z = float(max(z, TABLE_Z_SURFACE))
        return np.array([x, y, z])

    def _pixel_to_world_kinect(self, u, v, lm_idx):
        """Depth-sample a small patch at the landmark pixel, deproject with
        the Kinect intrinsics, TF-transform to world. Returns None if no
        valid depth or TF is available for this landmark."""
        patch = self.latest_depth[
            max(0, v - LM_PATCH_PX):min(IMG_H, v + LM_PATCH_PX + 1),
            max(0, u - LM_PATCH_PX):min(IMG_W, u + LM_PATCH_PX + 1)
        ]
        valid = patch[patch > 0.1]
        if valid.size == 0:
            return None
        z_raw = float(np.percentile(valid, 10))

        prev = self._lm_z_smooth.get(lm_idx)
        z = z_raw if prev is None else \
            DEPTH_EMA_ALPHA * z_raw + (1.0 - DEPTH_EMA_ALPHA) * prev
        self._lm_z_smooth[lm_idx] = z

        cam = np.array([(u - KINECT_CX) * z / KINECT_FX,
                        (v - KINECT_CY) * z / KINECT_FY,
                        z], dtype=float)

        pt                 = PointStamped()
        pt.header.frame_id = CAMERA_FRAME
        pt.header.stamp    = rclpy.time.Time().to_msg()
        pt.point.x, pt.point.y, pt.point.z = map(float, cam)
        try:
            w = self.tf_buffer.transform(
                pt, "world", timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(
                f"TF transform failed for landmark {lm_idx}: {e}",
                throttle_duration_sec=2.0)
            return None
        return np.array([w.point.x, w.point.y, w.point.z + WORLD_Z_OFFSET_M])

    # ─────────────────────────────────────────────────────────────────────────
    #  PRIMITIVE BUILDERS (body mode)
    # ─────────────────────────────────────────────────────────────────────────

    def _make_sphere(self, obj_id, centre, radius):
        co                 = CollisionObject()
        co.header          = Header()
        co.header.frame_id = "world"
        co.header.stamp    = self.get_clock().now().to_msg()
        co.id              = obj_id
        co.operation       = CollisionObject.ADD

        prim            = SolidPrimitive()
        prim.type       = SolidPrimitive.SPHERE
        prim.dimensions = [float(radius)]

        pose             = Pose()
        pose.position    = Point(x=float(centre[0]), y=float(centre[1]),
                                 z=float(centre[2]))
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        co.primitives      = [prim]
        co.primitive_poses = [pose]
        return co

    def _make_cylinder(self, obj_id, p_a, p_b, radius):
        """Cylinder from p_a to p_b (world frame). Height is padded by one
        radius at each end so consecutive segments overlap at the joints
        (poor man's capsule)."""
        d      = p_b - p_a
        length = float(np.linalg.norm(d))
        if length < MIN_SEG_LEN:
            return None
        axis_z = d / length

        co                 = CollisionObject()
        co.header          = Header()
        co.header.frame_id = "world"
        co.header.stamp    = self.get_clock().now().to_msg()
        co.id              = obj_id
        co.operation       = CollisionObject.ADD

        prim            = SolidPrimitive()
        prim.type       = SolidPrimitive.CYLINDER
        # SolidPrimitive cylinder: dimensions = [height, radius], axis = local Z
        prim.dimensions = [length + 2.0 * radius, float(radius)]

        centre           = (p_a + p_b) / 2.0
        pose             = Pose()
        pose.position    = Point(x=float(centre[0]), y=float(centre[1]),
                                 z=float(centre[2]))
        pose.orientation = self._quat_align_z(axis_z)

        co.primitives      = [prim]
        co.primitive_poses = [pose]
        return co

    @staticmethod
    def _quat_align_z(d):
        """Quaternion rotating the local +Z axis onto unit vector d."""
        z   = np.array([0.0, 0.0, 1.0])
        dot = float(np.dot(z, d))
        if dot > 0.99999:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        if dot < -0.99999:
            # anti-parallel: 180° about X
            return Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        axis = np.cross(z, d)
        s    = float(np.sqrt((1.0 + dot) * 2.0))
        return Quaternion(x=float(axis[0] / s),
                          y=float(axis[1] / s),
                          z=float(axis[2] / s),
                          w=s / 2.0)

    # ─────────────────────────────────────────────────────────────────────────
    #  HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_objects(self, cos):
        """Atomic scene diff: ADD the current objects and REMOVE any object
        we published before that is absent this cycle — one message, so no
        ghost parts ever linger (same trick as the original hand box swap)."""
        new_ids = {co.id for co in cos}
        removes = []
        for stale_id in self._active_ids - new_ids:
            r           = CollisionObject()
            r.id        = stale_id
            r.operation = CollisionObject.REMOVE
            removes.append(r)

        scene_msg                         = PlanningScene()
        scene_msg.is_diff                 = True
        scene_msg.world.collision_objects = list(cos) + removes
        self.scene_pub.publish(scene_msg)
        self._active_ids = new_ids

    def _publish_remove_only(self):
        ids = self._active_ids if self._active_ids else {COLLISION_ID}
        removes = []
        for obj_id in ids:
            r           = CollisionObject()
            r.id        = obj_id
            r.operation = CollisionObject.REMOVE
            removes.append(r)
        scene_msg                         = PlanningScene()
        scene_msg.is_diff                 = True
        scene_msg.world.collision_objects = removes
        self.scene_pub.publish(scene_msg)
        self._active_ids = set()


def main(args=None):
    rclpy.init(args=args)
    node = HandToCollision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Removing collision object and shutting down...")
        try:
            node._publish_remove_only()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
