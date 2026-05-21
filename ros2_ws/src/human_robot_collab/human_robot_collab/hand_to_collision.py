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

BASE_FRAME     = "base_link"
## CAMERA_FRAME   = "camera_color_optical_frame"
CAMERA_FRAME   = "kinect_link"          # ← changed from "world"
COLLISION_ID   = "hand_exclusion_zone"
DEPTH_PADDING  = 0.05
MARGIN_PX      = 10
IMG_W          = 640
IMG_H          = 480
SAFETY_PADDING = 0.05
STALE_TIMEOUT  = 0.5

# ─────────────────────────────────────────────────────────────────────────────
#  MODE SWITCH
#  WEBCAM_MODE = True  → webcam testing, no depth camera, skips TF transform
#  WEBCAM_MODE = False → Kinect, full 3D deprojection, TF transform active
# ─────────────────────────────────────────────────────────────────────────────
WEBCAM_MODE   = True                    # ← set to False to use Kinect
FIXED_DEPTH_M = 0.9

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
#  Derived from URDF: table is 1.4m x 0.7m, centered at world origin.
#  These clamp the collision box so it can never wander off the table surface.
#  X spans the long axis (1.4m total → ±0.7m)
#  Y spans the short axis (0.7m total → ±0.35m)
# ─────────────────────────────────────────────────────────────────────────────
TABLE_X_MIN = -0.7
TABLE_X_MAX =  0.7
TABLE_Y_MIN = -0.35
TABLE_Y_MAX =  0.35


class HandToCollision(Node):

    def __init__(self):
        super().__init__("hand_to_collision")

        self.bridge             = CvBridge()
        self.latest_landmarks   = None
        self.last_landmark_time = None
        self.latest_depth       = None  # ← removed camera_info, not needed for Kinect

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.create_subscription(
            PolygonStamped,
            "/hand_bbox",
            self._bbox_cb,
            reliable_qos
        )

        if not WEBCAM_MODE:
            # ── Subscribe to Kinect depth topic (published by kinect_cpp) ────
            # Topic: /kinect/depth/image_raw  encoding: 32FC1 (metres)
            self.create_subscription(
                Image,
                "/depth/image_raw",  # ← changed from /camera/depth/image_rect_raw
                self._depth_cb,
                10
            )
            # Note: No CameraInfo subscription needed — using fixed Kinect v1 intrinsics

        self.scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )

        self.create_timer(0.1, self._update_scene)

        mode_str = "WEBCAM (fixed depth, no TF)" if WEBCAM_MODE else "KINECT (depth stream + TF)"
        self.get_logger().info(
            f"HandToCollision node ready — mode: {mode_str} — waiting for landmarks.")

    # ─────────────────────────────────────────────────────────────────────────
    #  SUBSCRIBERS
    # ─────────────────────────────────────────────────────────────────────────

    def _landmarks_cb(self, msg: PoseArray):
        self.latest_landmarks   = msg
        self.last_landmark_time = self.get_clock().now()

    def _bbox_cb(self, msg: PolygonStamped):
        # Convert PolygonStamped (4 bbox corners) into a fake PoseArray
        # so _build_collision_object can consume it unchanged
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
        # kinect_cpp publishes /kinect/depth/image_raw as 32FC1 in metres
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Safety: handle both 32FC1 (metres, from kinect_cpp) and
        # raw 16UC1 (11-bit values) in case encoding changes
        if depth.dtype == np.uint16:
            # Raw 11-bit → metres using standard Kinect v1 formula
            depth = depth.astype(np.float32)
            valid_mask = depth < 2047
            depth_m = np.zeros_like(depth, dtype=np.float32)
            depth_m[valid_mask] = 1.0 / (-0.00307 * depth[valid_mask] + 3.33)
            self.latest_depth = depth_m
        else:
            # Already in metres (32FC1 from kinect_cpp)
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
                    "Hand lost — collision box removed.",
                    throttle_duration_sec=1.0)
                return

        if self.latest_landmarks is None:
            return

        if not WEBCAM_MODE:
            if self.latest_depth is None:
                self.get_logger().warn(
                    "Waiting for Kinect depth image...",  # ← updated message
                    throttle_duration_sec=2.0)
                return

        co = self._build_collision_object(self.latest_landmarks)
        if co is None:
            return

        remove_co           = CollisionObject()
        remove_co.id        = COLLISION_ID
        remove_co.operation = CollisionObject.REMOVE

        scene_msg                         = PlanningScene()
        scene_msg.is_diff                 = True
        scene_msg.world.collision_objects = [remove_co, co]
        self.scene_pub.publish(scene_msg)  # this is what triggers rviz to show the green box

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
            Z = FIXED_DEPTH_M

            # Remap normalized pixel position (0→1) to the table's actual
            # metric range in the world frame. The full webcam frame maps to
            # the full table surface — hand on the left of camera = left of table.
            x_centre = TABLE_X_MIN + (u_c / IMG_W) * (TABLE_X_MAX - TABLE_X_MIN)
            y_centre = TABLE_Y_MIN + (v_c / IMG_H) * (TABLE_Y_MAX - TABLE_Y_MIN)

            # Clamp so the box centre can never wander outside the table boundary.
            x_centre = float(np.clip(x_centre, TABLE_X_MIN, TABLE_X_MAX))
            y_centre = float(np.clip(y_centre, TABLE_Y_MIN, TABLE_Y_MAX))

            centre_cam = np.array([x_centre, y_centre, Z])

            # Apply the same mapping to corners so box size scales with hand size.
            corner_tl = np.array([
                float(np.clip(
                    TABLE_X_MIN + (u_min / IMG_W) * (TABLE_X_MAX - TABLE_X_MIN),
                    TABLE_X_MIN, TABLE_X_MAX)),
                float(np.clip(
                    TABLE_Y_MIN + (v_min / IMG_H) * (TABLE_Y_MAX - TABLE_Y_MIN),
                    TABLE_Y_MIN, TABLE_Y_MAX)),
                Z
            ])
            corner_br = np.array([
                float(np.clip(
                    TABLE_X_MIN + (u_max / IMG_W) * (TABLE_X_MAX - TABLE_X_MIN),
                    TABLE_X_MIN, TABLE_X_MAX)),
                float(np.clip(
                    TABLE_Y_MIN + (v_max / IMG_H) * (TABLE_Y_MAX - TABLE_Y_MIN),
                    TABLE_Y_MIN, TABLE_Y_MAX)),
                Z
            ])

            box_w = abs(corner_br[0] - corner_tl[0]) + SAFETY_PADDING * 2
            box_h = abs(corner_br[1] - corner_tl[1]) + SAFETY_PADDING * 2
            box_d = DEPTH_PADDING + SAFETY_PADDING * 2

            # Publish directly in world frame — no TF transform needed in webcam mode
            # This is what actually publishes our green box
            co                 = CollisionObject()
            co.header          = Header()
            co.header.frame_id = "world"   # ← webcam mode stays in world frame
            co.header.stamp    = self.get_clock().now().to_msg()
            co.id              = COLLISION_ID
            co.operation       = CollisionObject.ADD

            box            = SolidPrimitive()
            box.type       = SolidPrimitive.BOX
            box.dimensions = [box_w, box_h, box_d]

            pose             = Pose()
            pose.position    = Point(
                x=float(centre_cam[0]),
                y=float(centre_cam[1]),
                z=float(centre_cam[2])
            )
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            co.primitives      = [box]
            co.primitive_poses = [pose]
            return co

        # ── KINECT MODE ───────────────────────────────────────────────────
        patch = self.latest_depth[
            max(0, v_c-5):min(IMG_H, v_c+5),
            max(0, u_c-5):min(IMG_W, u_c+5)
        ]
        valid = patch[patch > 0.1]

        if valid.size == 0:
            self.get_logger().warn(
                "No valid depth at hand centre — skipping frame.",
                throttle_duration_sec=1.0)
            return None

        Z = float(np.median(valid))

        # Use fixed Kinect v1 intrinsics instead of CameraInfo
        fx, fy = KINECT_FX, KINECT_FY
        cx, cy = KINECT_CX, KINECT_CY

        def deproject(u, v, z):
            return np.array([(u - cx) * z / fx,
                             (v - cy) * z / fy, z])

        centre_cam = deproject(u_c,   v_c,   Z)
        corner_tl  = deproject(u_min, v_min, Z)
        corner_br  = deproject(u_max, v_max, Z)

        box_w = abs(corner_br[0] - corner_tl[0]) + SAFETY_PADDING * 2
        box_h = abs(corner_br[1] - corner_tl[1]) + SAFETY_PADDING * 2
        box_d = DEPTH_PADDING + SAFETY_PADDING * 2

        pt                 = PointStamped()
        pt.header.frame_id = CAMERA_FRAME   # kinect_link
        pt.header.stamp    = msg.header.stamp
        pt.point.x         = float(centre_cam[0])
        pt.point.y         = float(centre_cam[1])
        pt.point.z         = float(centre_cam[2])

        try:
            pt_base = self.tf_buffer.transform(
                pt, BASE_FRAME,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
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

        co                 = CollisionObject()
        co.header          = Header()
        co.header.frame_id = BASE_FRAME
        co.header.stamp    = self.get_clock().now().to_msg()
        co.id              = COLLISION_ID
        co.operation       = CollisionObject.ADD

        box            = SolidPrimitive()
        box.type       = SolidPrimitive.BOX
        box.dimensions = [box_w, box_h, box_d]

        pose             = Pose()
        pose.position    = Point(
            x=pt_base.point.x,
            y=pt_base.point.y,
            z=pt_base.point.z
        )
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        co.primitives      = [box]
        co.primitive_poses = [pose]
        return co

    # ─────────────────────────────────────────────────────────────────────────
    #  HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_remove_only(self):
        remove_co           = CollisionObject()
        remove_co.id        = COLLISION_ID
        remove_co.operation = CollisionObject.REMOVE
        scene_msg                         = PlanningScene()
        scene_msg.is_diff                 = True
        scene_msg.world.collision_objects = [remove_co]
        self.scene_pub.publish(scene_msg)


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
