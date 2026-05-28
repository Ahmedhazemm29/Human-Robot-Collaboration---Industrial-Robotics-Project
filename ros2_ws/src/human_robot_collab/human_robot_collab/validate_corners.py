#!/usr/bin/env python3
"""Validate that all table corner waypoints are kinematically reachable by the UR5e.

Usage:
  ros2 run human_robot_collab validate_corners
  ros2 run human_robot_collab validate_corners /path/to/table_corners.json

Requires a running MoveIt2 instance (/compute_ik service must be available).
Reports reachability, joint-limit margins, and suggests nearby poses for
unreachable corners.
"""

import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK

# ─── Constants ────────────────────────────────────────────────────────────────

WAYPOINTS_FILE   = os.path.join(os.path.expanduser("~"), "table_corners.json")
PLANNING_GROUP   = "ur_manipulator"
EEF_LINK         = "tool0"
IK_TIMEOUT_S     = 1.0
SERVICE_WAIT_S   = 10.0
JOINT_WARN_DEG   = 5.0   # warn if closer than this to a joint limit

# UR5e joint limits (radians) — symmetric at ±2π for pan/wrist, ±π for elbow
_UR5E_JOINT_LIMITS = [
    (-6.2832, 6.2832),   # shoulder_pan
    (-6.2832, 6.2832),   # shoulder_lift
    (-3.1416, 3.1416),   # elbow
    (-6.2832, 6.2832),   # wrist_1
    (-6.2832, 6.2832),   # wrist_2
    (-6.2832, 6.2832),   # wrist_3
]


class CornerValidator(Node):

    def __init__(self):
        super().__init__("corner_validator")
        self._ik = self.create_client(GetPositionIK, "/compute_ik")

    # ── Public entry point ────────────────────────────────────────────────────

    def validate_all(self, waypoints_file: str) -> bool:
        self.get_logger().info(
            f"Waiting for /compute_ik (up to {SERVICE_WAIT_S}s)…")
        if not self._ik.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            self.get_logger().error(
                "/compute_ik not available — is MoveIt2 running?")
            return False

        try:
            with open(waypoints_file) as f:
                data = json.load(f)
        except Exception as exc:
            self.get_logger().error(f"Cannot load {waypoints_file}: {exc}")
            return False

        corners = data.get("corners", {})
        frame   = data.get("frame_id", "base_link")
        self.get_logger().info(
            f"Validating {len(corners)} corners from {waypoints_file} "
            f"(frame: {frame})")

        all_ok = True
        for name, corner in corners.items():
            t  = corner["translation"]       # [x, y, z]
            q  = corner["quaternion_xyzw"]   # [qx, qy, qz, qw]

            stamped = self._make_pose_stamped(t, q, frame)
            ok, code_str, joints = self._call_ik(stamped)

            if ok:
                margin_deg = math.degrees(self._min_joint_margin(joints))
                status = "REACHABLE"
                if margin_deg < JOINT_WARN_DEG:
                    status += (f"  ⚠ joint-limit margin only {margin_deg:.1f}°"
                               f" (threshold {JOINT_WARN_DEG}°)")
                    all_ok = False
                self.get_logger().info(
                    f"  {name:12s}: {status}\n"
                    f"             pos=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) m  "
                    f"margin={margin_deg:.1f}°")
            else:
                self.get_logger().error(
                    f"  {name:12s}: UNREACHABLE ({code_str})\n"
                    f"             pos=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) m")
                self._suggest_nearby(stamped)
                all_ok = False

        sep = "─" * 60
        if all_ok:
            self.get_logger().info(
                f"\n{sep}\nAll {len(corners)} corners REACHABLE "
                f"with sufficient margin.\n{sep}")
        else:
            self.get_logger().warn(
                f"\n{sep}\nSome corners have issues — review messages above.\n{sep}")
        return all_ok

    # ── Private helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _make_pose_stamped(
            translation: list,
            quaternion_xyzw: list,
            frame: str = "base_link") -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.pose.position    = Point(
            x=translation[0], y=translation[1], z=translation[2])
        ps.pose.orientation = Quaternion(
            x=quaternion_xyzw[0], y=quaternion_xyzw[1],
            z=quaternion_xyzw[2], w=quaternion_xyzw[3])
        return ps

    def _call_ik(self, stamped: PoseStamped):
        """Return (reachable: bool, error_str: str, joint_positions: list)."""
        req = GetPositionIK.Request()
        req.ik_request.group_name       = PLANNING_GROUP
        req.ik_request.ik_link_name     = EEF_LINK
        req.ik_request.pose_stamped     = stamped
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec      = int(IK_TIMEOUT_S)
        req.ik_request.timeout.nanosec  = int(
            (IK_TIMEOUT_S - int(IK_TIMEOUT_S)) * 1e9)

        future = self._ik.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=IK_TIMEOUT_S + 2.0)

        if not future.done():
            return False, "service_timeout", []

        res = future.result()
        if res.error_code.val == MoveItErrorCodes.SUCCESS:
            joints = list(res.solution.joint_state.position)
            return True, "OK", joints

        code_map = {
            MoveItErrorCodes.NO_IK_SOLUTION:       "NO_IK_SOLUTION",
            MoveItErrorCodes.GOAL_IN_COLLISION:     "GOAL_IN_COLLISION",
            MoveItErrorCodes.INVALID_LINK_NAME:     "INVALID_LINK_NAME",
            MoveItErrorCodes.FAILURE:               "FAILURE",
            MoveItErrorCodes.PLANNING_FAILED:       "PLANNING_FAILED",
        }
        label = code_map.get(res.error_code.val, f"code={res.error_code.val}")
        return False, label, []

    @staticmethod
    def _min_joint_margin(joints: list) -> float:
        """Return the minimum angular distance to any joint limit (radians)."""
        min_m = float("inf")
        for j, (lo, hi) in zip(joints, _UR5E_JOINT_LIMITS):
            margin = min(abs(j - lo), abs(hi - j))
            min_m  = min(min_m, margin)
        return min_m

    def _suggest_nearby(self, original: PoseStamped):
        """Try small offsets and report the first reachable one."""
        offsets = [
            ( 0.05,  0.00,  0.00, "+X"),
            (-0.05,  0.00,  0.00, "-X"),
            ( 0.00,  0.05,  0.00, "+Y"),
            ( 0.00, -0.05,  0.00, "-Y"),
            ( 0.00,  0.00,  0.05, "+Z"),
            ( 0.00,  0.00, -0.05, "-Z"),
        ]
        self.get_logger().info("    Searching for nearby reachable pose…")
        for dx, dy, dz, label in offsets:
            near = PoseStamped()
            near.header = original.header
            near.pose.position.x = original.pose.position.x + dx
            near.pose.position.y = original.pose.position.y + dy
            near.pose.position.z = original.pose.position.z + dz
            near.pose.orientation = original.pose.orientation
            ok, _, _ = self._call_ik(near)
            if ok:
                p = near.pose.position
                self.get_logger().info(
                    f"    Suggested ({label}): "
                    f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m")
                return
        self.get_logger().warn(
            "    No reachable pose found within 5 cm offsets.")


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CornerValidator()

    wp_file = sys.argv[1] if len(sys.argv) > 1 else WAYPOINTS_FILE

    success = node.validate_all(wp_file)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
