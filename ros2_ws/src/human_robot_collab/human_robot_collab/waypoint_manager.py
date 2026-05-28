#!/usr/bin/env python3
"""ROS2 node: table corner waypoint manager for HRC.

Responsibilities
----------------
1. Load 4 table-corner poses from ~/table_corners.json at startup.
2. Publish colored sphere + label markers to /waypoint_markers at 1 Hz so
   RViz always shows the corners.
3. Expose a topic-pair query interface:
     /waypoint_query   (geometry_msgs/Pose, subscriber) — publish a goal here
     /nearest_waypoint (geometry_msgs/PoseStamped, publisher) — nearest corner

The query interface is intentionally lightweight (topic pair rather than a
service) so it works without any custom interface package.  The C++ action
server computes nearest-corner internally; this interface is for test scripts
and BT nodes that need it via ROS.

Frame: base_link
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

# ─── Defaults ─────────────────────────────────────────────────────────────────

WAYPOINTS_FILE = os.path.join(os.path.expanduser("~"), "table_corners.json")
MARKER_FRAME   = "base_link"
PUBLISH_HZ     = 1.0

# Per-corner sphere colours  (r, g, b, a)
_CORNER_COLORS = {
    "TOP_LEFT":     (0.9, 0.2, 0.2, 0.9),   # red
    "TOP_RIGHT":    (0.2, 0.9, 0.2, 0.9),   # green
    "BOTTOM_LEFT":  (0.2, 0.4, 0.9, 0.9),   # blue
    "BOTTOM_RIGHT": (0.9, 0.8, 0.1, 0.9),   # yellow
}
_DEFAULT_COLOR = (0.7, 0.7, 0.7, 0.9)

# Hardcoded fallback (matches table_corners.json values)
_HARDCODED_CORNERS = {
    "TOP_LEFT":     ([-1.620,  0.454, 1.103], [-0.293, 0.956, 0.000, 0.017]),
    "TOP_RIGHT":    ([-0.220,  0.454, 1.103], [-0.293, 0.956, 0.000, 0.017]),
    "BOTTOM_LEFT":  ([-1.620, -0.256, 1.103], [-0.293, 0.956, 0.000, 0.017]),
    "BOTTOM_RIGHT": ([-0.220, -0.256, 1.103], [-0.293, 0.956, 0.000, 0.017]),
}


class WaypointManager(Node):

    def __init__(self):
        super().__init__("waypoint_manager")

        self.declare_parameter("waypoints_file", WAYPOINTS_FILE)
        wp_file = (self.get_parameter("waypoints_file")
                       .get_parameter_value().string_value)

        self._waypoints: dict[str, Pose] = self._load_waypoints(wp_file)

        # Publishers
        self._marker_pub = self.create_publisher(
            MarkerArray, "/waypoint_markers", 10)
        self._nearest_pub = self.create_publisher(
            PoseStamped, "/nearest_waypoint", 10)

        # Subscriber for query
        self.create_subscription(
            Pose, "/waypoint_query", self._query_cb, 10)

        # Periodic marker refresh
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_markers)

        self.get_logger().info(
            f"WaypointManager ready — {len(self._waypoints)} corners loaded "
            f"from {wp_file}")
        for name, pose in self._waypoints.items():
            p = pose.position
            self.get_logger().info(
                f"  {name:12s}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m")

    # ── Waypoint loading ──────────────────────────────────────────────────────

    def _load_waypoints(self, path: str) -> dict:
        try:
            with open(path) as f:
                data = json.load(f)
            corners = {}
            for name, corner in data["corners"].items():
                t = corner["translation"]
                q = corner["quaternion_xyzw"]
                pose = Pose()
                pose.position    = Point(x=float(t[0]), y=float(t[1]),
                                         z=float(t[2]))
                pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]),
                                              z=float(q[2]), w=float(q[3]))
                corners[name] = pose
            self.get_logger().info(
                f"Loaded {len(corners)} waypoints from {path}")
            return corners
        except FileNotFoundError:
            self.get_logger().warn(
                f"Waypoints file not found: {path} — using hardcoded defaults.")
        except Exception as exc:
            self.get_logger().error(
                f"Failed to parse {path}: {exc} — using hardcoded defaults.")

        corners = {}
        for name, (t, q) in _HARDCODED_CORNERS.items():
            pose = Pose()
            pose.position    = Point(x=t[0], y=t[1], z=t[2])
            pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            corners[name] = pose
        return corners

    # ── Nearest-waypoint query ────────────────────────────────────────────────

    def get_nearest(self, goal: Pose) -> tuple[str, Pose, float]:
        """Return (name, pose, distance) of the corner closest to *goal*."""
        best_name, best_pose, best_dist = None, None, float("inf")
        gx, gy, gz = goal.position.x, goal.position.y, goal.position.z
        for name, pose in self._waypoints.items():
            dx = pose.position.x - gx
            dy = pose.position.y - gy
            dz = pose.position.z - gz
            d  = math.sqrt(dx * dx + dy * dy + dz * dz)
            if d < best_dist:
                best_dist, best_name, best_pose = d, name, pose
        return best_name, best_pose, best_dist

    def _query_cb(self, goal: Pose):
        """Respond to /waypoint_query with the nearest corner on /nearest_waypoint."""
        name, pose, dist = self.get_nearest(goal)
        if pose is None:
            return
        stamped = PoseStamped()
        stamped.header = Header(
            frame_id=MARKER_FRAME,
            stamp=self.get_clock().now().to_msg())
        stamped.pose = pose
        self._nearest_pub.publish(stamped)
        self.get_logger().info(
            f"Query → nearest: {name} (dist={dist:.3f} m)",
            throttle_duration_sec=0.5)

    # ── RViz marker publication ───────────────────────────────────────────────

    def _publish_markers(self):
        if not self._waypoints:
            return

        now   = self.get_clock().now().to_msg()
        array = MarkerArray()
        lifetime = Duration(sec=2, nanosec=0)

        for i, (name, pose) in enumerate(self._waypoints.items()):
            r, g, b, a = _CORNER_COLORS.get(name, _DEFAULT_COLOR)
            base_id    = i * 2

            # ── Sphere at corner position ──────────────────────────────────
            sphere = Marker()
            sphere.header        = Header(frame_id=MARKER_FRAME, stamp=now)
            sphere.ns            = "wpt_spheres"
            sphere.id            = base_id
            sphere.type          = Marker.SPHERE
            sphere.action        = Marker.ADD
            sphere.pose          = pose
            sphere.scale.x       = 0.05
            sphere.scale.y       = 0.05
            sphere.scale.z       = 0.05
            sphere.color.r       = r
            sphere.color.g       = g
            sphere.color.b       = b
            sphere.color.a       = a
            sphere.lifetime      = lifetime
            array.markers.append(sphere)

            # ── Text label floating above the sphere ───────────────────────
            label = Marker()
            label.header        = Header(frame_id=MARKER_FRAME, stamp=now)
            label.ns            = "wpt_labels"
            label.id            = base_id + 1
            label.type          = Marker.TEXT_VIEW_FACING
            label.action        = Marker.ADD
            label.pose          = Pose(
                position=Point(
                    x=pose.position.x,
                    y=pose.position.y,
                    z=pose.position.z + 0.08),
                orientation=pose.orientation)
            label.scale.z       = 0.04
            label.color.r       = 1.0
            label.color.g       = 1.0
            label.color.b       = 1.0
            label.color.a       = 1.0
            label.text          = name
            label.lifetime      = lifetime
            array.markers.append(label)

        self._marker_pub.publish(array)


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
