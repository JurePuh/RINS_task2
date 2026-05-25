"""Service node that answers /line_fit_in_direction queries.

Given a robot-frame direction and angular cone half-width, takes the latest
/scan_filtered, projects every beam into base_link, restricts to the cone,
runs RANSAC to fit a 2D line, and returns:
  - perpendicular distance from base_link to the line,
  - yaw (rad, CCW positive) the robot must rotate so that `direction`
    points along the line's inward normal (i.e. so that `direction` becomes
    perpendicular to the wall),
  - the line equation as (normal_x, normal_y, offset).

Mirrors the structure of map_query/node.py.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from msg_types.srv import LineFitInDirection

from .exceptions import LineFitError, LineFitInternalError
from .ransac import fit_line_2d


# Minimum inlier count to consider the fit valid. Below this we treat the
# query as failed (LineFitError) rather than report a noisy line.
_MIN_POINTS_FOR_FIT = 15

# RANSAC parameters.
_RANSAC_ITERATIONS = 100
_RANSAC_INLIER_THRESHOLD_M = 0.03

class LineFitInDirectionNode(Node):
    """Service node answering /line_fit_in_direction."""

    def __init__(self) -> None:
        super().__init__("line_fit_in_direction_node")

        self._latest_scan: Optional[LaserScan] = None

        # Scan subscription. /scan_filtered is best-effort latency-wise, so use
        # SensorDataQoS-style best-effort reliability with depth 1.
        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._scan_sub = self.create_subscription(
            LaserScan, "/scan_filtered", self._on_scan, scan_qos
        )

        # TF buffer for base_link <- scan_frame lookup.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._srv = self.create_service(
            LineFitInDirection, "/line_fit_in_direction", self._handle
        )

        self._logger = self.get_logger()
        self._logger.info("LineFitInDirectionNode ready")

    # ── Scan subscription ─────────────────────────────────────────────────────

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    # ── Service handler ───────────────────────────────────────────────────────

    def _handle(
        self,
        request: LineFitInDirection.Request,
        response: LineFitInDirection.Response,
    ) -> LineFitInDirection.Response:
        try:
            response = self._query(request, response)
        except LineFitError as exc:
            self._logger.warning(f"LineFitInDirection query failed: {exc}")
            response.success = False
        # LineFitInternalError (and any other exception) propagates uncaught
        # so that programming errors crash loudly rather than being swallowed.
        return response

    def _query(
        self,
        request: LineFitInDirection.Request,
        response: LineFitInDirection.Response,
    ) -> LineFitInDirection.Response:
        # ── Step 1: validate input ────────────────────────────────────────────
        if not (0.0 < request.cone_half_width <= math.pi):
            raise LineFitError(
                f"bad input: cone_half_width must be in (0, pi], got {request.cone_half_width}"
            )
        if not (request.max_range > 0.0):
            raise LineFitError(
                f"bad input: max_range must be > 0, got {request.max_range}"
            )
        direction = float(request.direction)
        cone = float(request.cone_half_width)
        max_range = float(request.max_range)
        self._logger.debug(
            f"_query inputs: direction={direction:.3f} rad, "
            f"cone_half_width={cone:.3f} rad, max_range={max_range:.2f} m"
        )

        # ── Step 2: pull cached scan ──────────────────────────────────────────
        scan = self._latest_scan
        if scan is None:
            raise LineFitError("no scan received yet on /scan_filtered")
        self._logger.debug(
            f"latest scan: frame={scan.header.frame_id}, "
            f"{len(scan.ranges)} beams, "
            f"range=[{scan.range_min:.2f}, {scan.range_max:.2f}] m"
        )

        # ── Step 3: project scan beams into scan-frame (x, y) ────────────────
        ranges = np.asarray(scan.ranges, dtype=float)
        beam_angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = (
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges <= scan.range_max)
            & (ranges <= max_range)
        )
        ranges = ranges[valid]
        beam_angles = beam_angles[valid]
        if ranges.size == 0:
            raise LineFitError("no valid beams in scan after range filtering")
        xs_scan = ranges * np.cos(beam_angles)
        ys_scan = ranges * np.sin(beam_angles)
        pts_scan = np.stack([xs_scan, ys_scan], axis=1)
        self._logger.debug(
            f"valid beams after range filter: {ranges.size}/{len(scan.ranges)}"
        )

        # ── Step 4: transform points scan_frame → base_link ──────────────────
        pts_base = self._transform_points_to_base_link(pts_scan, scan.header.frame_id)

        # ── Step 5: cone filter in base_link ──────────────────────────────────
        bearings = np.arctan2(pts_base[:, 1], pts_base[:, 0])
        cone_mask = np.abs(_wrap_to_pi(bearings - direction)) <= cone
        pts_in_cone = pts_base[cone_mask]
        self._logger.debug(
            f"points in cone (direction={direction:.3f}, half_width={cone:.3f}): "
            f"{pts_in_cone.shape[0]}/{pts_base.shape[0]}"
        )
        if pts_in_cone.shape[0] < _MIN_POINTS_FOR_FIT:
            raise LineFitError(
                f"not enough points in cone: got {pts_in_cone.shape[0]}, "
                f"need >= {_MIN_POINTS_FOR_FIT}"
            )

        # ── Step 6: RANSAC line fit ───────────────────────────────────────────
        normal, offset, inlier_mask = fit_line_2d(
            pts_in_cone,
            iterations=_RANSAC_ITERATIONS,
            inlier_threshold=_RANSAC_INLIER_THRESHOLD_M,
        )
        inlier_count = int(inlier_mask.sum())
        if inlier_count < _MIN_POINTS_FOR_FIT:
            raise LineFitError(
                f"too few inliers from RANSAC: {inlier_count} < {_MIN_POINTS_FOR_FIT}"
            )

        # ── Step 7: orient normal so offset >= 0 (normal points wall→base_link) ─
        if offset < 0.0:
            normal = -normal
            offset = -offset
        perp_distance = float(offset)

        # ── Step 8: yaw rotation needed for `direction` to align with the normal ─
        normal_angle = float(math.atan2(normal[1], normal[0]))
        yaw_to_perp = _wrap_to_pi(normal_angle - direction)

        # ── Step 9: log ───────────────────────────────────────────────────────
        self._logger.info(
            f"line_fit: direction={direction:.3f}, cone={cone:.3f}, "
            f"points_in_cone={pts_in_cone.shape[0]}, inliers={inlier_count}, "
            f"perp_distance={perp_distance:.3f} m, yaw_to_perp={yaw_to_perp:.3f} rad, "
            f"normal=({normal[0]:.3f}, {normal[1]:.3f})"
        )

        # ── Step 10: fill response ────────────────────────────────────────────
        response.success = True
        response.perp_distance = perp_distance
        response.yaw_to_perp = float(yaw_to_perp)
        response.normal_x = float(normal[0])
        response.normal_y = float(normal[1])
        response.offset = perp_distance
        return response

    # ── TF helper ─────────────────────────────────────────────────────────────

    def _transform_points_to_base_link(
        self, pts_scan: np.ndarray, scan_frame: str
    ) -> np.ndarray:
        """Apply the base_link←scan_frame rigid transform to a (N, 2) array."""
        if scan_frame == "base_link":
            return pts_scan
        try:
            t = self._tf_buffer.lookup_transform(
                "base_link", scan_frame, rclpy.time.Time()
            )
        except Exception as exc:
            raise LineFitError(
                f"TF lookup base_link←{scan_frame} failed: {exc}"
            ) from exc

        tx = t.transform.translation.x
        ty = t.transform.translation.y
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        # 2D yaw from quaternion (z-axis component only).
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        c, s = math.cos(yaw), math.sin(yaw)

        xs = pts_scan[:, 0] * c - pts_scan[:, 1] * s + tx
        ys = pts_scan[:, 0] * s + pts_scan[:, 1] * c + ty
        self._logger.debug(
            f"TF base_link←{scan_frame}: tx={tx:.3f}, ty={ty:.3f}, yaw={yaw:.3f}"
        )
        return np.stack([xs, ys], axis=1)

def _wrap_to_pi(angle: float | np.ndarray) -> float | np.ndarray:
    """Wrap an angle (or array of angles) to (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
