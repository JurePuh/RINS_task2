from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import message_filters
from message_filters import ApproximateTimeSynchronizer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy

import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from msg_types.msg import RingDetect, SubjectClear


class Ring:
    def __init__(
        self,
        x: float,
        y: float,
        id: Optional[int] = None,
        seen_counter: int = 0,
        sum_x: float = 0.0,
        sum_y: float = 0.0,
    ) -> None:
        self.x: float = x
        self.y: float = y
        self.id: Optional[int] = id
        self.seen_counter: int = seen_counter
        self.sum_x: float = sum_x
        self.sum_y: float = sum_y
        self.color_votes: dict[str, int] = {}
        self.color: str = "unknown"
        self.last_published_x: Optional[float] = None
        self.last_published_y: Optional[float] = None


class DetectRings(Node):
    """Detects colored rings in RGB+depth, deduplicates them in map frame,
    and publishes the next ring to visit on /ring_detect."""

    def __init__(self) -> None:
        super().__init__('detect_rings2')

        # --- Publish / acceptance ---
        self.accept_threshold: int = 5
        self.dedup_distance: float = 0.4
        self.republish_move_threshold: float = 0.05

        # --- Contour / ellipse geometry (kept for tuning continuity) ---
        self.min_contour_area: float = 80.0
        self.max_contour_area: float = 90000.0
        self.min_ellipse_axis_px: float = 8.0
        self.max_ellipse_axis_px: float = 240.0
        self.min_axis_ratio: float = 0.3
        self.max_center_offset_px: float = 12.0
        self.min_outer_inner_ratio: float = 1.12
        self.max_outer_inner_ratio: float = 2.1
        self.min_ring_thickness_ratio: float = 0.04
        self.max_ring_thickness_ratio: float = 0.32

        # --- Depth sampling ---
        self.depth_window_radius: int = 2
        self.min_valid_depth_samples: int = 4
        self.min_depth_m: float = 0.1
        self.max_depth_m: float = 8.0

        # --- Color topology checks ---
        self.min_annulus_fill: float = 0.18
        self.max_center_leak: float = 0.45
        self.max_center_same_color_ratio: float = 0.2

        # Height gate in map frame; tunable at launch/runtime.
        self.declare_parameter('min_ring_height_m', 0.82)
        self.min_ring_height_m: float = float(self.get_parameter('min_ring_height_m').value)

        # --- ROS interfaces ---
        self.ring_pub = self.create_publisher(RingDetect, "/ring_detect", 10)
        self.marker_pub = self.create_publisher(Marker, "/ring_marker2", QoSReliabilityPolicy.BEST_EFFORT)
        self.clear_ring_sub = self.create_subscription(
            SubjectClear,
            "/ring_clear",
            self.ring_clear_callback,
            qos_profile_sensor_data,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rgb_sub = message_filters.Subscriber(
            self, Image, "/oakd/rgb/preview/image_raw", qos_profile=qos_profile_sensor_data
        )
        self.pc_sub = message_filters.Subscriber(
            self, PointCloud2, "/oakd/rgb/preview/depth/points", qos_profile=qos_profile_sensor_data
        )
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.pc_sub], queue_size=5, slop=0.05)
        self.ts.registerCallback(self.synced_callback)

        self.bridge = CvBridge()

        # --- State ---
        self.potential_rings: list[Ring] = []   # seen but not yet accepted
        self.visiting_rings: list[Ring] = []    # accepted, awaiting clear
        self.visited_rings: list[Ring] = []     # cleared, used for dedup
        self.id_counter: int = 64

        self.get_logger().info(
            f"DetectRings up: accept_threshold={self.accept_threshold} "
            f"dedup_distance={self.dedup_distance:.2f} min_ring_height_m={self.min_ring_height_m:.2f} "
            f"republish_move_threshold={self.republish_move_threshold:.3f}"
        )

    # --- Marker -------------------------------------------------------------

    def build_marker(self, pc_msg: PointCloud2, map_pose: PoseStamped, ring_id: Optional[int]) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = pc_msg.header.stamp
        marker.type = Marker.SPHERE
        marker.id = int(ring_id) if ring_id is not None else 0
        marker.pose.position.x = map_pose.pose.position.x
        marker.pose.position.y = map_pose.pose.position.y
        marker.pose.position.z = map_pose.pose.position.z
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    # --- Dedup helpers ------------------------------------------------------

    def visited(self, new_ring: tuple[float, float]) -> bool:
        x, y = new_ring
        for ring in self.visited_rings:
            if abs(ring.x - x) < self.dedup_distance and abs(ring.y - y) < self.dedup_distance:
                return True
        return False

    def seen(self, new_ring: tuple[float, float]) -> tuple[bool, Optional[Ring]]:
        x, y = new_ring
        for ring in self.potential_rings + self.visiting_rings:
            if abs(ring.x - x) < self.dedup_distance and abs(ring.y - y) < self.dedup_distance:
                return True, ring
        return False, None

    # --- Subscriptions / publishers ----------------------------------------

    def ring_clear_callback(self, data: SubjectClear) -> None:
        ring_id = data.id
        self.get_logger().debug(
            f"ring_clear_callback: id={ring_id} visiting_queue_len={len(self.visiting_rings)}"
        )
        if len(self.visiting_rings) == 0:
            self.get_logger().info(f"CLEAR received for id={ring_id}, but visiting queue is empty.")
            return
        if self.visiting_rings[0].id == ring_id:
            cleared = self.visiting_rings.pop(0)
            self.visited_rings.append(cleared)
            self.get_logger().info(
                f"CLEARED ring id={cleared.id} color={cleared.color} at ({cleared.x:.2f}, {cleared.y:.2f})"
            )
            if len(self.visiting_rings) > 0:
                first_ring = self.visiting_rings[0]
                self.get_logger().info(
                    f"VISITING first-in-queue id={first_ring.id} color={first_ring.color} "
                    f"at ({first_ring.x:.2f}, {first_ring.y:.2f})"
                )
        else:
            self.get_logger().info(
                f"CLEAR received for id={ring_id}, but first-in-queue id={self.visiting_rings[0].id}. Ignoring."
            )

    def publish_ring_msg(self, ring: Ring) -> None:
        msg = RingDetect()
        msg.x = ring.x
        msg.y = ring.y
        msg.id = ring.id if ring.id is not None else 0
        msg.color = ring.color
        self.ring_pub.publish(msg)
        ring.last_published_x = ring.x
        ring.last_published_y = ring.y
        self.get_logger().info(
            f"published RingDetect id={msg.id} color={msg.color} at ({msg.x:.2f}, {msg.y:.2f})"
        )

    def maybe_republish(self, ring: Ring) -> None:
        if ring.last_published_x is None or ring.last_published_y is None:
            return
        dx = ring.x - ring.last_published_x
        dy = ring.y - ring.last_published_y
        if (dx * dx + dy * dy) ** 0.5 > self.republish_move_threshold:
            self.publish_ring_msg(ring)

    # --- Depth sampling -----------------------------------------------------

    def _filter_depth_samples(self, patch: np.ndarray) -> Optional[np.ndarray]:
        """Drop non-finite and out-of-range samples; require a minimum count."""
        finite = np.isfinite(patch).all(axis=1)
        patch = patch[finite]
        if patch.shape[0] < self.min_valid_depth_samples:
            return None
        norm = np.linalg.norm(patch, axis=1)
        patch = patch[(norm > self.min_depth_m) & (norm < self.max_depth_m)]
        if patch.shape[0] < self.min_valid_depth_samples:
            return None
        return patch

    def median_depth_xyz(self, pc_xyz: np.ndarray, cx: int, cy: int) -> Optional[np.ndarray]:
        h, w, _ = pc_xyz.shape
        rr = self.depth_window_radius
        x0, x1 = max(0, cx - rr), min(w, cx + rr + 1)
        y0, y1 = max(0, cy - rr), min(h, cy + rr + 1)
        patch = pc_xyz[y0:y1, x0:x1, :].reshape((-1, 3))
        patch = self._filter_depth_samples(patch)
        if patch is None:
            return None
        return np.median(patch, axis=0)

    def median_depth_annulus_xyz(
        self,
        pc_xyz: np.ndarray,
        cx: int,
        cy: int,
        inner_radius: float,
        outer_radius: float,
    ) -> Optional[np.ndarray]:
        """Sample depth on the ring's annulus (avoids the empty hole behind it)."""
        h, w, _ = pc_xyz.shape
        rr = int(max(outer_radius + 2, self.depth_window_radius))
        x0, x1 = max(0, cx - rr), min(w, cx + rr + 1)
        y0, y1 = max(0, cy - rr), min(h, cy + rr + 1)
        if x0 >= x1 or y0 >= y1:
            return None

        yy, xx = np.indices((y1 - y0, x1 - x0))
        xx = xx + x0
        yy = yy + y0
        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        annulus_mask = (dist >= inner_radius) & (dist <= outer_radius)
        if not np.any(annulus_mask):
            return None

        patch = pc_xyz[y0:y1, x0:x1, :][annulus_mask]
        patch = self._filter_depth_samples(patch)
        if patch is None:
            return None
        return np.median(patch, axis=0)

    # --- Color masks --------------------------------------------------------

    def build_color_masks(self, bgr_image: np.ndarray) -> dict[str, np.ndarray]:
        """HSV thresholding for each ring color. Morphology intentionally disabled."""
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        red1 = cv2.inRange(hsv, (0, 90, 40), (10, 255, 255))
        red2 = cv2.inRange(hsv, (160, 90, 40), (179, 255, 255))
        red = cv2.bitwise_or(red1, red2)
        green = cv2.inRange(hsv, (35, 70, 40), (90, 255, 255))
        blue = cv2.inRange(hsv, (90, 70, 40), (140, 255, 255))
        yellow = cv2.inRange(hsv, (16, 90, 60), (35, 255, 255))
        black = cv2.inRange(hsv, (0, 0, 0), (179, 255, 30))
        return {
            "red": red,
            "green": green,
            "blue": blue,
            "yellow": yellow,
            "black": black,
        }

    # --- Topology checks ----------------------------------------------------

    def ring_color_topology_ok(
        self,
        mask: np.ndarray,
        cx: int,
        cy: int,
        inner_radius: float,
        outer_radius: float,
    ) -> bool:
        """The annulus should be mostly the color, the center mostly not."""
        h, w = mask.shape
        yy, xx = np.indices((h, w))
        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        annulus = (dist >= inner_radius) & (dist <= outer_radius)
        center = dist <= max(2.0, inner_radius * 0.75)

        annulus_count = int(np.count_nonzero(annulus))
        center_count = int(np.count_nonzero(center))
        if annulus_count < 20 or center_count < 8:
            self.get_logger().debug(
                f"topology: too-small regions annulus={annulus_count} center={center_count}"
            )
            return False

        color_bool = mask > 0
        ring_pix = int(np.count_nonzero(color_bool & annulus))
        center_pix = int(np.count_nonzero(color_bool & center))
        ring_fill = ring_pix / float(annulus_count)
        center_leak = center_pix / float(center_count)
        ok = ring_fill >= self.min_annulus_fill and center_leak <= self.max_center_leak
        self.get_logger().debug(
            f"topology: ring_fill={ring_fill:.2f} center_leak={center_leak:.2f} ok={ok}"
        )
        return ok

    def center_has_same_color(self, mask: np.ndarray, cx: int, cy: int, inner_radius: float) -> bool:
        """True if the disk inside the ring is also the ring color (rejects filled blobs)."""
        h, w = mask.shape
        yy, xx = np.indices((h, w))
        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        center = dist <= max(2.0, inner_radius * 0.6)
        center_count = int(np.count_nonzero(center))
        if center_count == 0:
            return True

        color_bool = mask > 0
        center_pix = int(np.count_nonzero(color_bool & center))
        center_ratio = center_pix / float(center_count)
        return center_ratio > self.max_center_same_color_ratio

    # --- Main callback ------------------------------------------------------

    def synced_callback(self, rgb_msg: Image, pc_msg: PointCloud2) -> None:
        """For each synced RGB+PC frame: find ring candidates per color, depth-resolve,
        transform to map, dedup, promote potential→visiting after enough hits."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        h_img, w_img = cv_image.shape[:2]
        self.get_logger().debug(f"frame received, image={w_img}x{h_img}")

        color_masks = self.build_color_masks(cv_image)
        for cname, m in color_masks.items():
            self.get_logger().debug(f"mask {cname}: nonzero={int(np.count_nonzero(m))}")

        # --- Find raw circle candidates per color ---
        detections: list[tuple[int, int, float, float, str, np.ndarray, int]] = []
        binary_preview: Optional[np.ndarray] = None
        for color_name, color_mask in color_masks.items():
            binary_preview = color_mask.copy() if binary_preview is None else cv2.bitwise_or(binary_preview, color_mask)

            img_blurred = cv2.medianBlur(color_mask, 5)
            circles: Optional[np.ndarray] = cv2.HoughCircles(
                img_blurred,
                cv2.HOUGH_GRADIENT_ALT,
                1,
                20,
                param1=100,
                param2=0.8,
                minRadius=5,
                maxRadius=0,
            )
            if circles is None:
                self.get_logger().debug(f"hough {color_name}: no circles")
                continue

            circles = np.uint16(np.around(circles))
            self.get_logger().debug(f"hough {color_name}: {circles.shape[1]} raw circles")
            for i in circles[0, :]:
                cx = int(i[0])
                cy = int(i[1])
                radius = int(i[2])
                if radius <= 0:
                    continue
                inner_radius = max(2.0, radius * 0.55)
                outer_radius = max(inner_radius + 1.0, float(radius))
                detections.append((cx, cy, inner_radius, outer_radius, color_name, color_mask, radius))

        # --- Resolve each candidate in 3D and update state ---
        height = pc_msg.height
        width = pc_msg.width
        pc = pc2.read_points_numpy(pc_msg, field_names=("x", "y", "z")).reshape((height, width, 3))

        for cx, cy, inner_radius, outer_radius, detected_color, color_mask, radius in detections:
            self.get_logger().debug(
                f"candidate color={detected_color} cx={cx} cy={cy} r={radius} "
                f"inner={inner_radius:.1f} outer={outer_radius:.1f}"
            )

            if cx < 0 or cy < 0 or cx >= width or cy >= height:
                self.get_logger().debug(f"REJECTED out-of-bounds ({cx},{cy}) vs ({width},{height})")
                continue
            if not self.ring_color_topology_ok(color_mask, cx, cy, inner_radius, outer_radius):
                self.get_logger().debug("REJECTED topology")
                continue
            if self.center_has_same_color(color_mask, cx, cy, inner_radius):
                self.get_logger().debug("REJECTED center has same color")
                continue

            xyz = self.median_depth_annulus_xyz(pc, cx, cy, inner_radius, outer_radius)
            if xyz is None:
                xyz = self.median_depth_xyz(pc, cx, cy)
            if xyz is None:
                self.get_logger().debug("REJECTED no valid depth samples")
                continue

            pose = PoseStamped()
            pose.header = pc_msg.header
            pose.pose.position.x = float(xyz[0])
            pose.pose.position.y = float(xyz[1])
            pose.pose.position.z = float(xyz[2])
            pose.pose.orientation.w = 1.0

            try:
                trans = self.tf_buffer.lookup_transform(
                    "map",
                    pose.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                map_pose = tf2_geometry_msgs.do_transform_pose_stamped(pose, trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().debug(f"REJECTED tf lookup failed: {e}")
                continue

            ring_x = map_pose.pose.position.x
            ring_y = map_pose.pose.position.y
            ring_z = map_pose.pose.position.z
            if not np.isfinite(ring_x) or not np.isfinite(ring_y) or not np.isfinite(ring_z):
                self.get_logger().debug(
                    f"REJECTED non-finite map pose ({ring_x}, {ring_y}, {ring_z})"
                )
                continue
            if ring_z <= self.min_ring_height_m:
                self.get_logger().debug(
                    f"REJECTED by height: z={ring_z:.3f} <= min={self.min_ring_height_m:.3f} "
                    f"at ({ring_x:.2f}, {ring_y:.2f})"
                )
                continue

            new_ring = (ring_x, ring_y)
            ring: Optional[Ring] = None

            # Overlay on preview window.
            cv2.circle(cv_image, (cx, cy), radius, (0, 255, 0), 2)
            cv2.circle(cv_image, (cx, cy), 3, (0, 0, 255), -1)
            cv2.putText(
                cv_image,
                detected_color,
                (cx + 5, cy - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            if self.visited(new_ring):
                self.get_logger().debug(
                    f"skipping already-visited ring near ({ring_x:.2f}, {ring_y:.2f})"
                )
            else:
                was_seen, ring = self.seen(new_ring)
                if not was_seen:
                    ring = Ring(x=ring_x, y=ring_y, seen_counter=1, sum_x=ring_x, sum_y=ring_y)
                    ring.color_votes[detected_color] = 1
                    ring.color = detected_color
                    self.potential_rings.append(ring)
                    self.get_logger().info(
                        f"POTENTIAL ring first seen color={detected_color} at ({ring_x:.2f}, {ring_y:.2f})"
                    )
                    self.get_logger().debug(
                        f"queues: potential={len(self.potential_rings)} "
                        f"visiting={len(self.visiting_rings)} visited={len(self.visited_rings)}"
                    )
                else:
                    assert ring is not None
                    ring.sum_x += ring_x
                    ring.sum_y += ring_y
                    ring.seen_counter += 1
                    ring.x = ring.sum_x / ring.seen_counter
                    ring.y = ring.sum_y / ring.seen_counter
                    ring.color_votes[detected_color] = ring.color_votes.get(detected_color, 0) + 1

                    # Track color flips on the running vote.
                    old_color = ring.color
                    new_color = max(ring.color_votes, key=lambda c: ring.color_votes[c])
                    if new_color != old_color:
                        ring.color = new_color
                        self.get_logger().info(
                            f"ring at ({ring.x:.2f}, {ring.y:.2f}) color vote switched: "
                            f"{old_color} -> {new_color} (votes={ring.color_votes})"
                        )
                    else:
                        ring.color = new_color
                    self.get_logger().debug(
                        f"updated ring at ({ring.x:.2f}, {ring.y:.2f}) "
                        f"seen={ring.seen_counter} votes={ring.color_votes}"
                    )

                    if ring.seen_counter > self.accept_threshold and ring not in self.visiting_rings:
                        ring.id = self.id_counter
                        self.id_counter += 1
                        self.visiting_rings.append(ring)
                        self.potential_rings.remove(ring)
                        self.get_logger().info(
                            f"ACCEPTED ring id={ring.id} color={ring.color} "
                            f"seen={ring.seen_counter} at ({ring.x:.2f}, {ring.y:.2f})"
                        )
                        if len(self.visiting_rings) == 1:
                            self.get_logger().info(
                                f"VISITING first-in-queue id={ring.id} color={ring.color} "
                                f"at ({ring.x:.2f}, {ring.y:.2f})"
                            )
                        self.publish_ring_msg(ring)
                    elif ring in self.visiting_rings:
                        self.maybe_republish(ring)

            marker_id: Optional[int] = ring.id if (ring is not None and ring.id is not None) else 0
            marker = self.build_marker(pc_msg, map_pose, marker_id)
            self.marker_pub.publish(marker)

        cv2.imshow("ring_candidates", cv_image)
        if binary_preview is not None:
            cv2.imshow("ring_binary", binary_preview)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            self.get_logger().info("ESC pressed, shutting down.")
            rclpy.shutdown()
            return


def main() -> None:
    rclpy.init(args=None)
    node = DetectRings()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
