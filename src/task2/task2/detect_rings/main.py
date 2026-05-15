import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import tf2_ros
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from msg_types.msg import RingDetect, SubjectClear
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import message_filters
from message_filters import ApproximateTimeSynchronizer


class Ring:
    def __init__(self, x, y, id=None, seen_counter=0, sum_x=0.0, sum_y=0.0):
        self.x = x
        self.y = y
        self.id = id
        self.seen_counter = seen_counter
        self.sum_x = sum_x
        self.sum_y = sum_y
        self.color_votes = {}
        self.color = "unknown"


class detect_rings2(Node):
    def __init__(self):
        super().__init__('detect_rings2')

        self.publish_hz = 5.0
        self.accept_threshold = 5
        self.dedup_distance = 0.4

        self.min_contour_area = 80.0
        self.max_contour_area = 90000.0
        self.min_ellipse_axis_px = 8.0
        self.max_ellipse_axis_px = 240.0
        self.min_axis_ratio = 0.3
        self.max_center_offset_px = 12.0
        self.min_outer_inner_ratio = 1.12
        self.max_outer_inner_ratio = 2.1
        self.min_ring_thickness_ratio = 0.04
        self.max_ring_thickness_ratio = 0.32

        self.depth_window_radius = 2
        self.min_valid_depth_samples = 4
        self.min_depth_m = 0.1
        self.max_depth_m = 8.0

        self.color_close_kernel = 11 
        self.color_open_kernel = 3
        self.min_annulus_fill = 0.18
        self.max_center_leak = 0.45
        self.max_center_same_color_ratio = 0.2
        # Height gate in map frame; can be tuned at launch/runtime.
        self.declare_parameter('min_ring_height_m', 0.82)
        self.min_ring_height_m = float(self.get_parameter('min_ring_height_m').value)

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
        self.timer = self.create_timer(1.0 / self.publish_hz, self.publish_ring_msg)

        self.potential_rings = []
        self.visited_rings = []
        self.visiting_rings = []
        self.id_counter = 64

    def build_marker(self, pc_msg, map_pose, ring_id):
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

    def visited(self, new_ring):
        x, y = new_ring[0], new_ring[1]
        for ring in self.visited_rings:
            if abs(ring.x - x) < self.dedup_distance and abs(ring.y - y) < self.dedup_distance:
                return True
        return False

    def seen(self, new_ring):
        x, y = new_ring[0], new_ring[1]
        for ring in self.potential_rings + self.visiting_rings:
            if abs(ring.x - x) < self.dedup_distance and abs(ring.y - y) < self.dedup_distance:
                return True, ring
        return False, None
    

    def ring_clear_callback(self, data):
        ring_id = data.id
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

    def publish_ring_msg(self):
        if len(self.visiting_rings) == 0:
            return
        ring = self.visiting_rings[0]
        msg = RingDetect()
        msg.x = ring.x
        msg.y = ring.y
        msg.id = ring.id
        msg.color = ring.color
        self.ring_pub.publish(msg)

    def median_depth_xyz(self, pc_xyz, cx, cy):
        h, w, _ = pc_xyz.shape
        rr = self.depth_window_radius
        x0 = max(0, cx - rr)
        x1 = min(w, cx + rr + 1)
        y0 = max(0, cy - rr)
        y1 = min(h, cy + rr + 1)
        patch = pc_xyz[y0:y1, x0:x1, :].reshape((-1, 3))
        finite_mask = np.isfinite(patch).all(axis=1)
        patch = patch[finite_mask ]
        if patch.shape[0] < self.min_valid_depth_samples:
            return None

        depth_norm = np.linalg.norm(patch, axis=1)
         
        patch = patch[(depth_norm > self.min_depth_m) & (depth_norm < self.max_depth_m)]
        if patch.shape[0] < self.min_valid_depth_samples:
            return None
        return np.median(patch, axis=0)

    def median_depth_annulus_xyz(self, pc_xyz, cx, cy, inner_radius, outer_radius):
        h, w, _ = pc_xyz.shape
        rr = int(max(outer_radius + 2, self.depth_window_radius))
        x0 = max(0, cx - rr)
        x1 = min(w, cx + rr + 1)
        y0 = max(0, cy - rr)
        y1 = min(h, cy + rr + 1)


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
        finite_mask = np.isfinite(patch).all(axis=1)
        patch = patch[finite_mask]
        if patch.shape[0] < self.min_valid_depth_samples:
            return None

        depth_norm = np.linalg.norm(patch, axis=1)
        patch = patch[(depth_norm > self.min_depth_m) & (depth_norm < self.max_depth_m)]
        if patch.shape[0] < self.min_valid_depth_samples:
            return None
        return np.median(patch, axis=0)

    def build_color_masks(self, bgr_image):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        red1 = cv2.inRange(hsv, (0, 90, 40), (10, 255, 255))
        red2 = cv2.inRange(hsv, (160, 90, 40), (179, 255, 255))

        red = cv2.bitwise_or(red1, red2)
        green = cv2.inRange(hsv, (35, 70, 40), (90, 255, 255))
        blue = cv2.inRange(hsv, (90, 70, 40), (140, 255, 255))
        yellow = cv2.inRange(hsv, (16, 90, 60), (35, 255, 255))
        black = cv2.inRange(hsv, (0, 0, 0), (179, 255, 30))

        ck = self.color_close_kernel if self.color_close_kernel % 2 == 1 else self.color_close_kernel + 1
        ok = self.color_open_kernel if self.color_open_kernel % 2 == 1 else self.color_open_kernel + 1
        close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ck, ck))
        open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ok, ok))

        def clean(mask):
            out = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
            out = cv2.morphologyEx(out, cv2.MORPH_OPEN, open_kernel)
            #return out
            return mask

        return {
            "red": clean(red),
            "green": clean(green),
            "blue": clean(blue),
            "yellow": clean(yellow),
            "black" : clean(black),
        }

    def ring_color_topology_ok(self, mask, cx, cy, inner_radius, outer_radius):
        h, w = mask.shape
        yy, xx = np.indices((h, w))
        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        annulus = (dist >= inner_radius) & (dist <= outer_radius)
        center = dist <= max(2.0, inner_radius * 0.75)

        annulus_count = int(np.count_nonzero(annulus))
        center_count = int(np.count_nonzero(center))
        if annulus_count < 20 or center_count < 8:
            return False

        color_bool = mask > 0
        ring_pix = int(np.count_nonzero(color_bool & annulus))
        center_pix = int(np.count_nonzero(color_bool & center))
        ring_fill = ring_pix / float(annulus_count)
        center_leak = center_pix / float(center_count)
        return ring_fill >= self.min_annulus_fill and center_leak <= self.max_center_leak

    def center_has_same_color(self, mask, cx, cy, inner_radius):
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

    def extract_ellipse_candidates(self, binary_image):
        contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:
            return [], binary_image, None
        hierarchy = hierarchy[0]

        ellipses = []
        for idx, cnt in enumerate(contours):
            if len(cnt) < 5:
                continue
            area = cv2.contourArea(cnt)
            if area < self.min_contour_area or area > self.max_contour_area:
                continue

            ellipse = cv2.fitEllipse(cnt)
            (cx, cy), (a, b), _angle = ellipse
            major = max(a, b)
            minor = min(a, b)
            if major < self.min_ellipse_axis_px or major > self.max_ellipse_axis_px:
                continue
            if minor <= 0:
                continue
            axis_ratio = minor / major
            if axis_ratio < self.min_axis_ratio:
                continue

            ellipses.append(
                {
                    "idx": idx,
                    "ellipse": ellipse,
                    "center": np.array([cx, cy], dtype=np.float32),
                    "major": float(major),
                    "minor": float(minor),
                    "child": int(hierarchy[idx][2]),
                }
            )
        return ellipses, binary_image, hierarchy

    def pair_ring_ellipses(self, ellipses, hierarchy):
        by_index = {e["idx"]: e for e in ellipses}
        pairs = []

        if hierarchy is None:
            return pairs

        for outer in ellipses:
            child_idx = outer["child"]
            best_inner = None
            best_dist = 1e9
            while child_idx != -1:
                inner = by_index.get(child_idx)
                if inner is not None and inner["major"] < outer["major"]:
                    center_dist = np.linalg.norm(outer["center"] - inner["center"])
                    if center_dist <= self.max_center_offset_px:
                        major_ratio = outer["major"] / inner["major"]
                        minor_ratio = outer["minor"] / inner["minor"] if inner["minor"] > 0 else 1e9
                        thick_ratio = (outer["major"] - inner["major"]) / max(outer["major"], 1e-6)
                        if (
                            self.min_outer_inner_ratio <= major_ratio <= self.max_outer_inner_ratio
                            and self.min_outer_inner_ratio <= minor_ratio <= self.max_outer_inner_ratio
                            and self.min_ring_thickness_ratio <= thick_ratio <= self.max_ring_thickness_ratio
                        ):
                            if center_dist < best_dist:
                                best_dist = center_dist
                                best_inner = inner


                child_idx = int(hierarchy[child_idx][0])

            if best_inner is not None:
                pairs.append((outer, best_inner))
        return pairs

    def synced_callback(self, rgb_msg, pc_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        color_masks = self.build_color_masks(cv_image)

        detections = []
        binary_preview = None
        for color_name, color_mask in color_masks.items():
            if binary_preview is None:
                binary_preview = color_mask.copy()
            else:
                binary_preview = cv2.bitwise_or(binary_preview, color_mask)

            img_blurred = cv2.medianBlur(color_mask, 5)
            circles = cv2.HoughCircles(
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
                continue

            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cx = int(i[0])
                cy = int(i[1])
                radius = int(i[2])
                if radius <= 0:
                    continue

                inner_radius = max(2.0, radius * 0.55)
                outer_radius = max(inner_radius + 1.0, float(radius))
                detections.append((cx, cy, inner_radius, outer_radius, color_name, color_mask, radius))

        height = pc_msg.height
        width = pc_msg.width
        pc = pc2.read_points_numpy(pc_msg, field_names=("x", "y", "z")).reshape((height, width, 3))

        for cx, cy, inner_radius, outer_radius, detected_color, color_mask, radius in detections:
            if cx < 0 or cy < 0 or cx >= width or cy >= height:
                continue

            if not self.ring_color_topology_ok(color_mask, cx, cy, inner_radius, outer_radius):
                continue
            if self.center_has_same_color(color_mask, cx, cy, inner_radius):
                continue

            xyz = self.median_depth_annulus_xyz(pc, cx, cy, inner_radius, outer_radius)
            if xyz is None:
                xyz = self.median_depth_xyz(pc, cx, cy)
            if xyz is None:
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
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            ring_x = map_pose.pose.position.x
            ring_y = map_pose.pose.position.y
            ring_z = map_pose.pose.position.z
            if not np.isfinite(ring_x) or not np.isfinite(ring_y) or not np.isfinite(ring_z):
                continue
            if ring_z <= self.min_ring_height_m:
                self.get_logger().debug(
                    f"Rejected ring candidate by height: z={ring_z:.3f} <= min_ring_height_m={self.min_ring_height_m:.3f}"
                )
                continue

            new_ring = (ring_x, ring_y)
            ring = None

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

            if not self.visited(new_ring):
                was_seen, ring = self.seen(new_ring)
                if not was_seen:
                    ring = Ring(x=ring_x, y=ring_y, seen_counter=1, sum_x=ring_x, sum_y=ring_y)
                    ring.color_votes[detected_color] = 1
                    ring.color = detected_color
                    self.potential_rings.append(ring)
                    self.get_logger().info(
                        f"POTENTIAL ring first seen color={detected_color} at ({ring_x:.2f}, {ring_y:.2f})"
                    )
                else:
                    ring.sum_x += ring_x
                    ring.sum_y += ring_y
                    ring.seen_counter += 1
                    ring.x = ring.sum_x / ring.seen_counter
                    ring.y = ring.sum_y / ring.seen_counter
                    ring.color_votes[detected_color] = ring.color_votes.get(detected_color, 0) + 1
                    ring.color = max(ring.color_votes, key=ring.color_votes.get)

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

            marker_id = 0
            if ring is not None and ring.id is not None:
                marker_id = ring.id
            marker = self.build_marker(pc_msg, map_pose, marker_id)
            self.marker_pub.publish(marker)

        cv2.imshow("ring_candidates", cv_image)
        if binary_preview is not None:
            cv2.imshow("ring_binary", binary_preview)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("exiting")
            exit()


def main():
    print('Ring detection v3 node starting.')

    rclpy.init(args=None)
    node = detect_rings2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
