"""
ROS 2 service: top camera RGB - intensity mask, quad warp, SuperSimpleNet anomaly check.

Run: ros2 run task2 detect_anomalies
Call: ros2 service call /detect_anomalies msg_types/srv/DetectAnomalies {}
"""
from __future__ import annotations

import os
import sys
import threading
import time
from datetime import datetime, timezone

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from msg_types.srv import DetectAnomalies

_this_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_this_dir, "ssn"))

from anomaly_detector import AnomalyDetector  # noqa: E402

RESULT_DEFECTED = "defected"
RESULT_NOT_DEFECTED = "not_defected"
RESULT_NOT_FOUND = "not_found"


class DetectAnomaliesNode(Node):
    def __init__(self):
        super().__init__("detect_anomalies")

        self.declare_parameter("image_topic", "/top_camera/rgb/preview/image_raw")
        self.declare_parameter("frame_timeout_sec", 2.0)
        self.declare_parameter("output_dir", os.path.join(_this_dir, "img"))

        self.declare_parameter("intensity_threshold", 100)
        self.declare_parameter("pre_blur_kernel", 5)
        self.declare_parameter("closing_kernel", 5)
        self.declare_parameter("pile_min_contour_area", 500.0)
        self.declare_parameter("mask_border_strip_px", 30)
        self.declare_parameter("pile_corner_eps_ratio_start", 0.008)
        self.declare_parameter("pile_corner_eps_ratio_max", 0.14)
        self.declare_parameter("pile_warp_size", 256)
        self.declare_parameter("anomaly_peak_mask_threshold", 0.7)

        self._image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self._frame_timeout = self.get_parameter("frame_timeout_sec").get_parameter_value().double_value
        self._output_root = self.get_parameter("output_dir").get_parameter_value().string_value

        self.detector = AnomalyDetector(
            weights_path=os.path.join(_this_dir, "ssn", "weights", "anomaly_model_augmented2.pt"),
            device="cuda",
            score_threshold=0.5,
            mask_threshold=0.5,
        )

        self._bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._last_msg: Image | None = None
        self._last_stamp_ns: int | None = None
        self._cb_group = ReentrantCallbackGroup()

        self._sub = self.create_subscription(
            Image,
            self._image_topic,
            self._image_cb,
            1,
            callback_group=self._cb_group,
        )
        self._srv = self.create_service(
            DetectAnomalies,
            "detect_anomalies",
            self._handle_detect,
            callback_group=self._cb_group,
        )
        os.makedirs(self._output_root, exist_ok=True)
        self.get_logger().info(
            f"detect_anomalies service ready: topic={self._image_topic} output={self._output_root}"
        )

    def _image_cb(self, msg: Image) -> None:
        with self._frame_lock:
            self._last_msg = msg
            self._last_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _wait_fresh_frame(self, previous_stamp_ns: int | None) -> Image | None:
        t0 = time.monotonic()
        while (time.monotonic() - t0) < self._frame_timeout:
            with self._frame_lock:
                last_msg = self._last_msg
                last_stamp = self._last_stamp_ns
            if last_msg is None:
                time.sleep(0.01)
                continue
            if previous_stamp_ns is None:
                return last_msg
            if last_stamp != previous_stamp_ns:
                return last_msg
            time.sleep(0.01)
        return None

    @staticmethod
    def _bgr_masked_by_binary_mask(bgr: np.ndarray, mask: np.ndarray) -> np.ndarray:
        out = np.zeros_like(bgr)
        cv2.bitwise_and(bgr, bgr, out, mask=mask)
        return out

    @staticmethod
    def _strip_mask_border(mask: np.ndarray, margin_px: int) -> np.ndarray:
        if margin_px <= 0:
            return mask
        out = mask.copy()
        h, w = out.shape[:2]
        m = min(margin_px, max(0, h // 2 - 1), max(0, w // 2 - 1))
        if m <= 0:
            return out
        out[:m, :] = 0
        out[h - m :, :] = 0
        out[:, :m] = 0
        out[:, w - m :] = 0
        return out

    @staticmethod
    def _fill_quad_interior(mask: np.ndarray, quad_xy: np.ndarray) -> np.ndarray:
        out = mask.copy()
        poly = quad_xy.reshape(-1, 1, 2).astype(np.int32)
        cv2.fillPoly(out, [poly], 255)
        return out

    @staticmethod
    def _sort_quad_ccw(quad_xy: np.ndarray) -> np.ndarray:
        c = quad_xy.astype(np.float64).mean(axis=0)
        ang = np.arctan2(
            quad_xy[:, 1].astype(np.float64) - c[1],
            quad_xy[:, 0].astype(np.float64) - c[0],
        )
        order = np.argsort(ang)
        return quad_xy[order].copy()

    @staticmethod
    def _quad_from_largest_contour(
        mask: np.ndarray,
        min_area: float,
        eps_ratio_start: float,
        eps_ratio_max: float,
    ) -> np.ndarray | None:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < min_area:
            return None
        if len(cnt) < 3:
            return None

        hull = cv2.convexHull(cnt)
        if hull is None or len(hull) < 3:
            return None
        peri = float(cv2.arcLength(hull, True))
        if peri < 1e-3:
            return None

        rs = max(1e-6, float(eps_ratio_start))
        rm = max(rs, float(eps_ratio_max))
        quad = None
        for eps in np.linspace(rs * peri, rm * peri, num=14):
            approx = cv2.approxPolyDP(hull, eps, True)
            if len(approx) == 4:
                quad = approx.reshape(4, 2)
                break

        if quad is None:
            rect = cv2.minAreaRect(cnt.reshape(-1, 2))
            box = cv2.boxPoints(rect)
            quad = np.rint(box).astype(np.int32)

        quad = DetectAnomaliesNode._sort_quad_ccw(quad)
        h, w = mask.shape[:2]
        quad[:, 0] = np.clip(quad[:, 0], 0, w - 1)
        quad[:, 1] = np.clip(quad[:, 1], 0, h - 1)
        return quad

    def _draw_pile_quad(self, bgr: np.ndarray, quad_xy: np.ndarray) -> np.ndarray:
        out = bgr.copy()
        h, w = bgr.shape[:2]
        r = max(4, int(0.003 * max(h, w)))
        poly = quad_xy.reshape(-1, 1, 2)
        cv2.polylines(out, [poly], isClosed=True, color=(220, 220, 220), thickness=2, lineType=cv2.LINE_AA)
        corner_colors = ((0, 0, 255), (0, 255, 255), (0, 220, 0), (255, 64, 64))
        for i, ((x, y), color) in enumerate(zip(quad_xy, corner_colors)):
            cv2.circle(out, (int(x), int(y)), r, color, -1, cv2.LINE_AA)
            cv2.circle(out, (int(x), int(y)), r + 2, (255, 255, 255), 1, cv2.LINE_AA)
            tx = min(w - 1, int(x) + r + 4)
            ty = max(16, int(y) - r - 4)
            cv2.putText(out, str(i), (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
        return out

    @staticmethod
    def _dst_quad_square(side: int) -> np.ndarray:
        s = side - 1
        raw = np.array([[0, 0], [s, 0], [s, s], [0, s]], dtype=np.int32)
        return DetectAnomaliesNode._sort_quad_ccw(raw).astype(np.float32)

    @staticmethod
    def _warp_pile_to_square(bgr: np.ndarray, quad_src_ccw: np.ndarray, side: int) -> np.ndarray:
        src = quad_src_ccw.astype(np.float32)
        dst = DetectAnomaliesNode._dst_quad_square(side)
        h = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(
            bgr,
            h,
            (side, side),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

    def _intensity_binary_mask(self, bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        k = int(self.get_parameter("pre_blur_kernel").get_parameter_value().integer_value)
        if k >= 3 and k % 2 == 1:
            gray = cv2.GaussianBlur(gray, (k, k), 0)

        thresh = int(self.get_parameter("intensity_threshold").get_parameter_value().integer_value)
        thresh = max(0, min(255, thresh))
        _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)

        ck = int(self.get_parameter("closing_kernel").get_parameter_value().integer_value)
        if ck >= 3 and ck % 2 == 1:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ck, ck))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return gray, mask

    def _run_pipeline(self, cv_image: np.ndarray) -> tuple[str, dict[str, np.ndarray] | None]:
        _gray, mask = self._intensity_binary_mask(cv_image)
        border_px = int(self.get_parameter("mask_border_strip_px").value)
        if border_px < 0:
            border_px = 0
        mask = self._strip_mask_border(mask, border_px)

        min_area = float(self.get_parameter("pile_min_contour_area").value)
        if min_area < 1.0:
            min_area = 500.0

        eps0 = float(self.get_parameter("pile_corner_eps_ratio_start").value)
        eps1 = float(self.get_parameter("pile_corner_eps_ratio_max").value)
        if eps0 <= 0:
            eps0 = 0.008
        if eps1 < eps0:
            eps1 = eps0 + 0.02

        quad = self._quad_from_largest_contour(
            mask,
            min_area=min_area,
            eps_ratio_start=eps0,
            eps_ratio_max=eps1,
        )
        if quad is None:
            self.get_logger().warn("pile quad not found")
            return RESULT_NOT_FOUND, None

        mask = self._fill_quad_interior(mask, quad)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        masked = self._bgr_masked_by_binary_mask(cv_image, mask)

        warp_sz = int(self.get_parameter("pile_warp_size").value)
        if warp_sz < 32:
            warp_sz = 256

        warped = self._warp_pile_to_square(masked, quad, warp_sz)
        vis = self._draw_pile_quad(cv_image, quad)

        try:
            score, _binary_mask, anomaly_map = self.detector.predict(warped)
        except Exception as e:
            self.get_logger().error(f"anomaly detector failed: {e}")
            return RESULT_NOT_FOUND, None

        self.get_logger().info(f"anomaly score={score:.4f} threshold={self.detector.score_threshold}")

        peak_thr = float(self.get_parameter("anomaly_peak_mask_threshold").value)
        peak_mask = (anomaly_map > peak_thr).astype(np.uint8) * 255
        if peak_mask.shape[:2] != warped.shape[:2]:
            peak_mask = cv2.resize(
                peak_mask,
                (warped.shape[1], warped.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

        hm_u8 = (anomaly_map * 255).astype(np.uint8)
        if hm_u8.shape[:2] != warped.shape[:2]:
            hm_u8 = cv2.resize(
                hm_u8,
                (warped.shape[1], warped.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            )
        heatmap = cv2.applyColorMap(hm_u8, cv2.COLORMAP_JET)
        overlay = cv2.addWeighted(warped, 0.6, heatmap, 0.4, 0)

        images = {
            "pile_warp": warped,
            "anomaly_mask": peak_mask,
        }

        if score <= self.detector.score_threshold:
            return RESULT_NOT_DEFECTED, images

        images.update({
            "detect_anomalies": vis,
            "intensity_mask": mask_bgr,
            "anomaly_heatmap": overlay,
        })
        return RESULT_DEFECTED, images

    def _save_result_images(self, images: dict[str, np.ndarray]) -> dict[str, str]:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
        out_dir = os.path.join(self._output_root, stamp)
        os.makedirs(out_dir, exist_ok=True)
        paths = {}
        for name, img in images.items():
            path = os.path.join(out_dir, f"{name}.png")
            if not cv2.imwrite(path, img):
                raise OSError(f"failed to write {path}")
            paths[name] = path
        self.get_logger().info(f"saved anomaly detection images to {out_dir}")
        return paths

    def _handle_detect(self, request, response):
        del request
        response.result = RESULT_NOT_FOUND
        response.pile_path = ""
        response.mask_path = ""

        msg = self._wait_fresh_frame(previous_stamp_ns=None)
        if msg is None:
            self.get_logger().warn(f"no frame on {self._image_topic} within {self._frame_timeout}s")
            return response

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return response

        if cv_image is None or cv_image.size == 0:
            self.get_logger().warn("empty image")
            return response

        result, images = self._run_pipeline(cv_image)
        if images is not None:
            try:
                paths = self._save_result_images(images)
            except OSError as e:
                self.get_logger().error(str(e))
                return response
            response.pile_path = paths.get("pile_warp", "")
            response.mask_path = paths.get("anomaly_mask", "")

        response.result = result
        self.get_logger().info(f"detect_anomalies -> {response.result}")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectAnomaliesNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
