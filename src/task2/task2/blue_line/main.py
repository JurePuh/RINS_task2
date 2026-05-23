"""BlueLine node: detect a downward-facing view of a blue line and publish offsets.

Subscribes to the top-down camera, masks blue on a single horizontal scanline,
extracts contiguous blue segments, and publishes `BlueLineStatus` on `/blue_line`:

Offsets are normalized to [-1, 1] across the image width.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image

from msg_types.msg import BlueLineStatus


# --- Topics ------------------------------------------------------------------

_IMAGE_TOPIC: str = "/top_camera/rgb/preview/image_raw"
_STATUS_TOPIC: str = "/blue_line"

# --- Detection parameters ----------------------------------------------------

# Scanline: fraction of image height to sample (0 = top, 1 = bottom).
_SCANLINE_FRAC: float = 0.75

# TODO: tune HSV thresholds against the simulator output.
_HSV_LOW: np.ndarray = np.array([90, 100, 255], dtype=np.uint8)
_HSV_HIGH: np.ndarray = np.array([120, 130, 255], dtype=np.uint8)

# Segment extraction tolerances on the masked scanline.
_MIN_SEG_PX: int = 10  # drop runs shorter than this
_MAX_GAP_PX: int = 5   # merge runs separated by <= this many off-pixels

# --- Debug visualization ----------------------------------------------------

_DEBUG_WINDOW: str = "blue_line debug"
_SCANLINE_THICKNESS: int = 3  # px height of the drawn segments overlay


# --- Internal types ----------------------------------------------------------

class LineState(IntEnum):
    """Mirror of BlueLineStatus.STATE_* constants for internal use."""
    LOST = 0
    LINE = 1
    CROSSROAD = 2


@dataclass(frozen=True)
class Segment:
    """Contiguous run of blue pixels on the scanline (inclusive indices)."""
    start: int
    end: int

    def width(self) -> int:
        return self.end - self.start + 1

    def center(self) -> float:
        return (self.start + self.end) / 2.0


# --- Pure helpers ------------------------------------------------------------

def _extract_segments(row_mask: np.ndarray) -> list[Segment]:
    """Extract blue-pixel runs from a 1-D boolean/uint8 row mask.

    Merges runs separated by <= `_MAX_GAP_PX` off-pixels, then drops any
    resulting run shorter than `_MIN_SEG_PX`. Pure function, no ROS deps.
    """
    # Find raw contiguous runs of "on" pixels.
    mask: np.ndarray = (row_mask > 0).astype(np.int8)
    if mask.size == 0:
        return []

    # Edge detection via diff; pad to catch runs touching the borders.
    padded: np.ndarray = np.concatenate(([0], mask, [0]))
    diffs: np.ndarray = np.diff(padded)
    starts: np.ndarray = np.where(diffs == 1)[0]
    ends: np.ndarray = np.where(diffs == -1)[0] - 1

    if starts.size == 0:
        return []

    # Merge runs whose inter-gap is small enough.
    merged: list[Segment] = []
    cur_start: int = int(starts[0])
    cur_end: int = int(ends[0])
    for s, e in zip(starts[1:], ends[1:]):
        gap: int = int(s) - cur_end - 1
        if gap <= _MAX_GAP_PX:
            cur_end = int(e)
        else:
            merged.append(Segment(cur_start, cur_end))
            cur_start, cur_end = int(s), int(e)
    merged.append(Segment(cur_start, cur_end))

    # Filter out too-thin runs (noise).
    return [seg for seg in merged if seg.width() >= _MIN_SEG_PX]


def _to_normalized(px: float, width: int) -> float:
    """Map pixel x in [0, width] to [-1, 1], clamped."""
    if width <= 0:
        return 0.0
    value: float = 2.0 * px / float(width) - 1.0
    # Clamp to the documented range.
    return max(-1.0, min(1.0, value))


# --- Node --------------------------------------------------------------------

class BlueLineNode(Node):
    """Publishes blue-line tracking offsets derived from the top-down camera."""

    def __init__(self) -> None:
        super().__init__("blue_line")
        # Cache the logger for nicer call sites.
        self._logger = self.get_logger()
        self._bridge: CvBridge = CvBridge()

        # Track previous classified state for transition logging.
        self._prev_state: LineState = LineState.LOST
        # Frame counter for periodic DEBUG heartbeats.
        self._frame_idx: int = 0

        # Subscribe to the downward camera feed.
        self._image_sub: Subscription = self.create_subscription(
            Image, _IMAGE_TOPIC, self._on_image, 10
        )
        # Publish the per-frame tracking status.
        self._status_pub: Publisher = self.create_publisher(
            BlueLineStatus, _STATUS_TOPIC, 10
        )

        # Standalone OpenCV debug window.
        cv2.namedWindow(_DEBUG_WINDOW, cv2.WINDOW_NORMAL)

        self._logger.info(
            f"BlueLineNode up: subscribing {_IMAGE_TOPIC}, publishing {_STATUS_TOPIC}"
        )

    def _on_image(self, msg: Image) -> None:
        # Convert ROS image to BGR; bail out on any bridge error.
        try:
            bgr: np.ndarray = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self._logger.warn(f"cv_bridge failure: {exc}")
            return

        img_h, img_w = bgr.shape[:2]
        if img_h == 0 or img_w == 0:
            self._logger.warn(f"Empty image received: {img_h}x{img_w}")
            return

        self._frame_idx += 1

        # Full-image blue mask drives both visualization and the scanline read.
        hsv: np.ndarray = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        full_mask: np.ndarray = cv2.inRange(hsv, _HSV_LOW, _HSV_HIGH)

        row_index: int = int(img_h * _SCANLINE_FRAC)
        row_index = min(max(row_index, 0), img_h - 1)
        row_mask: np.ndarray = full_mask[row_index, :]

        segments: list[Segment] = _extract_segments(row_mask)
        status: BlueLineStatus = self._classify(segments, img_w)
        self._status_pub.publish(status)

        # Per-frame DEBUG heartbeat (throttled).
        if self._frame_idx % 30 == 0:
            self._logger.debug(
                f"frame={self._frame_idx} state={LineState(status.state).name} "
                f"segs={len(segments)} L={status.offset_left:+.2f} "
                f"R={status.offset_right:+.2f}"
            )

        # Log state transitions with the values that caused them.
        new_state = LineState(status.state)
        if new_state != self._prev_state:
            self._log_transition(self._prev_state, new_state, status, segments)
            self._prev_state = new_state

        self._render_debug(bgr, full_mask, row_index, segments)

    def _log_transition(
        self,
        old: LineState,
        new: LineState,
        status: BlueLineStatus,
        segments: list[Segment],
    ) -> None:
        """Emit a human-readable INFO line whenever the tracker state changes."""
        seg_summary: str = ", ".join(
            f"[{s.start}-{s.end} w={s.width()}]" for s in segments
        ) or "<none>"
        self._logger.info(
            f"state {old.name} -> {new.name} | "
            f"L={status.offset_left:+.3f} R={status.offset_right:+.3f} "
            f"segs={seg_summary}"
        )

    def _render_debug(
        self,
        bgr: np.ndarray,
        full_mask: np.ndarray,
        row_index: int,
        segments: list[Segment],
    ) -> None:
        """Show a debug window: blue line painted black, segments drawn blue."""
        vis: np.ndarray = bgr.copy()
        # Paint detected blue pixels black so the line "disappears".
        vis[full_mask > 0] = (0, 0, 0)

        # Draw a thin gray guide along the scanline.
        cv2.line(vis, (0, row_index), (vis.shape[1] - 1, row_index), (80, 80, 80), 1)

        # Overlay the kept segments in pure blue at the scanline.
        half: int = max(_SCANLINE_THICKNESS // 2, 1)
        y0: int = max(row_index - half, 0)
        y1: int = min(row_index + half + 1, vis.shape[0])
        for seg in segments:
            vis[y0:y1, seg.start:seg.end + 1] = (255, 0, 0)

        cv2.imshow(_DEBUG_WINDOW, vis)
        # Required for OpenCV to actually pump the GUI event loop.
        cv2.waitKey(1)

    def _classify(self, segments: list[Segment], width: int) -> BlueLineStatus:
        """Map detected segments to a BlueLineStatus message."""
        msg = BlueLineStatus()

        if len(segments) == 0:
            # Nothing visible: report LOST with zeroed offsets.
            msg.state = BlueLineStatus.STATE_LOST
            msg.offset_left = 0.0
            msg.offset_right = 0.0
            return msg

        if len(segments) == 1:
            # Single line: both offsets collapse to the same center.
            seg = segments[0]
            offset: float = _to_normalized(seg.center(), width)
            msg.state = BlueLineStatus.STATE_LINE
            msg.offset_left = offset
            msg.offset_right = offset
            return msg

        # Crossroad: pick the two widest, then order by x position.
        widest: list[Segment] = sorted(segments, key=lambda s: s.width(), reverse=True)[:2]
        left, right = sorted(widest, key=lambda s: s.start)
        msg.state = BlueLineStatus.STATE_CROSSROAD
        msg.offset_left = _to_normalized(left.center(), width)
        msg.offset_right = _to_normalized(right.center(), width)
        return msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BlueLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
