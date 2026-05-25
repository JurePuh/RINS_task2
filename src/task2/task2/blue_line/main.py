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
# This is the "back" row — the primary reference for line position.
_SCANLINE_FRAC: float = 0.71

# A second "front" scanline is sampled this many pixels *above* the back row.
# Used to compute a heading term (front - back): the line curves at corners
# before it shifts at the back row, so this gives the controller advance warning.
_SCANLINE_FRONT_OFFSET_PX: int = 10

# Weights for combining position (back row) and heading (front - back) into the
# published offset. Heading dominant, position small so the robot still keeps the
# line in frame on long straights.
_W_POS: float = 0.2
_W_HEAD: float = 4.0

# HSV thresholds for blue masking. Tune these if the line isn't detected reliably.
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
    
    def left(self) -> int:
        return self.start

    def right(self) -> int:
        return self.end


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

        back_row: int = int(img_h * _SCANLINE_FRAC)
        back_row = min(max(back_row, 0), img_h - 1)
        front_row: int = min(max(back_row - _SCANLINE_FRONT_OFFSET_PX, 0), img_h - 1)

        back_segs, back_state, back_l, back_r, back_ll, back_rr = self._row_offsets(full_mask, back_row, img_w)
        front_segs, front_state, front_l, front_r, front_ll, front_rr = self._row_offsets(full_mask, front_row, img_w)

        status = BlueLineStatus()
        # State classification always follows the back row (matches old behaviour).
        status.state = int(back_state)

        if front_state == LineState.LOST:
            # Graceful fallback: front row sees nothing, behave like the old
            # single-row controller using just the back row.
            status.offset_left = back_l
            status.offset_right = back_r
        else:
            # Stanley-style combine: position (back) + heading (front - back).
            status.offset_left = _W_POS * back_l + _W_HEAD * (front_ll - back_ll)
            status.offset_right = _W_POS * back_r + _W_HEAD * (front_rr - back_rr)

        self._status_pub.publish(status)

        # Per-frame DEBUG heartbeat (throttled).
        if self._frame_idx % 30 == 0:
            self._logger.debug(
                f"frame={self._frame_idx} state={LineState(status.state).name} "
                f"back_segs={len(back_segs)} front_segs={len(front_segs)} "
                f"L={status.offset_left:+.2f} R={status.offset_right:+.2f}"
            )

        # Log state transitions with the values that caused them.
        new_state = LineState(status.state)
        if new_state != self._prev_state:
            self._log_transition(self._prev_state, new_state, status, back_segs)
            self._prev_state = new_state

        self._render_debug(bgr, full_mask, back_row, front_row, back_segs, front_segs)

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
        back_row: int,
        front_row: int,
        back_segs: list[Segment],
        front_segs: list[Segment],
    ) -> None:
        """Show a debug window: blue line painted black, segments drawn blue."""
        vis: np.ndarray = bgr.copy()
        # Paint detected blue pixels black so the line "disappears".
        vis[full_mask > 0] = (0, 0, 0)

        # Draw thin gray guides along both scanlines.
        cv2.line(vis, (0, back_row), (vis.shape[1] - 1, back_row), (80, 80, 80), 1)
        cv2.line(vis, (0, front_row), (vis.shape[1] - 1, front_row), (80, 80, 80), 1)

        half: int = max(_SCANLINE_THICKNESS // 2, 1)
        # Back row in pure blue, front row in cyan to tell them apart.
        for row, segs, color in (
            (back_row, back_segs, (255, 0, 0)),
            (front_row, front_segs, (255, 255, 0)),
        ):
            y0: int = max(row - half, 0)
            y1: int = min(row + half + 1, vis.shape[0])
            for seg in segs:
                vis[y0:y1, seg.start:seg.end + 1] = color

        cv2.imshow(_DEBUG_WINDOW, vis)
        # Required for OpenCV to actually pump the GUI event loop.
        cv2.waitKey(1)

    def _row_offsets(
        self, full_mask: np.ndarray, row_index: int, width: int
    ) -> tuple[list[Segment], LineState, float, float, float, float]:
        """Sample one scanline and classify it. Returns (segments, state, L_center, R_center, L_left, R_right)."""
        row_mask: np.ndarray = full_mask[row_index, :]
        segments: list[Segment] = _extract_segments(row_mask)

        if len(segments) == 0:
            return segments, LineState.LOST, 0.0, 0.0, 0.0, 0.0

        if len(segments) == 1:
            seg = segments[0]
            offset_center: float = _to_normalized(seg.center(), width)
            offset_left: float = _to_normalized(seg.left(), width)
            offset_right: float = _to_normalized(seg.right(), width)
            return segments, LineState.LINE, offset_center, offset_center, offset_left, offset_right

        # Crossroad: pick the two widest, then order by x position.
        widest: list[Segment] = sorted(segments, key=lambda s: s.width(), reverse=True)[:2]
        left, right = sorted(widest, key=lambda s: s.start)
        return (
            segments,
            LineState.CROSSROAD,
            _to_normalized(left.center(), width),
            _to_normalized(right.center(), width),
            _to_normalized(left.left(), width),
            _to_normalized(right.right(), width),
        )


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
