from __future__ import annotations

import numpy as np
from nav_msgs.msg import OccupancyGrid


class Map:
    """Internal representation of an OccupancyGrid map.

    Pixels are stored as a boolean grid where True means "not free"
    (occupied or unknown) and False means "free". This lets us detect
    wall boundaries as edges between True and False pixels.

    Coordinate conventions
    ----------------------
    - pixel (row, col) corresponds to the world point
        x = origin_x + col * resolution
        y = origin_y + row * resolution
    - The *centre* of pixel (row, col) is at
        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution
    """

    def __init__(
        self,
        grid: np.ndarray,
        origin_x: float,
        origin_y: float,
        resolution: float,
    ) -> None:
        # grid shape: (height, width), dtype bool
        self.grid = grid
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def world_to_pixel(self, x: float, y: float) -> tuple[int, int]:
        """Convert world (x, y) to the nearest (row, col) pixel index."""
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def pixel_to_world_centre(self, row: int, col: int) -> tuple[float, float]:
        """Return the world coordinates of the centre of pixel (row, col)."""
        wx = self.origin_x + (col + 0.5) * self.resolution
        wy = self.origin_y + (row + 0.5) * self.resolution
        return wx, wy

    def in_bounds(self, row: int, col: int) -> bool:
        height, width = self.grid.shape
        return 0 <= row < height and 0 <= col < width

    # ── Factory ───────────────────────────────────────────────────────────────

    @classmethod
    def from_occupancy_grid(cls, msg: OccupancyGrid) -> "Map":
        """Build a Map from a nav_msgs/OccupancyGrid message.

        OccupancyGrid values:
          0   → free
          100 → occupied
         -1   → unknown
        We treat anything that is not free (0) as "not free" (True).
        """
        width = msg.info.width
        height = msg.info.height

        # msg.data is a flat row-major array; reshape to (height, width)
        raw = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # True = not free: occupied (100) or unknown (-1)
        grid = raw != 0

        origin = msg.info.origin.position
        return cls(
            grid=grid,
            origin_x=float(origin.x),
            origin_y=float(origin.y),
            resolution=float(msg.info.resolution),
        )
