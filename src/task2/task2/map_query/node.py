from __future__ import annotations

from collections import deque
from typing import Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from msg_types.srv import WallNormalAt

from .exceptions import WallQueryError, WallQueryInternalError
from .models import Map

# ── Constants ──────────────────────────────────────────────────────────────────

# Half-side of the square window (in pixels) used to collect boundary pixels
# for the line fit.  A radius of 3 means a 7×7 pixel patch is searched.
_DEFAULT_NEIGHBOURHOOD_RADIUS = 3

# 4-connected neighbour offsets: up, down, left, right
_4_NEIGHBOURS = ((-1, 0), (1, 0), (0, -1), (0, 1))


class WallNormalAtNode(Node):
    """Service node that answers WallNormalAt queries.

    Given a (x, y) coordinate in the map frame, returns the position of the
    nearest wall-boundary pixel and the wall's outward unit normal vector.
    """

    def __init__(self) -> None:
        super().__init__("wall_normal_at_node")

        self._map: Optional[Map] = None

        # Subscribe with TRANSIENT_LOCAL so we receive the last published map
        # even if this node starts after the map server published it.
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._on_map, latched_qos
        )

        self._srv = self.create_service(
            WallNormalAt, "wall_normal_at", self._handle
        )

        # Visualization publishers (for debugging in RViz)
        self._normal_pub = self.create_publisher(
            Marker, "/wall_normal_at/normal", 10
        )
        self._fit_pts_pub = self.create_publisher(
            MarkerArray, "/wall_normal_at/fit_points", 10
        )

        self._logger = self.get_logger()

        self._logger.info("WallNormalAtNode ready")

    # ── Map subscription ──────────────────────────────────────────────────────

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = Map.from_occupancy_grid(msg)
        h, w = self._map.grid.shape
        self._logger.info(f"Map received: {w}×{h} px, resolution={self._map.resolution:.3f} m/px")

    # ── Service handler ───────────────────────────────────────────────────────

    def _handle(
        self, request: WallNormalAt.Request, response: WallNormalAt.Response
    ) -> WallNormalAt.Response:
        try:
            response = self._query(request, response)
        except WallQueryError as exc:
            # Expected failure: query cannot be answered right now
            self._logger.warn(f"WallNormalAt query failed: {exc}")
            response.success = False
        # WallQueryInternalError (and any other exception) propagates uncaught
        # so that programming errors crash loudly rather than being swallowed.
        return response

    def _query(
        self, request: WallNormalAt.Request, response: WallNormalAt.Response
    ) -> WallNormalAt.Response:
        if self._map is None:
            raise WallQueryError("Map not yet received")

        m = self._map

        # ── Step 1: world → pixel ─────────────────────────────────────────────
        row, col = m.world_to_pixel(request.x, request.y)

        if not m.in_bounds(row, col):
            raise WallQueryError(
                f"Query point ({request.x:.2f}, {request.y:.2f}) is outside the map"
            )

        # ── Step 2: nearest wall-boundary pixel ───────────────────────────────
        wall_row, wall_col = self._find_nearest_boundary(m, row, col)

        # ── Step 3: collect neighbourhood for line fitting ────────────────────
        fit_pixels = self._collect_neighbourhood(m, wall_row, wall_col)

        if len(fit_pixels) < 2:
            raise WallQueryError(
                "Not enough boundary pixels in neighbourhood to fit a line"
            )

        # ── Step 4: fit normal via PCA of pixel positions ─────────────────────
        nx, ny = self._fit_normal(fit_pixels)

        # ── Step 5: orient normal toward free space ───────────────────────────
        nx, ny = self._orient_normal(m, wall_row, wall_col, nx, ny)

        # ── Step 6: convert wall pixel to world coordinates ───────────────────
        wx, wy = m.pixel_to_world_centre(wall_row, wall_col)

        self._logger.info(f"WallNormalAt query at ({request.x:.2f}, {request.y:.2f}) → "
                     f"wall at ({wx:.2f}, {wy:.2f}) with normal ({nx:.2f}, {ny:.2f}), "
                     f"using {len(fit_pixels)} pixels for fitting")

        # ── Publish debug visualizations ──────────────────────────────────────
        self._publish_normal_marker(wx, wy, nx, ny)
        self._publish_fit_points(fit_pixels, m)

        response.success = True
        response.point_x = float(wx)
        response.point_y = float(wy)
        response.normal_x = float(nx)
        response.normal_y = float(ny)
        return response

    # ── Algorithm helpers ─────────────────────────────────────────────────────

    def _is_boundary_pixel(self, m: Map, row: int, col: int) -> bool:
        """True if (row, col) is not-free AND has at least one free 4-neighbour."""
        if not m.grid[row, col]:
            return False  # pixel is free; not a wall
        for dr, dc in _4_NEIGHBOURS:
            nr, nc = row + dr, col + dc
            if m.in_bounds(nr, nc) and not m.grid[nr, nc]:
                return True
        return False

    def _find_nearest_boundary(self, m: Map, row: int, col: int) -> tuple[int, int]:
        """BFS from (row, col) to find the nearest wall-boundary pixel."""
        visited: set[tuple[int, int]] = set()
        queue: deque[tuple[int, int]] = deque()
        queue.append((row, col))
        visited.add((row, col))

        # BFS 
        eight_neighbours = [
            (dr, dc)
            for dr in (-1, 0, 1)
            for dc in (-1, 0, 1)
            if (dr, dc) != (0, 0)
        ]

        while queue:
            r, c = queue.popleft()
            if self._is_boundary_pixel(m, r, c):
                return r, c
            for dr, dc in eight_neighbours:
                nr, nc = r + dr, c + dc
                if m.in_bounds(nr, nc) and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append((nr, nc))

        raise WallQueryError("No wall-boundary pixel found anywhere in the map")

    def _collect_neighbourhood(
        self, m: Map, row: int, col: int, radius: int = _DEFAULT_NEIGHBOURHOOD_RADIUS
    ) -> list[tuple[int, int]]:
        """Collect all boundary pixels within Chebyshev radius of (row, col)."""
        pixels: list[tuple[int, int]] = []
        r_range = range(
            max(0, row - radius),
            min(m.grid.shape[0], row + radius + 1),
        )
        c_range = range(
            max(0, col - radius),
            min(m.grid.shape[1], col + radius + 1),
        )
        for r in r_range:
            for c in c_range:
                if self._is_boundary_pixel(m, r, c):
                    pixels.append((r, c))
        return pixels

    def _fit_normal(self, pixels: list[tuple[int, int]]) -> tuple[float, float]:
        """Compute the wall normal via PCA of pixel positions.

        The eigenvector of the *smallest* eigenvalue of the covariance matrix
        is perpendicular to the dominant direction (the wall line), i.e. it is
        the normal direction we want.
        """
        # Use (col, row) as (x, y) for geometry; rows are the y-axis.
        pts = np.array([[c, r] for r, c in pixels], dtype=float)
        pts -= pts.mean(axis=0)  # centre at origin
        cov = pts.T @ pts
        _eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # eigh returns eigenvalues in ascending order; smallest = normal axis
        normal = eigenvectors[:, 0]
        length = np.linalg.norm(normal)
        if length < 1e-9:
            raise WallQueryInternalError("Normal vector is degenerate (zero length)")
        return float(normal[0] / length), float(normal[1] / length)

    def _orient_normal(
        self, m: Map, wall_row: int, wall_col: int, nx: float, ny: float
    ) -> tuple[float, float]:
        """Flip the normal if it points into the wall instead of toward free space."""
        dr, dc = self._free_neighbour_direction(m, wall_row, wall_col)
        # (dc, dr) is the toward-free vector in (x=col, y=row) space
        if nx * dc + ny * dr < 0:
            return -nx, -ny
        return nx, ny

    def _free_neighbour_direction(
        self, m: Map, row: int, col: int
    ) -> tuple[int, int]:
        """Return the (dr, dc) offset of the first free 4-neighbour of (row, col)."""
        for dr, dc in _4_NEIGHBOURS:
            nr, nc = row + dr, col + dc
            if m.in_bounds(nr, nc) and not m.grid[nr, nc]:
                return dr, dc
        raise WallQueryInternalError(
            f"Boundary pixel ({row}, {col}) has no free 4-neighbour — "
            "this contradicts the boundary definition"
        )

    # ── Visualization helpers ─────────────────────────────────────────────────

    def _make_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        return header

    def _publish_normal_marker(
        self, wx: float, wy: float, nx: float, ny: float
    ) -> None:
        """Publish an ARROW marker at the wall point, pointing in the normal direction."""
        # Arrow defined by two points: tail at wall position, tip offset by normal
        arrow_length = 0.4  # metres
        marker = Marker()
        marker.header = self._make_header()
        marker.ns = "wall_normal"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [
            Point(x=wx, y=wy, z=0.05),
            Point(x=wx + nx * arrow_length, y=wy + ny * arrow_length, z=0.05),
        ]
        marker.scale = Vector3(x=0.05, y=0.08, z=0.0)  # shaft diameter, head diameter
        marker.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0)  # orange
        marker.lifetime = Duration(sec=5)
        self._normal_pub.publish(marker)

    def _publish_fit_points(
        self, pixels: list[tuple[int, int]], m: Map
    ) -> None:
        """Publish a SPHERE for every pixel used in the line fit."""
        markers: list[Marker] = []

        # First delete old markers from previous queries
        delete_all = Marker()
        delete_all.header = self._make_header()
        delete_all.ns = "fit_points"
        delete_all.action = Marker.DELETEALL
        markers.append(delete_all)

        for i, (r, c) in enumerate(pixels):
            wx, wy = m.pixel_to_world_centre(r, c)
            dot = Marker()
            dot.header = self._make_header()
            dot.ns = "fit_points"
            dot.id = i + 1  # id 0 reserved for DELETEALL
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position = Point(x=wx, y=wy, z=0.05)
            dot.scale = Vector3(x=0.06, y=0.06, z=0.06)
            dot.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.9)  # cyan
            dot.lifetime = Duration(sec=5)
            markers.append(dot)

        self._fit_pts_pub.publish(MarkerArray(markers=markers))
