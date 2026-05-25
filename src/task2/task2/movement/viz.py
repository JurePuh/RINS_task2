"""Centralised RViz markers for movement-known entities (people, rings, barrels).

The detection nodes deliberately publish no markers; everything you see in RViz
comes through here so identity + classification info live in one place.

Marker identity scheme (rviz keys markers by (ns, id), updating in place on
re-publish):

  ns="people",  id=face_id*2        - white CUBE
  ns="people",  id=face_id*2+1      - "name (gender)" TEXT, offset above
  ns="rings",   id=ring_id          - flat CYLINDER in ring's color
  ns="barrels", id=barrel_id*3      - tall CYLINDER in barrel's color
  ns="barrels", id=barrel_id*3+1    - "horiz"/"vert" TEXT
  ns="barrels", id=barrel_id*3+2    - "LEAKING" / "dry" TEXT
"""

from __future__ import annotations

from dataclasses import dataclass

import rclpy.node
from visualization_msgs.msg import Marker, MarkerArray


_TOPIC = "/z_vizualization_markers"
_FRAME = "map"

# Color-name -> RGBA. Unknown colors fall back to gray.
_COLOR_NAME_TO_RGBA: dict[str, tuple[float, float, float, float]] = {
    "red":    (1.0, 0.0, 0.0, 1.0),
    "green":  (0.0, 1.0, 0.0, 1.0),
    "blue":   (0.0, 0.3, 1.0, 1.0),
    "yellow": (1.0, 1.0, 0.0, 1.0),
    "black":  (0.05, 0.05, 0.05, 1.0),
    "white":  (1.0, 1.0, 1.0, 1.0),
}
_GRAY_RGBA = (0.5, 0.5, 0.5, 1.0)
_WHITE_RGBA = (1.0, 1.0, 1.0, 1.0)
_TEXT_RGBA = (0.0, 0.0, 0.0, 1.0)
_LEAKING_RGBA = (1.0, 0.0, 0.0, 1.0)
_DRY_RGBA = (0.0, 0.8, 0.0, 1.0)


# Cached last-known position so behaviour-side updates (label/leak) don't need
# to re-supply coordinates the detection callback already gave us.
@dataclass
class _PersonState:
    x: float
    y: float

@dataclass
class _BarrelState:
    x: float
    y: float


class Visualizer:
    def __init__(self, node: rclpy.node.Node) -> None:
        self._node = node
        self._pub = node.create_publisher(MarkerArray, _TOPIC, 10)
        self._people: dict[int, _PersonState] = {}
        self._barrels: dict[int, _BarrelState] = {}

    # --- People ---

    def update_person(self, face_id: int, x: float, y: float) -> None:
        self._people[face_id] = _PersonState(x, y)
        cube = self._make_marker(
            ns="people", id=face_id * 2, marker_type=Marker.CUBE,
            x=x, y=y, z=0.15,
            scale=(0.25, 0.25, 0.25), rgba=_WHITE_RGBA,
        )
        self._publish([cube])

    def set_person_label(self, face_id: int, name: str, gender: str | None) -> None:
        state = self._people.get(face_id)
        if state is None:
            # No position cached yet; skip silently (callback hasn't fired).
            return
        text = f"{name} ({gender})" if gender else name
        label = self._text_marker(
            ns="people", id=face_id * 2 + 1,
            x=state.x, y=state.y, z=0.55,
            text=text, rgba=_TEXT_RGBA,
        )
        self._publish([label])

    # --- Rings ---

    def update_ring(self, ring_id: int, x: float, y: float, color: str) -> None:
        # Flat disk in the ring's color.
        disk = self._make_marker(
            ns="rings", id=ring_id, marker_type=Marker.CYLINDER,
            x=x, y=y, z=0.9,
            scale=(0.3, 0.3, 0.05), rgba=self._color_to_rgba(color),
        )
        self._publish([disk])

    # --- Barrels ---

    def update_barrel(
        self, barrel_id: int, x: float, y: float, color: str, horizontal: bool,
    ) -> None:
        self._barrels[barrel_id] = _BarrelState(x, y)
        # Tall cylinder body + orientation text above.
        body = self._make_marker(
            ns="barrels", id=barrel_id * 3, marker_type=Marker.CYLINDER,
            x=x, y=y, z=0.25,
            scale=(0.35, 0.35, 0.5), rgba=self._color_to_rgba(color),
        )
        orient = self._text_marker(
            ns="barrels", id=barrel_id * 3 + 1,
            x=x, y=y, z=0.75,
            text="horiz" if horizontal else "vert", rgba=_TEXT_RGBA,
        )
        self._publish([body, orient])

    def set_barrel_leak(self, barrel_id: int, leaking: bool) -> None:
        state = self._barrels.get(barrel_id)
        if state is None:
            return
        text = "LEAKING" if leaking else "dry"
        rgba = _LEAKING_RGBA if leaking else _DRY_RGBA
        leak = self._text_marker(
            ns="barrels", id=barrel_id * 3 + 2,
            x=state.x, y=state.y, z=0.95,
            text=text, rgba=rgba,
        )
        self._publish([leak])

    # --- Private helpers ---

    def _publish(self, markers: list[Marker]) -> None:
        self._pub.publish(MarkerArray(markers=markers))

    def _make_marker(
        self,
        ns: str,
        id: int,
        marker_type: int,
        x: float,
        y: float,
        z: float,
        scale: tuple[float, float, float],
        rgba: tuple[float, float, float, float],
    ) -> Marker:
        m = Marker()
        m.header.frame_id = _FRAME
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns = ns
        m.id = int(id)
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        # Identity orientation — RViz rejects (0,0,0,0) as invalid.
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _text_marker(
        self,
        ns: str,
        id: int,
        x: float,
        y: float,
        z: float,
        text: str,
        rgba: tuple[float, float, float, float],
    ) -> Marker:
        m = self._make_marker(
            ns=ns, id=id, marker_type=Marker.TEXT_VIEW_FACING,
            x=x, y=y, z=z,
            scale=(0.0, 0.0, 0.2),  # only scale.z matters for text height
            rgba=rgba,
        )
        m.text = text
        return m

    @staticmethod
    def _color_to_rgba(name: str) -> tuple[float, float, float, float]:
        return _COLOR_NAME_TO_RGBA.get(name.lower(), _GRAY_RGBA)
