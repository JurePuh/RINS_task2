"""Tiny data classes for the movement node.

Kept small on purpose: this rewrite only needs a 2D pose. Anything richer
(Face, Ring, Subject) will come back when those behaviours are added.
"""

from dataclasses import dataclass


@dataclass
class Pose:
    """2D pose in the map frame. `theta` is yaw in radians."""
    x: float
    y: float
    theta: float = 0.0
