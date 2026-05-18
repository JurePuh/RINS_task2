from dataclasses import dataclass


@dataclass
class Pose:
    """2D pose in the map frame. `theta` is yaw in radians."""
    x: float
    y: float
    theta: float = 0.0
