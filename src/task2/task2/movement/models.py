from dataclasses import dataclass, field
from enum import Enum


# --- POSE ---

@dataclass
class Point:
    """2D point in the map frame."""
    x: float
    y: float


@dataclass
class Vector:
    """2D vector in the map frame."""
    x: float
    y: float


@dataclass
class Pose:
    """2D pose in the map frame. `theta` is yaw in radians."""
    x: float
    y: float
    theta: float = 0.0


# --- STUFF ---


@dataclass
class Ring:
    point: Point
    color: str
    id: int

    @property
    def x(self) -> float:
        return self.point.x
    
    @property
    def y(self) -> float:
        return self.point.y


@dataclass
class Barrel:
    id: int
    point: Point
    color: str
    horizontal: bool
    leaking: bool | None = None
    image_path: str | None = None

    @property
    def x(self) -> float:
        return self.point.x
    
    @property
    def y(self) -> float:
        return self.point.y

class Gender(Enum):
    MALE = "male"
    FEMALE = "female"

@dataclass
class Person:
    point: Point
    face_id: int
    name: str = ""
    role: str = ""
    gender: Gender | None = None

    @property
    def x(self) -> float:
        return self.point.x
    
    @property
    def y(self) -> float:
        return self.point.y


@dataclass
class Tile:
    index: int
    broken: bool | None = None
    image_path: str | None = None
    mask_path: str | None = None


# --- TASKS ---


@dataclass
class Task:
    requesters: list[Person] = field(default_factory=list)

    @property
    def was_asked_for(self) -> bool:
        return bool(self.requesters)


@dataclass
class CountRingsTask(Task):
    rings: list[Ring] = field(default_factory=list)

    @property
    def rings_by_color(self) -> dict[str, list[Ring]]:
        result: dict[str, list[Ring]] = {}
        for ring in self.rings:
            result.setdefault(ring.color, []).append(ring)
        return result


@dataclass
class InspectBarrelsTask(Task):
    barrels: list[Barrel] = field(default_factory=list)

    @property
    def leaking_barrels(self) -> list[Barrel]:
        return [b for b in self.barrels if b.leaking]


@dataclass
class AnomalyTask(Task):
    tiles: list[Tile] = field(default_factory=list)

    @property
    def broken_tiles(self) -> list[Tile]:
        return [t for t in self.tiles if t.broken]

