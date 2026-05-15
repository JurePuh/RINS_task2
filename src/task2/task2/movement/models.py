
from enum import Enum
import threading



class Pose:
    """Represents the position and orientation of a subject in 2D space."""
    def __init__(self, x: float, y: float, theta: float | None = None):
        self.x = x
        self.y = y
        self.theta = theta

    def set_if_none(self, theta: float = 0.0) -> 'Pose':
        if self.theta is None:
            self.theta = theta
        return self


class Subject:
    """Base class for all subjects in the environment."""

    def __init__(self, id: int, pose: Pose):
        self.id = id
        
        self._pose_lock = threading.RLock()
        self._pose = pose
    
    @property
    def pose(self):
        with self._pose_lock:
            return self._pose
    
    @pose.setter
    def pose(self, value: Pose):
        with self._pose_lock:
            self._pose = value

    @property
    def type_str(self) -> str:
        return self.__class__.__name__


class Face(Subject):
    """Represents a face subject in the environment."""

    def __init__(self, id: int, pose: Pose):
        super().__init__(id, pose)


class Ring(Subject):
    """Represents a ring subject in the environment."""

    class Color(Enum):
        RED = 'red'
        GREEN = 'green'
        BLUE = 'blue'
        YELLOW = 'yellow'
        BLACK = 'black'

    def __init__(self, id: int, pose: Pose, color: 'Ring.Color'):
        super().__init__(id, pose)
        self.color = color
