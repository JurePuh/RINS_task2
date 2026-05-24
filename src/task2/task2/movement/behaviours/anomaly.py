"""Per-color anomaly inspection sub-trees.

Two independent pipelines (red, green) — each owns its belt geometry and tile
count. The "is this color currently active" flag is set elsewhere (face
interaction); this module only consumes it and clears it on completion.
"""

import math
import time

import py_trees
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.client import Client
from rclpy.task import Future

from msg_types.srv import DetectAnomalies

from task2.movement import blackboard as bb
from task2.movement.behaviours._arm import SetArmPosition
from task2.movement.behaviours._nav import LoggingNavWaypoint, build_nav_goal
from task2.movement.behaviours._odom_move import (
    ApproachToWallDistance,
    DriveTileWithCorrection,
    SpinByYaw,
)
from task2.movement.behaviours.follow_path import Pose
from task2.movement.log_utils import log_throttled
from task2.movement.models import Tile, AnomalyTask



# --- Hardcoded geometry / counts --------------------------------------------
_RED_BELT_POSE = Pose(0.18, -4.65, -1.56)
_GREEN_BELT_POSE = Pose(-4.60, -2.42, -3.11)

_RED_TILE_COUNT = 4
_GREEN_TILE_COUNT = 5

_TILE_STEP_RED_M = 0.55 # forward step between tiles
_TILE_STEP_GREEN_M = 0.67 # forward step between tiles

_BELT_DRIVE_DIST = 0.3 # How far from belt we drive
_BELT_TURN_TO_DRIVE_YAW = -math.pi / 2 # What direction we spin at belt
_BELT_DRIVE_LATERAL_DIR_RAD = math.pi / 2 # Which direction is "laterally towards the belt" (relative to drive dir)


_TASK_KEY_BY_COLOR = {
    "red": bb.TASK_ANOMALY_RED,
    "green": bb.TASK_ANOMALY_GREEN,
}
_ACTIVE_KEY_BY_COLOR = {
    "red": bb.ANOMALY_RED_ACTIVE,
    "green": bb.ANOMALY_GREEN_ACTIVE,
}


class GoToBelt(LoggingNavWaypoint):
    """Navigate to the hardcoded belt-approach pose for this color."""

    def __init__(self, color: str, pose: Pose, name: str | None = None):
        super().__init__(
            name=name or f"GoToBelt({color})",
            action_goal=build_nav_goal(pose),
        )


_BROKEN_BY_RESULT = {
    "defected": True,
    "not_defected": False,
    "not_found": None,
}


class CallAnomalyService(py_trees.behaviour.Behaviour):
    """Call /detect_anomalies; append a Tile with the result to this color's task."""

    def __init__(self, color: str, name: str | None = None):
        super().__init__(name=name or f"CallAnomalyService({color})")
        self._color = color
        self._task_key = _TASK_KEY_BY_COLOR[color]
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=self._task_key, access=py_trees.common.Access.READ)
        self._future: Future | None = None

    def setup(self, **kwargs):
        self._node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self._node.get_logger()
        self._client: Client = self._node.create_client(DetectAnomalies, "/detect_anomalies")

    def initialise(self):
        task: AnomalyTask = self.bb.get(self._task_key)
        next_idx = len(task.tiles)
        self._ros_logger.info(
            f"{self.name}: calling /detect_anomalies for color='{self._color}' "
            f"tile_idx={next_idx}"
        )
        self._ros_logger.debug(
            f"{self.name}: request payload = DetectAnomalies.Request() (empty); "
            f"existing tiles={[t.broken for t in task.tiles]}"
        )
        time.sleep(0.3) # Give the system a moment to settle before calling the service
        self._future = self._client.call_async(DetectAnomalies.Request())

    def update(self) -> py_trees.common.Status:
        if self._future is None:
            return py_trees.common.Status.FAILURE
        if not self._future.done():
            log_throttled(
                self._ros_logger, self._node, f"{self.name}.waiting", "debug",
                f"{self.name}: waiting for /detect_anomalies response",
            )
            return py_trees.common.Status.RUNNING

        # Update the task with the result
        result_str = self._future.result().result # type: ignore
        broken = _BROKEN_BY_RESULT.get(result_str)
        task: AnomalyTask = self.bb.get(self._task_key)
        tile = Tile(index=len(task.tiles), broken=broken)
        task.tiles.append(tile)
        self._ros_logger.info(
            f"{self.name}: result='{result_str}' -> tile {tile.index} broken={broken} "
            f"(total tiles for {self._color}: {len(task.tiles)})"
        )

        time.sleep(0.3) # Give the system a moment to settle before proceeding
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if (
            new_status == py_trees.common.Status.INVALID
            and self._future is not None
            and not self._future.done()
        ):
            self._future.cancel()
        self._future = None


class MarkAnomalyInactive(py_trees.behaviour.Behaviour):
    """Clear this color's ACTIVE flag once the sequence is done."""

    def __init__(self, color: str, name: str | None = None):
        super().__init__(name=name or f"MarkAnomalyInactive({color})")
        self._color = color
        self._flag_key = _ACTIVE_KEY_BY_COLOR[color]
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=self._flag_key, access=py_trees.common.Access.WRITE)
        self.bb.register_key(key=self._flag_key, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        try:
            prev = self.bb.get(self._flag_key)
        except Exception:
            prev = "?"
        self.bb.set(self._flag_key, False)
        self._ros_logger.info(
            f"{self.name}: cleared {self._flag_key} (was {prev} -> False)"
        )
        return py_trees.common.Status.SUCCESS


def _build_for_color(color: str, belt_pose: Pose, tile_count: int, tile_step_distance: float) -> py_trees.composites.Sequence:
    seq = py_trees.composites.Sequence(name=f"Anomaly{color.capitalize()}", memory=True)
    seq.add_child(GoToBelt(color, belt_pose))
    seq.add_child(ApproachToWallDistance(target_distance_m=_BELT_DRIVE_DIST))
    seq.add_child(SpinByYaw(target_yaw_delta_rad=_BELT_TURN_TO_DRIVE_YAW))
    seq.add_child(SetArmPosition("look_at_belt_left"))

    seq.add_child(CallAnomalyService(color, name=f"Detect{0}({color})"))
    for i in range(1, tile_count):
        seq.add_child(DriveTileWithCorrection(
            distance_m=tile_step_distance,
            target_lateral_m=_BELT_DRIVE_DIST,
            lateral_direction_rad=_BELT_DRIVE_LATERAL_DIR_RAD,
            name=f"Step{i}({color})",
        ))
        seq.add_child(CallAnomalyService(color, name=f"Detect{i}({color})"))
    
    seq.add_child(MarkAnomalyInactive(color))
    return seq


def build_red() -> py_trees.composites.Sequence:
    return _build_for_color("red", _RED_BELT_POSE, _RED_TILE_COUNT, _TILE_STEP_RED_M)


def build_green() -> py_trees.composites.Sequence:
    return _build_for_color("green", _GREEN_BELT_POSE, _GREEN_TILE_COUNT, _TILE_STEP_GREEN_M)
