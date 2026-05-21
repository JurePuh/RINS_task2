"""Per-color anomaly inspection sub-trees.

Two independent pipelines (red, green) — each owns its belt geometry and tile
count. The "is this color currently active" flag is set elsewhere (face
interaction); this module only consumes it and clears it on completion.
"""

import time

import py_trees
import py_trees_ros
from nav2_msgs.action import DriveOnHeading, Spin
from std_msgs.msg import String
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.client import Client
from rclpy.task import Future

from msg_types.srv import DetectAnomalies

from task2.movement import blackboard as bb
from task2.movement.behaviours._nav import LoggingNavWaypoint, build_nav_goal
from task2.movement.behaviours.follow_path import Pose
from task2.movement.models import Tile, AnomalyTask



# --- Hardcoded geometry / counts --------------------------------------------
_RED_BELT_POSE = Pose(x=0.0, y=0.0, theta=0.0)    # TODO: set real coords
_GREEN_BELT_POSE = Pose(x=0.0, y=0.0, theta=0.0)  # TODO: set real coords

_RED_TILE_COUNT = 4
_GREEN_TILE_COUNT = 5

_TILE_STEP_M = 0.3  # forward step between tiles

_ARM_SETTLE_SEC = 1.0


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


class AlignToBelt(py_trees_ros.action_clients.FromConstant):
    """Rotate 90deg CCW in place using nav2's Spin recovery action."""

    def __init__(self, name: str = "AlignToBelt"):
        import math
        goal = Spin.Goal()
        goal.target_yaw = math.pi / 2.0
        super().__init__(
            name=name,
            action_type=Spin,
            action_name="spin",
            action_goal=goal,
        )

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def initialise(self):
        super().initialise()
        self._ros_logger.info(f"{self.name}: spin +90deg")


class SetArmPosition(py_trees.behaviour.Behaviour):
    """Publish an arm pose name on /arm_command, then RUNNING for ~1s to settle."""

    def __init__(self, pose_string: str, name: str | None = None):
        super().__init__(name=name or f"SetArmPosition({pose_string})")
        self._pose_string = pose_string
        self._deadline: float | None = None

    def setup(self, **kwargs):
        node = kwargs["node"]
        self._ros_logger: RcutilsLogger = node.get_logger()
        self._publisher: Publisher = node.create_publisher(String, "/arm_command", 10)

    def initialise(self):
        msg = String()
        msg.data = self._pose_string
        self._publisher.publish(msg)
        self._ros_logger.info(f"{self.name}: published '{self._pose_string}'")
        # To make the node "running" for a bit, while arm adjusts
        self._deadline = time.monotonic() + _ARM_SETTLE_SEC

    def update(self) -> py_trees.common.Status:
        # Return "RUNNING" until the settle time has passed, then "SUCCESS"
        if self._deadline is None or time.monotonic() >= self._deadline:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._deadline = None


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
        node = kwargs["node"]
        self._ros_logger: RcutilsLogger = node.get_logger()
        self._client: Client = node.create_client(DetectAnomalies, "/detect_anomalies")

    def initialise(self):
        # Call service to detect anomaly in tile
        self._future = self._client.call_async(DetectAnomalies.Request())

    def update(self) -> py_trees.common.Status:
        if self._future is None:
            return py_trees.common.Status.FAILURE
        if not self._future.done():
            return py_trees.common.Status.RUNNING

        # Update the task with the result
        result_str = self._future.result().result # type: ignore
        broken = _BROKEN_BY_RESULT.get(result_str)
        task: AnomalyTask = self.bb.get(self._task_key)
        tile = Tile(index=len(task.tiles), broken=broken)
        task.tiles.append(tile)
        self._ros_logger.info(
            f"{self.name}: result='{result_str}' -> tile {tile.index} broken={broken}"
        )
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if (
            new_status == py_trees.common.Status.INVALID
            and self._future is not None
            and not self._future.done()
        ):
            self._future.cancel()
        self._future = None


class MoveToNextTile(py_trees_ros.action_clients.FromConstant):
    """Drive forward a fixed distance using nav2's DriveOnHeading recovery action."""

    def __init__(self, distance_m: float, name: str | None = None):
        goal = DriveOnHeading.Goal()
        goal.target.x = float(distance_m)
        super().__init__(
            name=name or f"MoveToNextTile({distance_m:.2f}m)",
            action_type=DriveOnHeading,
            action_name="drive_on_heading",
            action_goal=goal,
        )

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def initialise(self):
        super().initialise()
        self._ros_logger.info(f"{self.name}: drive forward")


class MarkAnomalyInactive(py_trees.behaviour.Behaviour):
    """Clear this color's ACTIVE flag once the sequence is done."""

    def __init__(self, color: str, name: str | None = None):
        super().__init__(name=name or f"MarkAnomalyInactive({color})")
        self._color = color
        self._flag_key = _ACTIVE_KEY_BY_COLOR[color]
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=self._flag_key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        self.bb.set(self._flag_key, False)
        self._ros_logger.info(f"{self.name}: cleared {self._flag_key}")
        return py_trees.common.Status.SUCCESS


def _build_for_color(color: str, belt_pose: Pose, tile_count: int) -> py_trees.composites.Sequence:
    seq = py_trees.composites.Sequence(name=f"Anomaly{color.capitalize()}", memory=True)
    seq.add_child(GoToBelt(color, belt_pose))
    seq.add_child(AlignToBelt())
    seq.add_child(SetArmPosition("look_at_belt_left"))
    for i in range(tile_count):
        seq.add_child(CallAnomalyService(color, name=f"Detect{i}({color})"))
        seq.add_child(MoveToNextTile(_TILE_STEP_M, name=f"Step{i}({color})"))
    seq.add_child(MarkAnomalyInactive(color))
    return seq


def build_red() -> py_trees.composites.Sequence:
    return _build_for_color("red", _RED_BELT_POSE, _RED_TILE_COUNT)


def build_green() -> py_trees.composites.Sequence:
    return _build_for_color("green", _GREEN_BELT_POSE, _GREEN_TILE_COUNT)
