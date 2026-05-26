"""Small condition leaves used by the priority Selector."""

import py_trees

from rclpy.impl.rcutils_logger import RcutilsLogger

from task2.movement import blackboard as bb
from task2.movement.log_utils import log_throttled


# --- ANOMALY TASKS ----

class _ActiveFlagPending(py_trees.behaviour.Behaviour):
    """SUCCESS iff the given boolean blackboard flag is True."""

    def __init__(self, name: str, flag_key: str):
        super().__init__(name=name)
        self._flag_key = flag_key
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=flag_key, access=py_trees.common.Access.READ)
        self._last_status: py_trees.common.Status | None = None

    def setup(self, **kwargs):
        self._node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self._node.get_logger()

    def update(self) -> py_trees.common.Status:
        flag = self.bb.get(self._flag_key)
        status = (
            py_trees.common.Status.SUCCESS if flag else py_trees.common.Status.FAILURE
        )
        if status != self._last_status:
            self._ros_logger.info(
                f"{self.name}: transition {self._last_status} -> {status.name} "
                f"({self._flag_key}={flag})"
            )
            self._last_status = status
        return status


class AnomalyRedActive(_ActiveFlagPending):
    def __init__(self, name: str = "RedAnomalyTaskPending"):
        super().__init__(name=name, flag_key=bb.ANOMALY_RED_ACTIVE)


class AnomalyGreenActive(_ActiveFlagPending):
    def __init__(self, name: str = "GreenAnomalyTaskPending"):
        super().__init__(name=name, flag_key=bb.ANOMALY_GREEN_ACTIVE)


# --- FACE TASKS ----

class HasUnhandledPerson(py_trees.behaviour.Behaviour):
    """SUCCESS iff the pending_people queue is non-empty."""

    def __init__(self, name: str = "HasUnhandledPerson"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_PEOPLE, access=py_trees.common.Access.READ)
        self._last_status: py_trees.common.Status | None = None

    def setup(self, **kwargs):
        self._node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self._node.get_logger()

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_PEOPLE)
        n = len(queue)
        status = (
            py_trees.common.Status.SUCCESS if queue else py_trees.common.Status.FAILURE
        )
        if status != self._last_status:
            self._ros_logger.info(
                f"{self.name}: transition {self._last_status} -> {status.name} "
                f"(pending_people={n})"
            )
            self._last_status = status
        return status


# --- BARREL TASKS ----

class BarrelVisitPending(py_trees.behaviour.Behaviour):
    """SUCCESS iff BARREL_ACTIVE is True AND PENDING_BARRELS is non-empty."""

    def __init__(self, name: str = "HasUnvisitedHorizontalBarrel"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.BARREL_ACTIVE, access=py_trees.common.Access.READ)
        self.bb.register_key(key=bb.PENDING_BARRELS, access=py_trees.common.Access.READ)
        self._last_status: py_trees.common.Status | None = None

    def setup(self, **kwargs):
        self._node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self._node.get_logger()
        self._log = self._ros_logger

    def update(self) -> py_trees.common.Status:
        active = self.bb.get(bb.BARREL_ACTIVE)
        queue = self.bb.get(bb.PENDING_BARRELS)
        status = (
            py_trees.common.Status.SUCCESS
            if (active and queue)
            else py_trees.common.Status.FAILURE
        )
        if status != self._last_status:
            self._ros_logger.info(
                f"{self.name}: transition {self._last_status} -> {status.name} "
                f"(barrel_active={active}, queue_size={len(queue) if queue else 0})"
            )
            self._last_status = status
        return status

class IsHeadBarrelLeaking(py_trees.behaviour.Behaviour):
    """SUCCESS iff PENDING_BARRELS[0].leaking is True."""

    def __init__(self, name: str = "IsHeadBarrelLeaking"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_BARRELS, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_BARRELS)
        if not queue:
            return py_trees.common.Status.FAILURE
        leaking = bool(queue[0].leaking)
        self._ros_logger.info(
            f"{self.name}: barrel {queue[0].id} leaking={leaking}"
        )
        return (
            py_trees.common.Status.SUCCESS if leaking else py_trees.common.Status.FAILURE
        )


# --- NAVIGATION ----

class RecomputeNotRequested(py_trees.behaviour.Behaviour):
    """SUCCESS while the recompute flag is False; FAILURE when it flips True.

    First child of a memory=False Sequence whose second child is a navigation
    behaviour. When the flag flips True mid-flight, this fails, the Sequence
    fails, and py_trees terminates the RUNNING nav sibling with INVALID — which
    triggers FromBlackboard.terminate() to cancel the nav2 goal.
    """

    def __init__(self, name: str, flag_key: str):
        super().__init__(name=name)
        self._flag_key = flag_key
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=flag_key, access=py_trees.common.Access.READ)
        self._last_status: py_trees.common.Status | None = None

    def setup(self, **kwargs):
        self._node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self._node.get_logger()

    def update(self) -> py_trees.common.Status:
        flag = self.bb.get(self._flag_key)
        status = (
            py_trees.common.Status.FAILURE if flag else py_trees.common.Status.SUCCESS
        )
        if status != self._last_status:
            self._ros_logger.info(
                f"{self.name}: transition {self._last_status} -> {status.name} "
                f"({self._flag_key}={flag})"
            )
            self._last_status = status
        return status
