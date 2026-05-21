"""Small condition leaves used by the priority Selector."""

import py_trees

from rclpy.impl.rcutils_logger import RcutilsLogger

from task2.movement import blackboard as bb


# --- ANOMALY TASKS ----

class _ActiveFlagPending(py_trees.behaviour.Behaviour):
    """SUCCESS iff the given boolean blackboard flag is True."""

    def __init__(self, name: str, flag_key: str):
        super().__init__(name=name)
        self._flag_key = flag_key
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=flag_key, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        if self.bb.get(self._flag_key):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


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

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_PEOPLE)
        if queue:
            self._ros_logger.debug(f"{len(queue)} pending faces in queue")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


# --- BARREL TASKS ----

class BarrelVisitPending(py_trees.behaviour.Behaviour):
    """SUCCESS iff BARREL_ACTIVE is True AND PENDING_BARRELS is non-empty."""

    def __init__(self, name: str = "HasUnvisitedHorizontalBarrel"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.BARREL_ACTIVE, access=py_trees.common.Access.READ)
        self.bb.register_key(key=bb.PENDING_BARRELS, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self._log = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        if self.bb.get(bb.BARREL_ACTIVE) and self.bb.get(bb.PENDING_BARRELS):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

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

    def update(self) -> py_trees.common.Status:
        if self.bb.get(self._flag_key):
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
