"""Person subtree: drive to person, classify, converse, activate requested task."""

import math

import py_trees
import py_trees_ros
import rclpy.client
import tf2_ros
from nav2_msgs.action import Spin
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from rclpy.impl.rcutils_logger import RcutilsLogger

from msg_types.srv import ClassifyFace, WallNormalAt

from task2.movement import blackboard as bb
from task2.movement.behaviours._nav import (
    NavigateToBlackboardGoal,
    SERVICE_TIMEOUT_SEC,
    build_nav_goal,
    lookup_robot_xy,
    standoff_goal_from_normal,
    standoff_goal_from_robot,
)
from task2.movement.behaviours.conditions import RecomputeNotRequested
from task2.movement.models import Gender, Person, Pose, Point, Vector


# Stub: i-th person we converse with returns the i-th task string.
_STUB_CONVERSATION_RESULTS = ["count_rings", "count_rings", "count_rings"]

_RESULT_TO_ACTIVE_FLAG = {
    "anomaly_red":     bb.ANOMALY_RED_ACTIVE,
    "anomaly_green":   bb.ANOMALY_GREEN_ACTIVE,
    "inspect_barrels": bb.BARREL_ACTIVE,
    "count_rings":     bb.RING_ACTIVE,
}


def _wrap_to_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ComputePersonDestination(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "ComputePersonDestination"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_PEOPLE, access=py_trees.common.Access.READ)
        self.bb.register_key(key=bb.FACE_DESTINATION, access=py_trees.common.Access.WRITE)
        self.bb.register_key(key=bb.RECOMPUTE_FACE_DESTINATION, access=py_trees.common.Access.WRITE)

        self.node: Node
        self.client: rclpy.client.Client
        self.tf_buffer: tf2_ros.Buffer
        self.tf_listener: tf2_ros.TransformListener

        self._future = None
        self._person: Person | None = None
        self._start_time: float = 0.0

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self.node.get_logger()
        # Set up wall_normal_at client
        self.client = self.node.create_client(WallNormalAt, "wall_normal_at")
        # Set up TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        queue = self.bb.get(bb.PENDING_PEOPLE); assert queue, f"{self.qualified_name}: initialise called with empty pending_people queue"
        self._person = queue[0]; assert self._person, f"{self.qualified_name}: initialise set _person to None"
        self.bb.set(bb.RECOMPUTE_FACE_DESTINATION, False)

        self._future = None
        self._start_time = self.node.get_clock().now().nanoseconds * 1e-9
        person = self._person
        # Query wall_normal_at
        if self.client.service_is_ready():
            req = WallNormalAt.Request()
            req.x = float(person.x)
            req.y = float(person.y)
            self._future = self.client.call_async(req)
            self._ros_logger.info(
                f"querying wall_normal_at({person.x:.2f}, {person.y:.2f}) "
                f"for person {person.face_id}"
            )
        else:
            self._ros_logger.warning(
                f"wall_normal_at not available; using fallback for person {person.face_id}"
            )

    def update(self) -> py_trees.common.Status:
        person = self._person; assert person, f"{self.qualified_name}: update called before initialise set _person"

        # Check if wall_normal_at returned yet
        if self._future is not None:
            if self._future.done() and self._future.result() is not None:
                resp: WallNormalAt.Response = self._future.result()  # type: ignore
                point = Point(resp.point_x, resp.point_y)
                normal = Vector(resp.normal_x, resp.normal_y)
                if resp is not None and getattr(resp, "success", False):
                    goal = standoff_goal_from_normal(point, normal)
                    self.bb.set(bb.FACE_DESTINATION, goal)
                    return py_trees.common.Status.SUCCESS
                self._ros_logger.warning(
                    f"wall_normal_at returned failure for person {person.face_id}; fallback"
                )
                self._future = None
            else:
                now = self.node.get_clock().now().nanoseconds * 1e-9
                if now - self._start_time < SERVICE_TIMEOUT_SEC:
                    return py_trees.common.Status.RUNNING
                self._ros_logger.warning(
                    f"wall_normal_at timed out for person {person.face_id}; fallback"
                )
                self._future = None

        # Fallback: navigate directly to face position with a fixed standoff
        robot_xy = lookup_robot_xy(self.tf_buffer, self._ros_logger)
        if robot_xy is None:
            # Fallback 2: navigate straight to face
            self._ros_logger.warning(
                f"no robot pose; navigating directly to person {person.face_id}"
            )
            self.bb.set(bb.FACE_DESTINATION, build_nav_goal(Pose(person.x, person.y, 0.0)))
            return py_trees.common.Status.SUCCESS

        self.bb.set(bb.FACE_DESTINATION, standoff_goal_from_robot(Point(robot_xy[0], robot_xy[1]), Point(person.x, person.y)))
        return py_trees.common.Status.SUCCESS


class TurnTowardsPerson(py_trees_ros.action_clients.FromConstant):
    """Spin in place so base_link faces the head of PENDING_PEOPLE.

    Recomputes target_yaw in initialise() from current robot+person poses; if tf
    lookup fails, skips the spin entirely (no-op SUCCESS).
    """

    def __init__(self, name: str = "TurnTowardsPerson"):
        self._spin_goal = Spin.Goal()
        super().__init__(
            name=name,
            action_type=Spin,
            action_name="spin",
            action_goal=self._spin_goal,
        )
        self._pbb = self.attach_blackboard_client(name=f"{name}_pending")
        self._pbb.register_key(key=bb.PENDING_PEOPLE, access=py_trees.common.Access.READ)
        self._skip = False

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self.node.get_logger()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        self._skip = False
        queue = self._pbb.get(bb.PENDING_PEOPLE)
        assert queue is not None, "TurnTowardsPerson: PENDING_PEOPLE not found on blackboard"
        person: Person = queue[0]

        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except Exception as exc:
            self._ros_logger.warning(
                f"TurnTowardsPerson: tf lookup map->base_link failed: {exc}; "
                "skipping spin"
            )
            self._skip = True
            return

        rx = t.transform.translation.x
        ry = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        desired = math.atan2(person.y - ry, person.x - rx)
        self._spin_goal.target_yaw = _wrap_to_pi(desired - yaw)
        self._ros_logger.info(
            f"TurnTowardsPerson: spin {self._spin_goal.target_yaw:+.2f} rad"
        )
        super().initialise()

    def update(self):
        if self._skip:
            return py_trees.common.Status.SUCCESS
        return super().update()


class ClassifyPersonCall(py_trees.behaviour.Behaviour):
    """Call /classify_face; mutate the head Person in PENDING_PEOPLE with the result."""

    def __init__(self, name: str = "ClassifyPersonCall"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_PEOPLE, access=py_trees.common.Access.READ)

        self._future: Future | None = None
        self._start_time: float = 0.0

    def setup(self, **kwargs):
        self.node: Node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self.node.get_logger()
        self.client = self.node.create_client(ClassifyFace, "classify_face")

    def initialise(self):
        self._future = self.client.call_async(ClassifyFace.Request())
        self._start_time = self.node.get_clock().now().nanoseconds * 1e-9

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_PEOPLE)
        assert queue, "ClassifyPersonCall: PENDING_PEOPLE empty"
        person: Person = queue[0]

        if self._future is None:
            return py_trees.common.Status.FAILURE

        if not self._future.done():
            now = self.node.get_clock().now().nanoseconds * 1e-9
            if now - self._start_time < SERVICE_TIMEOUT_SEC:
                return py_trees.common.Status.RUNNING
            self._ros_logger.warning(
                f"classify_face timed out for person {person.face_id}"
            )
            return py_trees.common.Status.FAILURE

        resp: ClassifyFace.Response = self._future.result()  # type: ignore
        if resp is None or not getattr(resp, "success", False):
            self._ros_logger.warning(
                f"classify_face failed for person {person.face_id}: "
                f"{getattr(resp, 'message', '?')}"
            )
            return py_trees.common.Status.FAILURE

        person.name = resp.name
        person.role = resp.role
        try:
            person.gender = Gender(resp.gender)
        except ValueError:
            person.gender = None
        self._ros_logger.info(
            f"classified person {person.face_id}: "
            f"name={person.name} role={person.role} gender={person.gender}"
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


class ConversePerson(py_trees.behaviour.Behaviour):
    """[stub] Pretend the i-th conversation requests the i-th task in the constant list."""

    def __init__(self, name: str = "ConversePerson"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_PEOPLE, access=py_trees.common.Access.READ)
        self.bb.register_key(key=bb.HANDLED_PEOPLE, access=py_trees.common.Access.READ)
        self.bb.register_key(key=bb.CONVERSATION_RESULT, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        handled = self.bb.get(bb.HANDLED_PEOPLE)
        idx = len(handled)
        if idx < len(_STUB_CONVERSATION_RESULTS):
            result = _STUB_CONVERSATION_RESULTS[idx]
        else:
            result = ""
        self.bb.set(bb.CONVERSATION_RESULT, result)

        queue = self.bb.get(bb.PENDING_PEOPLE)
        person: Person = queue[0]
        self._ros_logger.info(
            f"[stub] ConversePerson with {person.name or person.face_id} -> '{result}'"
        )
        return py_trees.common.Status.SUCCESS


class MarkPersonHandled(py_trees.behaviour.Behaviour):
    """Pop the visited person off the queue and record its face id."""

    def __init__(self, name: str = "MarkPersonHandled"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_PEOPLE, access=py_trees.common.Access.WRITE)
        self.bb.register_key(key=bb.HANDLED_PEOPLE, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_PEOPLE)
        handled = self.bb.get(bb.HANDLED_PEOPLE)
        person: Person = queue.popleft()
        handled.add(person.face_id)
        self._ros_logger.info(f"person {person.face_id} marked handled")
        return py_trees.common.Status.SUCCESS


class ActivateRequestedTask(py_trees.behaviour.Behaviour):
    """Flip the active flag matching CONVERSATION_RESULT, then clear it.

    Runs as the last child of the Person sequence — after MarkPersonHandled has
    already popped the head, so the priority guard HasUnhandledPerson is False.
    The active flag we set here can therefore safely cause the parent Selector
    to switch branches on the next tick.
    """

    def __init__(self, name: str = "ActivateRequestedTask"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.CONVERSATION_RESULT, access=py_trees.common.Access.WRITE)
        for flag_key in _RESULT_TO_ACTIVE_FLAG.values():
            self.bb.register_key(key=flag_key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        result = self.bb.get(bb.CONVERSATION_RESULT) or ""
        flag_key = _RESULT_TO_ACTIVE_FLAG.get(result)
        if flag_key is not None:
            self.bb.set(flag_key, True)
            self._ros_logger.info(f"activated task: {result} ({flag_key}=True)")
        else:
            self._ros_logger.info(f"no active flag for result '{result}'")
        self.bb.set(bb.CONVERSATION_RESULT, "")
        return py_trees.common.Status.SUCCESS


# --- BUILD ---

def build() -> py_trees.composites.Sequence:
    """Drive to the person, converse, then activate whichever task they asked for."""
    drive_or_recompute = py_trees.composites.Sequence(
        name="DriveOrRecomputePerson", memory=False,
    )
    drive_or_recompute.add_children([
        RecomputeNotRequested(
            name="NoPersonRecomputeRequested",
            flag_key=bb.RECOMPUTE_FACE_DESTINATION,
        ),
        NavigateToBlackboardGoal(
            name="NavigateToPersonDestination",
            goal_key=bb.FACE_DESTINATION,
        ),
    ])

    seq = py_trees.composites.Sequence(name="Person", memory=True)
    seq.add_children([
        ComputePersonDestination(),
        drive_or_recompute,
        TurnTowardsPerson(),
        ClassifyPersonCall(),
        ConversePerson(),
        MarkPersonHandled(),
        ActivateRequestedTask(),
    ])
    return seq
