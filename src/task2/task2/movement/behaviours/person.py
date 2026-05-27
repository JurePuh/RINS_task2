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
from msg_types.srv import ConversePerson as ConversePersonSrv

from task2.movement import blackboard as bb
from task2.movement.behaviours._nav import (
    NavigateToBlackboardGoal,
    NavRetryThenSucceed,
    SERVICE_TIMEOUT_SEC,
    build_nav_goal,
    lookup_robot_xy,
    standoff_goal_from_normal,
    standoff_goal_from_robot,
)
from task2.movement.behaviours.conditions import RecomputeNotRequested
from task2.movement.log_utils import log_throttled
from task2.movement.models import Gender, Person, Pose, Point, Vector


# If True, ConversePerson calls the /converse_person service; if False, uses the stub list below.
_USE_CONVERSATION_PERSON: bool = False

# Stub: i-th person we converse with returns the i-th task string.
_STUB_CONVERSATION_RESULTS = ["inspect_barrels", "count_rings"]

_RESULT_TO_ACTIVE_FLAG = {
    "anomaly_red":     bb.ANOMALY_RED_ACTIVE,
    "anomaly_green":   bb.ANOMALY_GREEN_ACTIVE,
    "inspect_barrels": bb.BARREL_ACTIVE,
    "count_rings":     bb.RING_ACTIVE,
}

_RESULT_TO_TASK_KEY = {
    "anomaly_red":     bb.TASK_ANOMALY_RED,
    "anomaly_green":   bb.TASK_ANOMALY_GREEN,
    "inspect_barrels": bb.TASK_INSPECT_BARRELS,
    "count_rings":     bb.TASK_COUNT_RINGS,
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
        self._ros_logger.info(
            f"{self.name}: starting for person {self._person.face_id}; "
            f"RECOMPUTE_FACE_DESTINATION=False; queue_size={len(queue)}"
        )

        self._future = None
        self._start_time = self.node.get_clock().now().nanoseconds * 1e-9
        person = self._person
        self._ros_logger.debug(
            f"{self.name}: inputs person.face_id={person.face_id} "
            f"xy=({person.x:.2f},{person.y:.2f}) "
            f"service_ready={self.client.service_is_ready()}"
        )
        # Query wall_normal_at
        if self.client.service_is_ready():
            robot_xy = lookup_robot_xy(self.tf_buffer, self._ros_logger)
            if robot_xy is None:
                self._ros_logger.warning(
                    f"robot pose not available via TF; using fallback for person {person.face_id}"
                )
            else:
                req = WallNormalAt.Request()
                req.x = float(person.x)
                req.y = float(person.y)
                req.robot_x = float(robot_xy[0])
                req.robot_y = float(robot_xy[1])
                self._future = self.client.call_async(req)
                self._ros_logger.info(
                    f"querying wall_normal_at({person.x:.2f}, {person.y:.2f}) "
                    f"from robot ({req.robot_x:.2f}, {req.robot_y:.2f}) "
                    f"for person {person.face_id}"
                )
                self._ros_logger.debug(
                    f"{self.name}: WallNormalAt request x={req.x:.2f} y={req.y:.2f} "
                    f"robot_x={req.robot_x:.2f} robot_y={req.robot_y:.2f}"
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
                # returned - check if succeeded
                resp: WallNormalAt.Response = self._future.result()  # type: ignore
                point = Point(resp.point_x, resp.point_y)
                normal = Vector(resp.normal_x, resp.normal_y)
                if resp is not None and getattr(resp, "success", False):
                    # succeeded - set goal and return SUCCESS
                    goal = standoff_goal_from_normal(point, normal)
                    self.bb.set(bb.FACE_DESTINATION, goal)
                    gp = goal.pose.pose.position
                    self._ros_logger.info(
                        f"{self.name}: FACE_DESTINATION via wall_normal for "
                        f"person {person.face_id} -> ({gp.x:.2f}, {gp.y:.2f})"
                    )
                    self._ros_logger.debug(
                        f"{self.name}: wall_normal point=({point.x:.2f},{point.y:.2f}) "
                        f"normal=({normal.x:.2f},{normal.y:.2f})"
                    )
                    return py_trees.common.Status.SUCCESS
                # failed
                self._ros_logger.warning(
                    f"wall_normal_at returned failure for person {person.face_id}; fallback"
                )
                self._future = None
            else:
                # didnt return yet - check for timeout
                now = self.node.get_clock().now().nanoseconds * 1e-9
                elapsed = now - self._start_time
                if elapsed < SERVICE_TIMEOUT_SEC:
                    log_throttled(
                        self._ros_logger, self.node, f"{self.name}.waiting", "debug",
                        f"{self.name}: waiting for wall_normal_at "
                        f"(elapsed={elapsed:.2f}s)",
                    )
                    return py_trees.common.Status.RUNNING
                self._ros_logger.warning(
                    f"wall_normal_at timed out for person {person.face_id} "
                    f"after {elapsed:.2f}s; fallback"
                )
                self._future = None

        # Fallback: navigate directly to face position with a fixed standoff
        robot_xy = lookup_robot_xy(self.tf_buffer, self._ros_logger)
        if robot_xy is None:
            # Fallback 2: navigate straight to face
            self._ros_logger.warning(
                f"no robot pose; navigating directly to person {person.face_id}"
            )
            goal = build_nav_goal(Pose(person.x, person.y, 0.0))
            self.bb.set(bb.FACE_DESTINATION, goal)
            self._ros_logger.info(
                f"{self.name}: FACE_DESTINATION via direct fallback for "
                f"person {person.face_id} -> ({person.x:.2f}, {person.y:.2f})"
            )
            return py_trees.common.Status.SUCCESS

        goal = standoff_goal_from_robot(
            Point(robot_xy[0], robot_xy[1]), Point(person.x, person.y)
        )
        self.bb.set(bb.FACE_DESTINATION, goal)
        gp = goal.pose.pose.position
        self._ros_logger.info(
            f"{self.name}: FACE_DESTINATION via robot-tf fallback for "
            f"person {person.face_id} -> ({gp.x:.2f}, {gp.y:.2f})"
        )
        self._ros_logger.debug(
            f"{self.name}: robot=({robot_xy[0]:.2f},{robot_xy[1]:.2f}) "
            f"person=({person.x:.2f},{person.y:.2f})"
        )
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
            self._ros_logger.info(
                f"{self.name}: skipping spin (tf lookup failed in initialise)"
            )
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
        queue = self.bb.get(bb.PENDING_PEOPLE)
        head = queue[0] if queue else None
        self._ros_logger.info(
            f"{self.name}: calling /classify_face for person "
            f"{head.face_id if head else '?'}"
        )
        self._ros_logger.debug(
            f"{self.name}: request payload = ClassifyFace.Request() (empty); "
            f"queue_size={len(queue) if queue else 0}"
        )
        self._future = self.client.call_async(ClassifyFace.Request())
        self._start_time = self.node.get_clock().now().nanoseconds * 1e-9

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_PEOPLE)
        assert queue, "ClassifyPersonCall: PENDING_PEOPLE empty"
        person: Person = queue[0]
        assert self._future is not None, f"{self.qualified_name}: update called before initialise set _future"

        # Check if classify_face returned yet
        if not self._future.done():
            # hasnt returned yet - check for timeout
            now = self.node.get_clock().now().nanoseconds * 1e-9
            elapsed = now - self._start_time
            if elapsed < SERVICE_TIMEOUT_SEC:
                log_throttled(
                    self._ros_logger, self.node, f"{self.name}.waiting", "debug",
                    f"{self.name}: waiting for classify_face "
                    f"(elapsed={elapsed:.2f}s, person={person.face_id})",
                )
                return py_trees.common.Status.RUNNING
            self._ros_logger.error(
                f"classify_face timed out for person {person.face_id} "
                f"after {elapsed:.2f}s; labelling as 'unknown' and continuing"
            )
            self._assign_unknown(person)
            return py_trees.common.Status.SUCCESS

        resp: ClassifyFace.Response = self._future.result()  # type: ignore
        if resp is None or not getattr(resp, "success", False):
            self._ros_logger.error(
                f"classify_face failed for person {person.face_id}: "
                f"{getattr(resp, 'message', '?')}; labelling as 'unknown' and continuing"
            )
            self._assign_unknown(person)
            return py_trees.common.Status.SUCCESS

        person.name = resp.name.split("_")[0]  # e.g. "alice_smith" -> "alice"
        person.role = resp.role
        try:
            person.gender = Gender(resp.gender)
        except ValueError:
            person.gender = None
        self._ros_logger.info(
            f"classified person {person.face_id}: "
            f"name={person.name} role={person.role} gender={person.gender}"
        )
        # Push the identity onto the person's RViz marker.
        gender_str = person.gender.value if person.gender else None
        self.node.visualizer.set_person_label(person.face_id, person.name, gender_str)  # type: ignore[attr-defined]
        return py_trees.common.Status.SUCCESS

    def _assign_unknown(self, person: Person) -> None:
        person.name = "unknown"
        person.role = ""
        person.gender = None
        self.node.visualizer.set_person_label(person.face_id, person.name, None)  # type: ignore[attr-defined]

    def terminate(self, new_status):
        if (
            new_status == py_trees.common.Status.INVALID
            and self._future is not None
            and not self._future.done()
        ):
            self._future.cancel()
        self._future = None


class ConversePerson(py_trees.behaviour.Behaviour):
    """Call /converse_person to get the task this person requests.

    When `_USE_CONVERSATION_PERSON` is False, falls back to a stub that returns
    the i-th entry of `_STUB_CONVERSATION_RESULTS` based on how many people
    have been handled so far.
    """

    def __init__(self, name: str = "ConversePerson"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_PEOPLE, access=py_trees.common.Access.READ)
        self.bb.register_key(key=bb.HANDLED_PEOPLE, access=py_trees.common.Access.READ)
        self.bb.register_key(key=bb.CONVERSATION_RESULT, access=py_trees.common.Access.WRITE)

        self._future: Future | None = None

    def setup(self, **kwargs):
        self.node: Node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self.node.get_logger()
        if _USE_CONVERSATION_PERSON:
            self.client = self.node.create_client(ConversePersonSrv, "converse_person")

    def initialise(self):
        self._future = None
        if not _USE_CONVERSATION_PERSON:
            return
        queue = self.bb.get(bb.PENDING_PEOPLE)
        assert queue, f"{self.qualified_name}: initialise called with empty PENDING_PEOPLE"
        person: Person = queue[0]
        req = ConversePersonSrv.Request()
        req.gender = person.gender.value if person.gender else ""
        self._ros_logger.info(
            f"{self.name}: calling /converse_person for person "
            f"{person.face_id} (gender='{req.gender}')"
        )
        self._future = self.client.call_async(req)

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_PEOPLE)
        assert queue, "ConversePerson: PENDING_PEOPLE empty"
        person: Person = queue[0]

        # stub for debug: return predetermined results based on how many people we've handled so far
        if not _USE_CONVERSATION_PERSON:
            handled = self.bb.get(bb.HANDLED_PEOPLE)
            idx = len(handled)
            if idx < len(_STUB_CONVERSATION_RESULTS):
                result = _STUB_CONVERSATION_RESULTS[idx]
            else:
                result = ""
            self.bb.set(bb.CONVERSATION_RESULT, result)
            self._ros_logger.info(
                f"[stub] ConversePerson with {person.name or person.face_id} -> '{result}' "
                f"(handled_so_far={idx})"
            )
            self._ros_logger.debug(
                f"{self.name}: CONVERSATION_RESULT='{result}' "
                f"person.face_id={person.face_id} role={person.role}"
            )
            return py_trees.common.Status.SUCCESS

        assert self._future is not None, f"{self.qualified_name}: update called before initialise set _future"

        # Check if converse_person returned yet
        if not self._future.done():
            log_throttled(
                self._ros_logger, self.node, f"{self.name}.waiting", "debug",
                f"{self.name}: waiting for converse_person; person={person.face_id})",
            )
            return py_trees.common.Status.RUNNING

        # returned - check if succeeded
        resp: ConversePersonSrv.Response = self._future.result()  # type: ignore
        if resp is None:
            # no response at all (e.g. service call failed)
            self._ros_logger.warning(
                f"converse_person returned no response for person {person.face_id}"
            )
            return py_trees.common.Status.FAILURE

        # retrieve result and succeed
        result = resp.task
        self.bb.set(bb.CONVERSATION_RESULT, result)
        self._ros_logger.info(
            f"converse_person({person.name or person.face_id}) -> '{result}'"
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


class MarkPersonHandled(py_trees.behaviour.Behaviour):
    """Pop the visited person off the queue and record its face id."""

    def __init__(self, name: str = "MarkPersonHandled"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_PEOPLE, access=py_trees.common.Access.WRITE)
        self.bb.register_key(key=bb.HANDLED_PEOPLE, access=py_trees.common.Access.WRITE)
        self.bb.register_key(key=bb.LAST_HANDLED_PERSON, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_PEOPLE)
        handled = self.bb.get(bb.HANDLED_PEOPLE)
        person: Person = queue.popleft()
        handled.add(person.face_id)
        self.bb.set(bb.LAST_HANDLED_PERSON, person)
        self._ros_logger.info(
            f"person {person.face_id} marked handled; "
            f"pending={len(queue)} handled_total={len(handled)}"
        )
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
        self.bb.register_key(key=bb.LAST_HANDLED_PERSON, access=py_trees.common.Access.WRITE)
        for flag_key in _RESULT_TO_ACTIVE_FLAG.values():
            self.bb.register_key(key=flag_key, access=py_trees.common.Access.WRITE)
        for task_key in _RESULT_TO_TASK_KEY.values():
            self.bb.register_key(key=task_key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        result = self.bb.get(bb.CONVERSATION_RESULT) or ""
        self._ros_logger.debug(
            f"{self.name}: inputs CONVERSATION_RESULT='{result}' "
            f"valid_keys={list(_RESULT_TO_ACTIVE_FLAG.keys())}"
        )
        # Set wanted task to active
        flag_key = _RESULT_TO_ACTIVE_FLAG.get(result)
        if flag_key is not None:
            self.bb.set(flag_key, True)
            self._ros_logger.info(f"activated task: {result} ({flag_key}=True)")
        else:
            self._ros_logger.warning(f"no active flag for result '{result}'")

        # Record the requester on the task object so it shows up in the report.
        task_key = _RESULT_TO_TASK_KEY.get(result)
        requester: Person | None = self.bb.get(bb.LAST_HANDLED_PERSON)
        if task_key is not None and requester is not None:
            task = self.bb.get(task_key)
            if task is not None and requester not in task.requesters:
                task.requesters.append(requester)
                self._ros_logger.info(
                    f"added requester {requester.name or requester.face_id} "
                    f"to {task_key}"
                )
        self.bb.set(bb.LAST_HANDLED_PERSON, None)

        # Clear and succeed
        self.bb.set(bb.CONVERSATION_RESULT, "")
        self._ros_logger.debug(f"{self.name}: cleared CONVERSATION_RESULT")
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
        NavRetryThenSucceed(
            child=NavigateToBlackboardGoal(
                name="NavigateToPersonDestination",
                goal_key=bb.FACE_DESTINATION,
            ),
            queue_key=bb.PENDING_PEOPLE,
            head_id_fn=lambda p: p.face_id,
            name="PersonNavRetry",
        ),
    ])

    seq = py_trees.composites.Sequence(name="Person", memory=True)
    seq.add_children([
        ComputePersonDestination(),
        drive_or_recompute,
        py_trees.decorators.FailureIsSuccess(
            name="TurnTowardsPersonTolerant",
            child=TurnTowardsPerson(),
        ),
        ClassifyPersonCall(),
        ConversePerson(),
        MarkPersonHandled(),
        ActivateRequestedTask(),
    ])
    return seq
