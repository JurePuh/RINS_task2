import math

import py_trees
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

from task2.movement import blackboard as bb


# Distance from the face/wall we want to stop at, in metres.
_STANDOFF = 0.4
# How long we'll wait for /wall_normal_at to respond before giving up, using fallback.
_SERVICE_TIMEOUT_SEC = 1.0

# Stall detection for NavigateToFaceDestination.
_STALL_TIMEOUT = 5.0       # s without improvement in distance_remaining
_IMPROVEMENT_EPSILON = 0.1  # m — what counts as "getting closer"


class ComputeFaceDestination(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "ComputeFaceDestination"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_FACES, access=py_trees.common.Access.READ)
        self.bb.register_key(key=bb.FACE_DESTINATION, access=py_trees.common.Access.WRITE)
        self.bb.register_key(key=bb.RECOMPUTE_DESTINATION, access=py_trees.common.Access.WRITE)

        # Filled in setup(); the tree-owned rclpy node is shared with us.
        self.node: Node
        self.client = None
        self.tf_buffer: tf2_ros.Buffer
        self.tf_listener: tf2_ros.TransformListener

        # Per-call state, reset in initialise().
        self._future = None
        self._face = None
        self._start_time: float = 0.0

    def setup(self, **kwargs):
        # py_trees_ros passes the shared node in via kwargs at tree setup time.
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                f"{self.qualified_name}: 'node' missing from setup kwargs"
            ) from e
        self._ros_logger = self.node.get_logger()

        # Lazy import: WallNormalAt.srv may not be built yet — we want the tree
        # to still come up and use the fallback path in that case.
        try:
            from msg_types.srv import WallNormalAt
            self._WallNormalAt = WallNormalAt
            self.client = self.node.create_client(WallNormalAt, "wall_normal_at")
        except ImportError:
            self._ros_logger.warning(
                "msg_types.srv.WallNormalAt not available — GoToFace will use "
                "the straight-line fallback for every detection."
            )
            self._WallNormalAt = None
            self.client = None

        # We need the robot pose for the fallback destination.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        # Read (but don't pop) the head face — MarkFaceHandled pops it later.
        queue = self.bb.get(bb.PENDING_FACES)
        if not queue:
            # Shouldn't happen — HasUnhandledFace gates this. But be defensive.
            self.logger.warning("initialise called with empty pending_faces queue")
            self._face = None
            return
        self._face = queue[0]

        # Fresh attempt — clear any stale recompute flag. The /face_detect
        # subscription will re-raise it if the head face is updated again
        # while we're driving.
        self.bb.set(bb.RECOMPUTE_DESTINATION, False)

        self._future = None
        self._start_time = self.node.get_clock().now().nanoseconds * 1e-9
        face = self._face
        if (
            self.client is not None
            and self._WallNormalAt is not None
            and self.client.service_is_ready()
        ):
            req = self._WallNormalAt.Request()
            req.x = float(face.x)
            req.y = float(face.y)
            self._future = self.client.call_async(req)
            self._ros_logger.info(
                f"querying wall_normal_at({face.x:.2f}, {face.y:.2f}) "
                f"for face {face.id}"
            )
        else:
            self._ros_logger.info(
                f"wall_normal_at not available; will use fallback for face "
                f"{face.id}"
            )

    def update(self) -> py_trees.common.Status:
        face = self._face
        if face is None:
            # If initialise() failed to read a face somehow
            self.logger.warning("update called without a face set in initialise()")
            return py_trees.common.Status.FAILURE

        # Service path: wait on the future, with a hard timeout.
        if self._future is not None:
            if self._future.done():
                resp = self._future.result()
                if resp is not None and getattr(resp, "success", False):
                    goal = self._goal_from_wall_normal(resp)
                    self.bb.set(bb.FACE_DESTINATION, goal)
                    return py_trees.common.Status.SUCCESS
                # Service responded but with success=False → fall through to
                # the straight-line fallback rather than failing the sequence.
                self._ros_logger.warning(
                    f"wall_normal_at returned failure for face {face.id}, "
                    "using fallback destination"
                )
                self._future = None  # drop it, take fallback below
            else:
                now = self.node.get_clock().now().nanoseconds * 1e-9
                if now - self._start_time < _SERVICE_TIMEOUT_SEC:
                    return py_trees.common.Status.RUNNING
                self._ros_logger.warning(
                    f"wall_normal_at timed out for face {face.id}, "
                    "using fallback destination"
                )
                self._future = None  # take fallback below

        # Fallback path: derive a destination from the robot's current pose.
        goal = self._fallback_goal(face)
        if goal is None:
            # Fallback failed too (e.g. no TF) — give up on this face and move on to the next.
            self._ros_logger.error(
                f"failed to compute fallback destination for face {face.id}; "
                "skipping it"
            )
            return py_trees.common.Status.FAILURE
        
        self.bb.set(bb.FACE_DESTINATION, goal)
        return py_trees.common.Status.SUCCESS

    # ----- destination helpers ------------------------------------------------

    def _goal_from_wall_normal(self, resp) -> NavigateToPose.Goal:
        """Place destination _STANDOFF metres outside the wall, facing it."""
        dest_x = resp.point_x + _STANDOFF * resp.normal_x
        dest_y = resp.point_y + _STANDOFF * resp.normal_y
        theta = math.atan2(-resp.normal_y, -resp.normal_x)  # look at the wall
        return _build_nav_goal(dest_x, dest_y, theta)

    def _fallback_goal(self, face) -> NavigateToPose.Goal | None:
        """Aim _STANDOFF metres short of the face along the robot→face line."""
        robot = self._lookup_robot_xy()
        if robot is None:
            # No pose available — last resort, go straight at the face.
            self._ros_logger.warning(
                f"no robot pose available; navigating directly to face "
                f"{face.id} position"
            )
            return _build_nav_goal(face.x, face.y, 0.0)

        rx, ry = robot
        dx = face.x - rx
        dy = face.y - ry
        dist = math.hypot(dx, dy)
        ux, uy = dx / dist, dy / dist
        dest_x = face.x - _STANDOFF * ux
        dest_y = face.y - _STANDOFF * uy
        theta = math.atan2(dy, dx)  # face the face
        return _build_nav_goal(dest_x, dest_y, theta)

    def _lookup_robot_xy(self) -> tuple[float, float] | None:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", Time()
            )
        except Exception as exc:
            self._ros_logger.warning(f"tf lookup map→base_link failed: {exc}")
            return None
        return (t.transform.translation.x, t.transform.translation.y)


class NavigateToFaceDestination(py_trees.behaviour.Behaviour):
    """Send FACE_DESTINATION to nav2 and drive there, with two extras over the
    stock action-client behaviour:

    1. **Recompute on update.** If RECOMPUTE_DESTINATION flips to True
       mid-flight (the /face_detect subscription saw the head face move),
       cancel the goal and return FAILURE. The parent Sequence aborts before
       MarkFaceHandled ticks, so the face stays in the queue and the next
       entry recomputes the destination with the fresh coordinates.

    2. **Stall bail-out.** Watch nav2's feedback.distance_remaining. If it
       hasn't improved by _IMPROVEMENT_EPSILON for _STALL_TIMEOUT seconds,
       cancel the goal and return SUCCESS — treat the face as visited so we
       don't grind on an unreachable point forever.

    Any natural termination (SUCCEEDED, ABORTED, CANCELED) also returns
    SUCCESS — the face is considered handled regardless of outcome.
    """

    def __init__(self, name: str = "NavigateToFaceDestination"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.FACE_DESTINATION, access=py_trees.common.Access.READ)
        self.bb.register_key(key=bb.RECOMPUTE_DESTINATION, access=py_trees.common.Access.READ)

        self.node: Node
        self.client: ActionClient

        # Per-attempt state, all reset in initialise().
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._latest_distance: float | None = None
        self._best_distance: float = math.inf
        self._last_improvement_time: float = 0.0
        # Once we've decided to cancel, what status to return after the cancel
        # round-trip finishes. SUCCESS for stall, FAILURE for recompute.
        self._post_cancel_status: py_trees.common.Status | None = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                f"{self.qualified_name}: 'node' missing from setup kwargs"
            ) from e
        self._ros_logger = self.node.get_logger()
        self.client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")

    def initialise(self):
        goal = self.bb.get(bb.FACE_DESTINATION)
        self._send_goal_future = self.client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._latest_distance = None
        self._best_distance = math.inf
        self._last_improvement_time = self._now()
        self._post_cancel_status = None

    def _on_feedback(self, feedback_msg) -> None:
        # Just store it; the tick reads it. Keeps the state machine in one place.
        self._latest_distance = float(feedback_msg.feedback.distance_remaining)

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def update(self) -> py_trees.common.Status:
        # Phase A: awaiting goal acceptance.
        if self._goal_handle is None:
            if not self._send_goal_future.done():
                return py_trees.common.Status.RUNNING
            self._goal_handle = self._send_goal_future.result()
            if self._goal_handle is None or not self._goal_handle.accepted:
                self._ros_logger.warning("nav2 rejected the face goal")
                return py_trees.common.Status.SUCCESS
            self._result_future = self._goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        # Phase C: a cancel is in flight; wait for it then return the stashed status.
        if self._cancel_future is not None:
            if not self._cancel_future.done():
                return py_trees.common.Status.RUNNING
            return self._post_cancel_status or py_trees.common.Status.SUCCESS

        # Phase B: goal in flight. Check terminations in priority order.

        # 1. Recompute flag — bail with FAILURE so the parent Sequence restarts.
        if self.bb.get(bb.RECOMPUTE_DESTINATION):
            self._ros_logger.info("recompute flag set, cancelling face goal")
            self._cancel_future = self._goal_handle.cancel_goal_async()
            self._post_cancel_status = py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        # 2. Natural completion (SUCCEEDED / ABORTED / CANCELED).
        if self._result_future.done():
            return py_trees.common.Status.SUCCESS

        # 3. Stall detection from feedback.
        if self._latest_distance is not None:
            if self._latest_distance < self._best_distance - _IMPROVEMENT_EPSILON:
                self._best_distance = self._latest_distance
                self._last_improvement_time = self._now()
            elif self._now() - self._last_improvement_time > _STALL_TIMEOUT:
                self._ros_logger.warning(
                    f"face goal stalled at {self._best_distance:.2f} m; cancelling"
                )
                self._cancel_future = self._goal_handle.cancel_goal_async()
                self._post_cancel_status = py_trees.common.Status.SUCCESS
                return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        # If we're being yanked by a parent (status INVALID) while a goal is
        # still alive, fire-and-forget a cancel. Don't wait — terminate() must
        # not block the tick.
        if (
            new_status == py_trees.common.Status.INVALID
            and self._goal_handle is not None
            and self._cancel_future is None
        ):
            try:
                self._goal_handle.cancel_goal_async()
            except Exception as exc:
                self._ros_logger.warning(f"cancel on terminate failed: {exc}")


def _build_nav_goal(x: float, y: float, theta: float) -> NavigateToPose.Goal:
    """Construct a NavigateToPose.Goal from (x, y, yaw) in the map frame."""
    goal = NavigateToPose.Goal()
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.z = math.sin(theta / 2.0)
    ps.pose.orientation.w = math.cos(theta / 2.0)
    goal.pose = ps
    return goal


def build() -> py_trees.composites.Sequence:
    """Construct the GoToFace sub-tree (compute destination, then drive there)."""
    seq = py_trees.composites.Sequence(name="GoToFace", memory=True)

    compute = ComputeFaceDestination()
    drive = NavigateToFaceDestination()

    seq.add_children([compute, drive])
    return seq
