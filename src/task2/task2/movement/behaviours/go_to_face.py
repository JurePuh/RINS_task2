"""GoToFace behaviour: navigate to the face at the head of the pending queue.

This module exposes one builder, `build()`, that returns a Sequence:

    GoToFace (Sequence, memory=True)
    ├── ComputeFaceDestination     # service call → writes NavigateToPose.Goal
    └── NavigateToFaceDestination  # py_trees_ros action client (FromBlackboard)

ComputeFaceDestination is the only "interesting" leaf — it shows how to call
a ROS 2 service from inside a behaviour without blocking the tree: the
service future is held across ticks, and the behaviour returns RUNNING until
the future resolves.

Wall-normal flow (preferred):
    Query `/wall_normal_at` at the face's reported (x, y). The service returns
    the nearest wall point + outward normal. We pick a destination 0.4m along
    the normal, facing the wall.

Fallback (if the service isn't running or fails):
    Aim for a point 0.4m short of the face along the straight line from the
    robot's current pose. Same idea, much rougher.
"""

import math
from typing import Optional

import py_trees
import py_trees_ros
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from task2.movement import blackboard as bb


# Distance from the face/wall we want to stop at, in metres.
_STANDOFF = 0.4
# How long we'll wait for /wall_normal_at to respond before giving up and
# using the fallback destination. Keep it short — tree ticks at 10 Hz.
_SERVICE_TIMEOUT_SEC = 1.0


class ComputeFaceDestination(py_trees.behaviour.Behaviour):
    """Compute a NavigateToPose goal from the head face on the queue.

    Tick lifecycle:
        initialise(): kick off a /wall_normal_at service request (if the
            service is up); record the start time.
        update():
            - if the service future has resolved → build goal, write to bb,
              SUCCESS.
            - if it's been pending too long → fall back to robot-pose-based
              destination, SUCCESS.
            - otherwise → RUNNING.
        terminate(): nothing to clean up (the future will resolve on its own
            and be garbage collected).
    """

    def __init__(self, name: str = "ComputeFaceDestination"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_FACES, access=py_trees.common.Access.READ)
        self.bb.register_key(key=bb.FACE_DESTINATION, access=py_trees.common.Access.WRITE)

        # Filled in setup(); the tree-owned rclpy node is shared with us.
        self.node: Optional[rclpy.node.Node] = None
        self.client = None
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None

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

        # Lazy import: WallNormalAt.srv may not be built yet — we want the tree
        # to still come up and use the fallback path in that case.
        try:
            from msg_types.srv import WallNormalAt  # noqa: F401
            self._WallNormalAt = WallNormalAt
            self.client = self.node.create_client(WallNormalAt, "wall_normal_at")
        except ImportError:
            self.node.get_logger().warning(
                "msg_types.srv.WallNormalAt not available — GoToFace will use "
                "the straight-line fallback for every detection."
            )
            self._WallNormalAt = None
            self.client = None

        # We need the robot pose for the fallback destination.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        # Read (but don't pop) the head face — MarkFaceHandled pops it later,
        # only on the success path.
        queue = self.bb.get(bb.PENDING_FACES)
        if not queue:
            # Shouldn't happen — HasUnhandledFace gates this. But be defensive.
            self._face = None
            return
        self._face = queue[0]

        self._future = None
        self._start_time = self.node.get_clock().now().nanoseconds * 1e-9  # type: ignore[union-attr]

        if self.client is not None and self.client.service_is_ready():
            req = self._WallNormalAt.Request()
            req.x = float(self._face.x)
            req.y = float(self._face.y)
            self._future = self.client.call_async(req)
            self.logger.info(
                f"querying wall_normal_at({self._face.x:.2f}, {self._face.y:.2f}) "
                f"for face {self._face.id}"
            )
        else:
            self.logger.info(
                f"wall_normal_at not available; will use fallback for face "
                f"{self._face.id}"
            )

    def update(self) -> py_trees.common.Status:
        if self._face is None:
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
                self.logger.warning(
                    f"wall_normal_at returned failure for face {self._face.id}, "
                    "using fallback destination"
                )
                self._future = None  # drop it, take fallback below
            else:
                now = self.node.get_clock().now().nanoseconds * 1e-9  # type: ignore[union-attr]
                if now - self._start_time < _SERVICE_TIMEOUT_SEC:
                    return py_trees.common.Status.RUNNING
                self.logger.warning(
                    f"wall_normal_at timed out for face {self._face.id}, "
                    "using fallback destination"
                )
                self._future = None  # take fallback below

        # Fallback path: derive a destination from the robot's current pose.
        goal = self._fallback_goal()
        if goal is None:
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

    def _fallback_goal(self) -> NavigateToPose.Goal | None:
        """Aim _STANDOFF metres short of the face along the robot→face line."""
        robot = self._lookup_robot_xy()
        if robot is None:
            # No pose available — last resort, go straight at the face.
            self.logger.warning(
                f"no robot pose available; navigating directly to face "
                f"{self._face.id} position"
            )
            return _build_nav_goal(self._face.x, self._face.y, 0.0)

        rx, ry = robot
        dx = self._face.x - rx
        dy = self._face.y - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            # Already on top of it — just face whatever direction.
            return _build_nav_goal(self._face.x, self._face.y, 0.0)
        ux, uy = dx / dist, dy / dist
        dest_x = self._face.x - _STANDOFF * ux
        dest_y = self._face.y - _STANDOFF * uy
        theta = math.atan2(dy, dx)  # face the face
        return _build_nav_goal(dest_x, dest_y, theta)

    def _lookup_robot_xy(self) -> tuple[float, float] | None:
        try:
            t = self.tf_buffer.lookup_transform(  # type: ignore[union-attr]
                "map", "base_link", rclpy.time.Time()
            )
        except Exception as exc:
            self.logger.warning(f"tf lookup map→base_link failed: {exc}")
            return None
        return (t.transform.translation.x, t.transform.translation.y)


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
    drive = py_trees_ros.action_clients.FromBlackboard(
        name="NavigateToFaceDestination",
        action_type=NavigateToPose,
        action_name="navigate_to_pose",
        key=bb.FACE_DESTINATION,
    )

    seq.add_children([compute, drive])
    return seq
