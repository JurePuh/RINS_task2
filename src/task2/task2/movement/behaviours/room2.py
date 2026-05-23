"""Room 2: exit corridor, blue-line following, CTO check, report, final dance."""

import math

import py_trees
import py_trees_ros
import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import Spin
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.publisher import Publisher
from std_msgs.msg import String

from rclpy.node import Node
from rclpy.task import Future

from msg_types.msg import BlueLineStatus
from msg_types.srv import ClassifyFace

from task2.movement import blackboard as bb
from task2.movement.behaviours._arm import SetArmPosition
from task2.movement.behaviours._nav import LoggingNavWaypoint, SERVICE_TIMEOUT_SEC, build_nav_goal
from task2.movement.models import Pose


_CORRIDOR_ENTRANCE_POSE = Pose( 2.8, -0.2, -1.5)

_FWD_SPEED = 0.25      # m/s forward while following the line
_KP = 2.5              # proportional gain: angular.z = -_KP * offset
_END_FRAMES = 10       # consecutive STATE_LOST frames after first LINE → done

_END_LINE = "All anomalies inspected. I'd take a bow but I don't have hips."

_MAX_CTO_ATTEMPTS = 8  # follow-line + check tries before giving up


class GoToCorridorEntrance(LoggingNavWaypoint):
    """Navigate to the hardcoded corridor-entrance pose."""

    def __init__(self, name: str = "GoToCorridorEntrance"):
        super().__init__(name=name, action_goal=build_nav_goal(_CORRIDOR_ENTRANCE_POSE))


class FollowBlueLine(py_trees.behaviour.Behaviour):
    """Steer along the blue line by consuming /blue_line and publishing /cmd_vel.

    Closed-loop controller (proportional on `offset`). Terminates SUCCESS once
    the tracker has reported STATE_LOST for `_END_FRAMES` consecutive frames,
    but only after at least one real STATE_LINE frame has been seen (so a
    startup-time LOST stream doesn't immediately succeed).
    """

    def __init__(self, name: str = "FollowBlueLine"):
        super().__init__(name=name)
        self._last: BlueLineStatus | None = None
        self._lost_streak = 0
        self._seen_line = False

    def setup(self, **kwargs):
        node = kwargs["node"]
        self._node = node  # kept to stamp TwistStamped with the ROS clock
        self._ros_logger: RcutilsLogger = node.get_logger()
        # Nav2 here is configured with enable_stamped_cmd_vel=true, so the drive
        # subscriber expects TwistStamped on /cmd_vel — plain Twist is ignored.
        self._cmd_pub: Publisher = node.create_publisher(TwistStamped, "/cmd_vel", 10)
        node.create_subscription(BlueLineStatus, "/blue_line", self._on_status, 10)

    def _on_status(self, msg: BlueLineStatus) -> None:
        self._last = msg

    def initialise(self):
        self._lost_streak = 0
        self._seen_line = False
        self._last = None

    def update(self) -> py_trees.common.Status:
        msg = self._last
        
        # No status received yet, wait
        if msg is None:
            self._ros_logger.debug(f"{self.name}: waiting for /blue_line messages...")
            return py_trees.common.Status.RUNNING

        # Lost line - temporary glitch or end of line
        if msg.state == BlueLineStatus.STATE_LOST:
            self._lost_streak += 1
            if self._seen_line and self._lost_streak >= _END_FRAMES: # End of line
                self._publish_stop()
                self._ros_logger.info(f"{self.name}: line ended, SUCCESS")
                return py_trees.common.Status.SUCCESS
            # Coast: zero command while we wait to confirm end-of-line.
            self._publish_stop()
            self._ros_logger.debug(f"{self.name}: line lost (streak={self._lost_streak}), coasting...")
            return py_trees.common.Status.RUNNING

        # Drive according to line offset.
        self._seen_line = True
        self._lost_streak = 0
        cmd = TwistStamped()
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.twist.linear.x = _FWD_SPEED
        cmd.twist.angular.z = -_KP * float(msg.offset_right)
        self._cmd_pub.publish(cmd)

        self._ros_logger.debug(f"{self.name}: Following line, publishing cmd_vel: linear.x={cmd.twist.linear.x:.2f} angular.z={cmd.twist.angular.z:.2f}")
        
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._publish_stop()

    def _publish_stop(self) -> None:
        stop = TwistStamped()
        stop.header.stamp = self._node.get_clock().now().to_msg()
        self._cmd_pub.publish(stop)


class CheckIfAtCTO(py_trees.behaviour.Behaviour):
    """Call /classify_face; SUCCESS iff the returned role is 'cto'."""

    def __init__(self, name: str = "CheckIfAtCTO"):
        super().__init__(name=name)
        self._future: Future | None = None
        self._start_time: float = 0.0

    def setup(self, **kwargs):
        self._node: Node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self._node.get_logger()
        self._client = self._node.create_client(ClassifyFace, "classify_face")

    def initialise(self):
        self._future = self._client.call_async(ClassifyFace.Request())
        self._start_time = self._node.get_clock().now().nanoseconds * 1e-9

    def update(self) -> py_trees.common.Status:
        if self._future is None:
            return py_trees.common.Status.FAILURE

        if not self._future.done():
            now = self._node.get_clock().now().nanoseconds * 1e-9
            if now - self._start_time < SERVICE_TIMEOUT_SEC:
                return py_trees.common.Status.RUNNING
            self._ros_logger.warning("CheckIfAtCTO: classify_face timed out")
            return py_trees.common.Status.FAILURE

        resp: ClassifyFace.Response = self._future.result()  # type: ignore
        if resp is None or not getattr(resp, "success", False):
            self._ros_logger.info(
                f"CheckIfAtCTO: classify_face failed: {getattr(resp, 'message', '?')}"
            )
            return py_trees.common.Status.FAILURE

        role = (resp.role or "").lower()
        self._ros_logger.info(f"CheckIfAtCTO: role='{resp.role}' name='{resp.name}'")
        return (
            py_trees.common.Status.SUCCESS
            if role == "cto"
            else py_trees.common.Status.FAILURE
        )

    def terminate(self, new_status):
        if (
            new_status == py_trees.common.Status.INVALID
            and self._future is not None
            and not self._future.done()
        ):
            self._future.cancel()
        self._future = None


class UTurn(py_trees_ros.action_clients.FromConstant):
    """Rotate 180deg in place using nav2's Spin recovery action."""

    def __init__(self, name: str = "UTurn"):
        goal = Spin.Goal()
        goal.target_yaw = math.pi
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
        self._ros_logger.info(f"{self.name}: spin 180deg")


class GenerateReport(py_trees.behaviour.Behaviour):
    """TODO: Log the collected task results. Stub for a future /generate_report service."""

    def __init__(self, name: str = "GenerateReport"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        for key in (
            bb.TASK_COUNT_RINGS,
            bb.TASK_INSPECT_BARRELS,
            bb.TASK_ANOMALY_RED,
            bb.TASK_ANOMALY_GREEN,
        ):
            self.bb.register_key(key=key, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        rings = self.bb.get(bb.TASK_COUNT_RINGS)
        barrels = self.bb.get(bb.TASK_INSPECT_BARRELS)
        red = self.bb.get(bb.TASK_ANOMALY_RED)
        green = self.bb.get(bb.TASK_ANOMALY_GREEN)
        self._ros_logger.info(
            f"GenerateReport TODO: rings={len(rings.rings)} "
            f"barrels={len(barrels.barrels)} (insp={len(barrels.inspections)}) "
            f"red_tiles={len(red.tiles)} green_tiles={len(green.tiles)}"
        )
        return py_trees.common.Status.SUCCESS


class _FullSpin(py_trees_ros.action_clients.FromConstant):
    """Spin 360deg in place."""

    def __init__(self, name: str = "FullSpin"):
        goal = Spin.Goal()
        goal.target_yaw = 2.0 * math.pi
        super().__init__(
            name=name,
            action_type=Spin,
            action_name="spin",
            action_goal=goal,
        )


class _Speak(py_trees.behaviour.Behaviour):
    """Fire-and-forget publish to /speak; SUCCESS immediately."""

    def __init__(self, line: str, name: str = "Speak"):
        super().__init__(name=name)
        self._line = line

    def setup(self, **kwargs):
        node = kwargs["node"]
        self._ros_logger = node.get_logger()
        self._pub: Publisher = node.create_publisher(String, "/speak", 10)

    def update(self) -> py_trees.common.Status:
        msg = String()
        msg.data = self._line
        self._pub.publish(msg)
        self._ros_logger.info(f"{self.name}: '{self._line}'")
        return py_trees.common.Status.SUCCESS


class _Shutdown(py_trees.behaviour.Behaviour):
    """Call rclpy.shutdown() so the node exits."""

    def __init__(self, name: str = "Shutdown"):
        super().__init__(name=name)

    def setup(self, **kwargs):
        self._ros_logger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        self._ros_logger.info(f"{self.name}: mission complete, shutting down")
        rclpy.shutdown()
        return py_trees.common.Status.SUCCESS


def FinalDance(name: str = "FinalDance") -> py_trees.composites.Sequence:
    """Start speaking, spin 360deg while the speech plays out, then shut down."""
    seq = py_trees.composites.Sequence(name=name, memory=True)
    seq.add_children([_Speak(_END_LINE), _FullSpin(), _Shutdown()])
    return seq


def build() -> py_trees.composites.Sequence:
    """Room 2: corridor → arm down → (follow line, check CTO, U-turn on miss)* → report."""
    # On a failed CTO check, run UTurn but report FAILURE so the Retry loop
    # restarts FollowBlueLine. Inverter flips UTurn's SUCCESS → FAILURE.
    check_or_turn = py_trees.composites.Selector(name="CTOorUTurn", memory=True)
    check_or_turn.add_children([
        CheckIfAtCTO(),
        py_trees.decorators.SuccessIsFailure(name="UTurnThenRetry", child=UTurn()),
    ])

    loop_body = py_trees.composites.Sequence(name="FollowAndCheck", memory=True)
    loop_body.add_children([FollowBlueLine(), check_or_turn])

    cto_loop = py_trees.decorators.Retry(
        name="UntilAtCTO",
        child=loop_body,
        num_failures=_MAX_CTO_ATTEMPTS,
    )

    seq = py_trees.composites.Sequence(name="Room2", memory=True)
    seq.add_children([
        GoToCorridorEntrance(),
        SetArmPosition("look_for_qr"),
        cto_loop,
        GenerateReport(),
    ])
    return seq
