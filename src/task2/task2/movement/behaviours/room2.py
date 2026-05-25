"""Room 2: exit corridor, blue-line following, CTO check, report, final dance."""

import math
import os
import subprocess

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
from task2.movement.behaviours._odom_move import SpinByYaw
from task2.movement.log_utils import log_throttled
from task2.movement.models import Pose


_CORRIDOR_ENTRANCE_POSE = Pose( 2.85, -0.2, -1.5)

_FWD_SPEED = 0.25      # m/s forward while following the line
_KP = 4.0              # proportional gain: angular.z = -_KP * offset
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
            log_throttled(
                self._ros_logger, self._node, f"{self.name}.waiting", "debug",
                f"{self.name}: waiting for /blue_line messages...",
            )
            return py_trees.common.Status.RUNNING

        # Lost line - temporary glitch or end of line
        if msg.state == BlueLineStatus.STATE_LOST:
            prev_streak = self._lost_streak
            self._lost_streak += 1
            if prev_streak == 0:
                self._ros_logger.info(
                    f"{self.name}: line lost (streak=1), coasting "
                    f"(seen_line={self._seen_line})"
                )
            if self._seen_line and self._lost_streak >= _END_FRAMES: # End of line
                self._publish_stop()
                self._ros_logger.info(
                    f"{self.name}: line ended after {self._lost_streak} lost frames, SUCCESS"
                )
                return py_trees.common.Status.SUCCESS
            # Coast: zero command while we wait to confirm end-of-line.
            self._publish_stop()
            log_throttled(
                self._ros_logger, self._node, f"{self.name}.coasting", "debug",
                f"{self.name}: line lost (streak={self._lost_streak}/{_END_FRAMES}), coasting",
            )
            return py_trees.common.Status.RUNNING

        # Drive according to line offset.
        if not self._seen_line:
            self._ros_logger.info(
                f"{self.name}: first STATE_LINE acquired, "
                f"offset_right={msg.offset_right:.3f}"
            )
        if self._lost_streak > 0:
            self._ros_logger.info(
                f"{self.name}: line re-acquired after {self._lost_streak} lost frames"
            )
        self._seen_line = True
        self._lost_streak = 0
        cmd = TwistStamped()
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.twist.linear.x = _FWD_SPEED
        cmd.twist.angular.z = -_KP * float(msg.offset_right)
        self._cmd_pub.publish(cmd)

        log_throttled(
            self._ros_logger, self._node, f"{self.name}.driving", "debug",
            f"{self.name}: following line offset_right={msg.offset_right:.3f} "
            f"-> cmd_vel linear.x={cmd.twist.linear.x:.2f} angular.z={cmd.twist.angular.z:.2f}",
        )

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
        self._ros_logger.info(
            f"{self.name}: calling /classify_face (timeout={SERVICE_TIMEOUT_SEC}s)"
        )
        self._ros_logger.debug(
            f"{self.name}: request payload = ClassifyFace.Request() (empty)"
        )
        self._future = self._client.call_async(ClassifyFace.Request())
        self._start_time = self._node.get_clock().now().nanoseconds * 1e-9

    def update(self) -> py_trees.common.Status:
        if self._future is None:
            return py_trees.common.Status.FAILURE

        if not self._future.done():
            now = self._node.get_clock().now().nanoseconds * 1e-9
            elapsed = now - self._start_time
            if elapsed < SERVICE_TIMEOUT_SEC:
                log_throttled(
                    self._ros_logger, self._node, f"{self.name}.waiting", "debug",
                    f"{self.name}: waiting for classify_face response "
                    f"(elapsed={elapsed:.2f}s)",
                )
                return py_trees.common.Status.RUNNING
            self._ros_logger.warning(
                f"CheckIfAtCTO: classify_face timed out after {elapsed:.2f}s"
            )
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


class GenerateReport(py_trees.behaviour.Behaviour):
    """Build the inspection PDF from blackboard task results and open it."""

    _OUT_DIR = os.path.expanduser("~/LOCAL/Faks/RInS/project2_ws/src/task2/reports")

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
        from task2.movement.report import build_report

        candidates = [
            (self.bb.get(bb.TASK_COUNT_RINGS), "Ring Counting"),
            (self.bb.get(bb.TASK_INSPECT_BARRELS), "Barrel Inspection"),
            (self.bb.get(bb.TASK_ANOMALY_RED), "Anomaly Detection (Red)"),
            (self.bb.get(bb.TASK_ANOMALY_GREEN), "Anomaly Detection (Green)"),
        ]
        tasks = [(t, title) for t, title in candidates if t is not None and t.was_asked_for]
        self._ros_logger.info(
            f"{self.name}: starting PDF build for {len(tasks)} task(s): "
            f"{[title for _, title in tasks]}"
        )
        self._ros_logger.debug(
            f"{self.name}: task inputs = "
            f"{[(title, repr(t)) for t, title in tasks]}"
        )

        try:
            path = build_report(tasks, self._OUT_DIR, self._ros_logger)
            self._ros_logger.info(f"GenerateReport: wrote {path}")
        except Exception as e:
            self._ros_logger.error(f"GenerateReport: failed to build PDF: {e}")
            return py_trees.common.Status.SUCCESS

        try:
            subprocess.Popen(["xdg-open", path])
            self._ros_logger.info(f"{self.name}: opened viewer for {path}")
        except Exception as e:
            self._ros_logger.warn(f"GenerateReport: could not open viewer: {e}")
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
        self._ros_logger.info(f"{self.name}: mission complete, calling rclpy.shutdown()")
        rclpy.shutdown()
        self._ros_logger.info(f"{self.name}: rclpy.shutdown() returned")
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
        py_trees.decorators.SuccessIsFailure(
            name="UTurnThenRetry",
            child=SpinByYaw(target_yaw_delta_rad=math.pi, name="UTurn"),
        ),
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
        # TODO For debugging in ------
        GenerateReport(),
        # For debugging out -----
        SetArmPosition("look_for_qr", arm_settle_delay=0.0),
        GoToCorridorEntrance(),
        cto_loop,
        GenerateReport(),
    ])
    return seq
