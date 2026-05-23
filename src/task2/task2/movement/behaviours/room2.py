"""Room 2: exit corridor, blue-line following, CEO check, report, final dance."""

import math

import py_trees
import py_trees_ros
import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import Spin
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.publisher import Publisher
from std_msgs.msg import String

from msg_types.msg import BlueLineStatus

from task2.movement import blackboard as bb
from task2.movement.behaviours._arm import SetArmPosition
from task2.movement.behaviours._nav import LoggingNavWaypoint, build_nav_goal
from task2.movement.models import Pose


_CORRIDOR_ENTRANCE_POSE = Pose( 2.8, -0.2, -1.5)

_FWD_SPEED = 0.15      # m/s forward while following the line
_KP = 0.8              # proportional gain: angular.z = -_KP * offset
_END_FRAMES = 10       # consecutive STATE_LOST frames after first LINE → done

_END_LINE = "All anomalies inspected. I'd take a bow but I don't have hips."

_MAX_CEO_ATTEMPTS = 8  # follow-line + check tries before giving up


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
        self._ros_logger: RcutilsLogger = node.get_logger()
        self._cmd_pub: Publisher = node.create_publisher(Twist, "/cmd_vel", 10)
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
            return py_trees.common.Status.RUNNING

        # Lost line - temporary glitch or end of line
        if msg.state == BlueLineStatus.STATE_LOST:
            self._lost_streak += 1
            if self._seen_line and self._lost_streak >= _END_FRAMES:
                self._publish_stop()
                self._ros_logger.info(f"{self.name}: line ended, SUCCESS")
                return py_trees.common.Status.SUCCESS
            # Coast: zero command while we wait to confirm end-of-line.
            self._publish_stop()
            return py_trees.common.Status.RUNNING

        # Drive according to line offset.
        self._seen_line = True
        self._lost_streak = 0
        cmd = Twist()
        cmd.linear.x = _FWD_SPEED
        cmd.angular.z = -_KP * float(msg.offset)
        self._cmd_pub.publish(cmd)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._publish_stop()

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())


class CheckIfAtCEO(py_trees.behaviour.Behaviour):
    """[TODO] SUCCESS if we ended up in front of the CEO, FAILURE otherwise."""

    def __init__(self, name: str = "CheckIfAtCEO"):
        super().__init__(name=name)

    def setup(self, **kwargs):
        self._log = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        self._log.info("[TODO] CheckIfAtCEO -> assuming SUCCESS")
        return py_trees.common.Status.SUCCESS


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
    """Room 2: corridor → arm down → (follow line, check CEO, U-turn on miss)* → report."""
    # On a failed CEO check, run UTurn but report FAILURE so the Retry loop
    # restarts FollowBlueLine. Inverter flips UTurn's SUCCESS → FAILURE.
    check_or_turn = py_trees.composites.Selector(name="CEOorUTurn", memory=False)
    check_or_turn.add_children([
        CheckIfAtCEO(),
        py_trees.decorators.Inverter(name="UTurnThenRetry", child=UTurn()),
    ])

    loop_body = py_trees.composites.Sequence(name="FollowAndCheck", memory=True)
    loop_body.add_children([FollowBlueLine(), check_or_turn])

    ceo_loop = py_trees.decorators.Retry(
        name="UntilAtCEO",
        child=loop_body,
        num_failures=_MAX_CEO_ATTEMPTS,
    )

    seq = py_trees.composites.Sequence(name="Room2", memory=True)
    seq.add_children([
        GoToCorridorEntrance(),
        SetArmPosition("look_for_qr"),
        ceo_loop,
        GenerateReport(),
    ])
    return seq
