"""Room 2: exit corridor, blue-line following, CTO check, report, final dance."""

import math
import os
import subprocess

import py_trees
import py_trees_ros
import rclpy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import Spin
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from rclpy.node import Node
from rclpy.task import Future

from msg_types.msg import BlueLineStatus
from msg_types.srv import ClassifyFace

from task2.movement import blackboard as bb
from task2.movement.behaviours._arm import SetArmPosition
from task2.movement.behaviours._speak import Speak
from task2.movement.behaviours._nav import LoggingNavWaypoint, SERVICE_TIMEOUT_SEC, build_nav_goal
from task2.movement.behaviours._odom_move import SpinByYaw
from task2.movement.log_utils import log_throttled
from task2.movement.models import Person, Pose


_CORRIDOR_ENTRANCE_POSE = Pose( 2.85, -0.2, -1.5)

_FWD_SPEED = 0.25      # m/s forward while following the line
_KP = 4.0              # proportional gain: angular.z = -_KP * offset
_END_FRAMES = 10       # consecutive STATE_LOST frames after first LINE → done

_OBSTACLE_DIST = 0.4          # m — bin avg below this → blocked
_OBSTACLE_CONE = math.pi / 4  # rad — forward arc, split into _OBSTACLE_BINS
_OBSTACLE_BINS = 7            # number of equal-width direction buckets across the cone
_LIDAR_FORWARD_OFFSET = -math.pi/2  # rad — scan-frame angle that points robot-forward,
                                # determined empirically (closest-beam debug log with
                                # robot squared to a wall). TF yaw alone disagreed.

_END_LINE = "All anomalies inspected. I'd take a bow but I don't have hips."

_MAX_CTO_ATTEMPTS = 8  # follow-line + check tries before giving up

_NEARBY_FACE_DIST = 1.0  # m — search radius for the pending face to turn towards


def _wrap_to_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class TurnTowardsNearbyFace(py_trees_ros.action_clients.FromConstant):
    """Spin so the camera frames a nearby pending face before CheckIfAtCTO.

    Picks the closest Person in PENDING_PEOPLE within `_NEARBY_FACE_DIST` of the
    robot and spins to face it. If none qualifies or tf fails, no-op SUCCESS.
    """

    def __init__(self, name: str = "TurnTowardsNearbyFace"):
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
        if not queue:
            self._ros_logger.info(f"{self.name}: no pending people; skipping")
            self._skip = True
            return

        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except Exception as exc:
            self._ros_logger.warning(
                f"{self.name}: tf lookup map->base_link failed: {exc}; skipping"
            )
            self._skip = True
            return

        rx = t.transform.translation.x
        ry = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)

        nearest: Person | None = None
        nearest_d = _NEARBY_FACE_DIST
        for p in queue:
            d = math.hypot(p.x - rx, p.y - ry)
            if d <= nearest_d:
                nearest_d = d
                nearest = p

        if nearest is None:
            self._ros_logger.info(
                f"{self.name}: no pending face within {_NEARBY_FACE_DIST:.2f}m; skipping"
            )
            self._skip = True
            return

        desired = math.atan2(nearest.y - ry, nearest.x - rx)
        self._spin_goal.target_yaw = _wrap_to_pi(desired - yaw)
        self._ros_logger.info(
            f"{self.name}: face_id={nearest.face_id} at d={nearest_d:.2f}m, "
            f"spin {self._spin_goal.target_yaw:+.2f} rad"
        )
        super().initialise()

    def update(self):
        if self._skip:
            return py_trees.common.Status.SUCCESS
        return super().update()


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
        self._scan: LaserScan | None = None
        self._lost_streak = 0
        self._seen_line = False
        self._state: str = "init"

    def setup(self, **kwargs):
        node = kwargs["node"]
        self._node = node  # kept to stamp TwistStamped with the ROS clock
        self._ros_logger: RcutilsLogger = node.get_logger()
        # Nav2 here is configured with enable_stamped_cmd_vel=true, so the drive
        # subscriber expects TwistStamped on /cmd_vel — plain Twist is ignored.
        self._cmd_pub: Publisher = node.create_publisher(TwistStamped, "/cmd_vel", 10)
        node.create_subscription(BlueLineStatus, "/blue_line", self._on_status, 10)
        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(LaserScan, "/scan_filtered", self._on_scan, scan_qos)

    def _on_status(self, msg: BlueLineStatus) -> None:
        self._last = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _lidar_snapshot(self) -> list[float | None]:
        """Return mean range per direction bucket across the forward cone.

        The forward ±_OBSTACLE_CONE/2 arc is split into _OBSTACLE_BINS equal-width
        buckets, ordered left → right. A bin's value is None when no valid beam
        falls in it.
        """
        scan = self._scan
        if scan is None:
            return [None] * _OBSTACLE_BINS
        half_cone = _OBSTACLE_CONE / 2.0
        bin_width = _OBSTACLE_CONE / _OBSTACLE_BINS
        sums = [0.0] * _OBSTACLE_BINS
        counts = [0] * _OBSTACLE_BINS
        min_r = float("inf")
        min_raw_angle = 0.0
        min_rel_angle = 0.0
        angle = scan.angle_min
        for r in scan.ranges:
            # Track global closest valid beam (no cone filter) for debugging.
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max and r < min_r:
                min_r = r
                min_raw_angle = angle
                min_rel_angle = (angle - _LIDAR_FORWARD_OFFSET + math.pi) % (2.0 * math.pi) - math.pi
            # Angle relative to robot forward; wrap to (-π, π].
            rel = (angle - _LIDAR_FORWARD_OFFSET + math.pi) % (2.0 * math.pi) - math.pi
            if -half_cone <= rel <= half_cone:
                # Skip invalid measurements
                if math.isnan(r) or math.isinf(r) or r > scan.range_max or r < scan.range_min:
                    continue
                else:
                    value = r
                idx = int((half_cone - rel) / bin_width)
                if idx == _OBSTACLE_BINS:
                    idx = _OBSTACLE_BINS - 1
                sums[idx] += value
                counts[idx] += 1
            angle += scan.angle_increment
        if math.isfinite(min_r):
            log_throttled(
                self._ros_logger, self._node, f"{self.name}.lidar_min", "debug",
                f"{self.name}: closest beam r={min_r:.3f}m at scan_angle="
                f"{min_raw_angle:.3f}rad ({math.degrees(min_raw_angle):.1f}°), "
                f"rel_to_forward={min_rel_angle:.3f}rad "
                f"({math.degrees(min_rel_angle):.1f}°)",
            )
        return [
            (sums[i] / counts[i]) if counts[i] > 0 else None
            for i in range(_OBSTACLE_BINS)
        ]

    def _set_state(self, new_state: str, detail: str = "") -> None:
        if new_state == self._state:
            return
        self._ros_logger.info(
            f"{self.name}: state {self._state} → {new_state}"
            + (f" ({detail})" if detail else "")
        )
        self._state = new_state

    def initialise(self):
        self._lost_streak = 0
        self._seen_line = False
        self._last = None
        self._scan = None
        self._state = "init"

    def update(self) -> py_trees.common.Status:
        msg = self._last

        means = self._lidar_snapshot()
        blocked = any(m is not None and m < _OBSTACLE_DIST for m in means)
        bin_str = ", ".join(
            f"b{i}={m:.2f}" if m is not None else f"b{i}=--"
            for i, m in enumerate(means)
        )
        log_throttled(
            self._ros_logger, self._node, f"{self.name}.lidar", "debug",
            f"{self.name}: lidar bins [{bin_str}] thresh={_OBSTACLE_DIST:.2f}m "
            f"→ {'HALT' if blocked else 'go'}",
        )

        if blocked:
            self._publish_stop()
            self._set_state("blocked", f"bins=[{bin_str}]")
            return py_trees.common.Status.SUCCESS

        # No status received yet, wait
        if msg is None:
            self._set_state("waiting", "no /blue_line yet")
            log_throttled(
                self._ros_logger, self._node, f"{self.name}.waiting", "debug",
                f"{self.name}: waiting for /blue_line messages...",
            )
            return py_trees.common.Status.RUNNING

        # Lost line - temporary glitch or end of line
        if msg.state == BlueLineStatus.STATE_LOST:
            self._lost_streak += 1
            if self._seen_line and self._lost_streak >= _END_FRAMES: # End of line
                self._publish_stop()
                self._set_state("ended", f"{self._lost_streak} lost frames")
                return py_trees.common.Status.SUCCESS
            # Coast: zero command while we wait to confirm end-of-line.
            self._publish_stop()
            self._set_state("coasting", f"streak={self._lost_streak}/{_END_FRAMES}, seen_line={self._seen_line}")
            log_throttled(
                self._ros_logger, self._node, f"{self.name}.coasting", "debug",
                f"{self.name}: line lost (streak={self._lost_streak}/{_END_FRAMES}), coasting",
            )
            return py_trees.common.Status.RUNNING

        # Drive according to line offset.
        self._set_state("following", f"offset_right={msg.offset_right:.3f}")
        self._seen_line = True
        self._lost_streak = 0
        cmd = TwistStamped()
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.twist.linear.x = _FWD_SPEED
        cmd.twist.angular.z = -_KP * float(msg.offset_right)
        self._cmd_pub.publish(cmd) # TODO debug

        # log_throttled( # TODO debug
        #     self._ros_logger, self._node, f"{self.name}.driving", "debug",
        #     f"{self.name}: following line offset_right={msg.offset_right:.3f} "
        #     f"-> cmd_vel linear.x={cmd.twist.linear.x:.2f} angular.z={cmd.twist.angular.z:.2f}",
        # )

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
    seq.add_children([Speak(_END_LINE), _FullSpin(), _Shutdown()])
    return seq


def build() -> py_trees.composites.Sequence:
    """Room 2: corridor → arm down → (follow line, check CTO, U-turn on miss)* → report."""
    # On a failed CTO check, run UTurn but report FAILURE so the Retry loop
    # restarts FollowBlueLine. Inverter flips UTurn's SUCCESS → FAILURE.
    check_or_turn = py_trees.composites.Selector(name="CTOorUTurn", memory=True)
    check_sequence = py_trees.composites.Sequence(name="FaceThenCheck", memory=True)
    check_sequence.add_children([TurnTowardsNearbyFace(), CheckIfAtCTO()])
    check_or_turn.add_children([
        check_sequence,
        py_trees.decorators.SuccessIsFailure( # TODO Debug
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
        SetArmPosition("look_for_qr", arm_settle_delay=0.0),
        # GoToCorridorEntrance(), # TODO debug
        cto_loop,
        GenerateReport(),
    ])
    return seq
