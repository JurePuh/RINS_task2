"""Reusable odom-driven motion behaviours that publish TwistStamped directly.

These bypass nav2's Spin / DriveOnHeading recovery actions, which abort when
the projected footprint touches an inflated cell in the local costmap — a
common situation when working very close to a wall (e.g. at the belt).

Three behaviours are exposed:

- `SpinByYaw`              — odom-only rotation by a target yaw delta.
- `ApproachToWallDistance` — query /line_fit_in_direction in a given direction,
                             then odom-drive forward/back so the queried wall
                             sits at a target perpendicular distance.
- `DriveTileWithCorrection` — odom-drive a fixed straight distance while
                              periodically re-querying /line_fit_in_direction
                              in a lateral direction and applying a small
                              angular correction to maintain a constant
                              perpendicular distance to the wall.

All three take all tunables as `__init__` parameters with sensible defaults,
so callers in mission code only need to pass their targets.
"""

from __future__ import annotations

import math
from typing import Callable, Optional

import py_trees
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.client import Client
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.task import Future

from msg_types.srv import LineFitInDirection

from task2.movement.log_utils import log_throttled


# ── Default tunables ─────────────────────────────────────────────────────────

_ANGULAR_SPEED = 0.6          # rad/s
_LINEAR_SPEED = 0.15          # m/s
_YAW_TOLERANCE = 0.02         # ~1.1 deg
_DIST_TOLERANCE = 0.01        # 1 cm

_LINE_FIT_CONE_HALF_WIDTH = math.pi / 4   # 90 deg total cone
_LINE_FIT_MAX_RANGE = 4.0                 # m

_LATERAL_KP = 2.0                         # position-error gain (the "sprinkle")
_LATERAL_KD = 6.0                         # error-rate gain (the damping term)
_LATERAL_MAX_ANGULAR = 0.6
_SERVICE_PERIOD_S = 0.2                   # ~5 Hz lateral re-query
# Cap on (now - last_service_t) used when computing d_error / dt. Prevents a
# single delayed service response from producing a giant rate-of-change spike.
_LATERAL_MAX_DT_FOR_D_S = 0.5


# ── Helpers ──────────────────────────────────────────────────────────────────


def _yaw_from_odom(msg: Odometry) -> float:
    """Extract yaw (rad) from an Odometry quaternion. Z-axis component only."""
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_to_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _now_seconds(node: Node) -> float:
    """Current ROS time in seconds (float)."""
    return node.get_clock().now().nanoseconds * 1e-9


def _zero_twist(node: Node) -> TwistStamped:
    """A stamped TwistStamped with all zeros — used to halt the robot."""
    msg = TwistStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    return msg


# ── SpinByYaw ────────────────────────────────────────────────────────────────


class SpinByYaw(py_trees.behaviour.Behaviour):
    """Spin in place by a signed yaw delta using /odom feedback.

    CCW positive, matches REP-103. SUCCESS when |remaining| drops below
    `yaw_tolerance`. FAILURE on timeout. Publishes a zero twist on terminate()
    so the robot is guaranteed to stop.
    """

    def __init__(
        self,
        target_yaw_delta_rad: float,
        angular_speed: float = _ANGULAR_SPEED,
        yaw_tolerance: float = _YAW_TOLERANCE,
        timeout_s: float = 10.0,
        name: str = "SpinByYaw",
    ) -> None:
        super().__init__(name=name)
        self._target = float(target_yaw_delta_rad)
        self._angular_speed = float(angular_speed)
        self._yaw_tolerance = float(yaw_tolerance)
        self._timeout_s = float(timeout_s)

        self._node: Optional[Node] = None
        self._logger: Optional[RcutilsLogger] = None
        self._cmd_pub: Optional[Publisher] = None
        self._latest_odom: Optional[Odometry] = None

        # Per-tick state, reset in initialise().
        self._start_yaw: float = 0.0
        self._prev_yaw: float = 0.0
        self._unwrapped_delta: float = 0.0
        self._t_start: float = 0.0

    def setup(self, **kwargs) -> None:
        self._node = kwargs["node"]
        self._logger = self._node.get_logger()
        self._cmd_pub = self._node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self._node.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._logger.debug(f"{self.name}.setup complete")

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def initialise(self) -> None:
        if self._latest_odom is None:
            # Will be re-checked in update(); leave state defaulted.
            self._logger.warning(  # type: ignore[union-attr]
                f"{self.name}.initialise: no odom yet — will retry in update()"
            )
            self._start_yaw = 0.0
        else:
            self._start_yaw = _yaw_from_odom(self._latest_odom)
        self._prev_yaw = self._start_yaw
        self._unwrapped_delta = 0.0
        self._t_start = _now_seconds(self._node)  # type: ignore[arg-type]
        self._logger.info(  # type: ignore[union-attr]
            f"{self.name}: target={self._target:+.3f} rad, "
            f"start_yaw={self._start_yaw:+.3f} rad, "
            f"speed={self._angular_speed:.2f} rad/s, tol={self._yaw_tolerance:.3f} rad"
        )

    def update(self) -> py_trees.common.Status:
        if self._latest_odom is None:
            log_throttled(
                self._logger, self._node, f"{self.name}.no_odom", "debug",  # type: ignore[arg-type]
                f"{self.name}: waiting for /odom",
            )
            return py_trees.common.Status.RUNNING

        # Lazy capture of start yaw if odom was missing during initialise().
        if self._start_yaw == 0.0 and self._prev_yaw == 0.0 and self._unwrapped_delta == 0.0:
            self._start_yaw = _yaw_from_odom(self._latest_odom)
            self._prev_yaw = self._start_yaw

        current_yaw = _yaw_from_odom(self._latest_odom)
        wrapped_step = _wrap_to_pi(current_yaw - self._prev_yaw)
        self._unwrapped_delta += wrapped_step
        self._prev_yaw = current_yaw
        remaining = self._target - self._unwrapped_delta

        if abs(remaining) < self._yaw_tolerance:
            self._cmd_pub.publish(_zero_twist(self._node))  # type: ignore[union-attr]
            self._logger.info(  # type: ignore[union-attr]
                f"{self.name}: SUCCESS — rotated {self._unwrapped_delta:+.3f} rad "
                f"(target {self._target:+.3f}, residual {remaining:+.3f})"
            )
            return py_trees.common.Status.SUCCESS

        elapsed = _now_seconds(self._node) - self._t_start  # type: ignore[arg-type]
        if elapsed > self._timeout_s:
            self._cmd_pub.publish(_zero_twist(self._node))  # type: ignore[union-attr]
            self._logger.warning(  # type: ignore[union-attr]
                f"{self.name}: TIMEOUT after {elapsed:.2f}s — "
                f"rotated {self._unwrapped_delta:+.3f}, remaining {remaining:+.3f}"
            )
            return py_trees.common.Status.FAILURE

        cmd = _zero_twist(self._node)  # type: ignore[arg-type]
        cmd.twist.angular.z = math.copysign(self._angular_speed, remaining)
        self._cmd_pub.publish(cmd)  # type: ignore[union-attr]
        log_throttled(
            self._logger, self._node, f"{self.name}.spinning", "debug",  # type: ignore[arg-type]
            f"{self.name}: remaining={remaining:+.3f} rad, "
            f"angular.z={cmd.twist.angular.z:+.2f}",
        )
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._cmd_pub is not None and self._node is not None:
            self._cmd_pub.publish(_zero_twist(self._node))
        if self._logger is not None:
            self._logger.info(f"{self.name}: terminate (status={new_status})")


# ── ApproachToWallDistance ───────────────────────────────────────────────────


class ApproachToWallDistance(py_trees.behaviour.Behaviour):
    """Query /line_fit_in_direction in `direction_rad`, then drive along the
    current heading so the queried wall sits at `target_distance_m` perpendicular
    distance.

    Two internal phases: "querying" (waiting for the service response) and
    "driving" (odom-based linear motion). On a service failure returns FAILURE.

    Note on geometry: traveling `d` meters along the current heading reduces
    the queried perp distance by `d * cos(angle_to_perpendicular)`. If the
    robot is not facing the wall perpendicularly the final perp distance
    under-approaches the target by that cosine factor. For the modest yaw
    misalignments coming out of GoToBelt this is acceptable; the residual gets
    cleaned up by `DriveTileWithCorrection`'s lateral loop later.
    """

    _PHASE_QUERYING = "querying"
    _PHASE_DRIVING = "driving"

    def __init__(
        self,
        target_distance_m: float,
        direction_rad: float = 0.0,
        linear_speed: float = _LINEAR_SPEED,
        dist_tolerance: float = _DIST_TOLERANCE,
        cone_half_width: float = _LINE_FIT_CONE_HALF_WIDTH,
        max_range: float = _LINE_FIT_MAX_RANGE,
        timeout_s: float = 15.0,
        name: str = "ApproachToWallDistance",
    ) -> None:
        super().__init__(name=name)
        self._target_distance = float(target_distance_m)
        self._direction = float(direction_rad)
        self._linear_speed = float(linear_speed)
        self._dist_tolerance = float(dist_tolerance)
        self._cone_half_width = float(cone_half_width)
        self._max_range = float(max_range)
        self._timeout_s = float(timeout_s)

        self._node: Optional[Node] = None
        self._logger: Optional[RcutilsLogger] = None
        self._cmd_pub: Optional[Publisher] = None
        self._client: Optional[Client] = None
        self._latest_odom: Optional[Odometry] = None

        # Reset in initialise().
        self._phase: str = self._PHASE_QUERYING
        self._future: Optional[Future] = None
        self._t_start: float = 0.0
        self._target_drive: float = 0.0
        self._start_x: float = 0.0
        self._start_y: float = 0.0
        self._start_yaw: float = 0.0

    def setup(self, **kwargs) -> None:
        self._node = kwargs["node"]
        self._logger = self._node.get_logger()
        self._cmd_pub = self._node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self._client = self._node.create_client(
            LineFitInDirection, "/line_fit_in_direction"
        )
        self._node.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._logger.debug(f"{self.name}.setup complete")

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def initialise(self) -> None:
        req = LineFitInDirection.Request()
        req.direction = float(self._direction)
        req.cone_half_width = float(self._cone_half_width)
        req.max_range = float(self._max_range)
        self._future = self._client.call_async(req)  # type: ignore[union-attr]
        self._phase = self._PHASE_QUERYING
        self._t_start = _now_seconds(self._node)  # type: ignore[arg-type]
        self._logger.info(  # type: ignore[union-attr]
            f"{self.name}: querying line_fit (direction={self._direction:+.3f}, "
            f"cone={self._cone_half_width:.3f}, max_range={self._max_range:.2f}), "
            f"target_perp_distance={self._target_distance:.3f} m"
        )

    def update(self) -> py_trees.common.Status:
        elapsed = _now_seconds(self._node) - self._t_start  # type: ignore[arg-type]
        if elapsed > self._timeout_s:
            self._cmd_pub.publish(_zero_twist(self._node))  # type: ignore[union-attr]
            self._logger.warning(  # type: ignore[union-attr]
                f"{self.name}: TIMEOUT after {elapsed:.2f}s in phase '{self._phase}'"
            )
            return py_trees.common.Status.FAILURE

        if self._phase == self._PHASE_QUERYING:
            return self._tick_querying()
        return self._tick_driving()

    def _tick_querying(self) -> py_trees.common.Status:
        if self._future is None or not self._future.done():
            log_throttled(
                self._logger, self._node, f"{self.name}.waiting", "debug",  # type: ignore[arg-type]
                f"{self.name}: waiting for line_fit response",
            )
            return py_trees.common.Status.RUNNING

        resp: LineFitInDirection.Response = self._future.result()  # type: ignore[assignment]
        if resp is None or not resp.success:
            self._logger.warning(  # type: ignore[union-attr]
                f"{self.name}: line_fit returned success=False — aborting"
            )
            return py_trees.common.Status.FAILURE

        if self._latest_odom is None:
            log_throttled(
                self._logger, self._node, f"{self.name}.no_odom", "debug",  # type: ignore[arg-type]
                f"{self.name}: have line_fit but no odom yet",
            )
            return py_trees.common.Status.RUNNING

        self._target_drive = float(resp.perp_distance) - self._target_distance
        pose = self._latest_odom.pose.pose.position
        self._start_x = pose.x
        self._start_y = pose.y
        self._start_yaw = _yaw_from_odom(self._latest_odom)
        self._phase = self._PHASE_DRIVING
        self._logger.info(  # type: ignore[union-attr]
            f"{self.name}: line_fit perp_distance={resp.perp_distance:.3f} m, "
            f"target={self._target_distance:.3f} m, drive={self._target_drive:+.3f} m "
            f"(start xy=({self._start_x:.3f}, {self._start_y:.3f}), "
            f"start_yaw={self._start_yaw:+.3f}) — phase=driving"
        )
        return py_trees.common.Status.RUNNING

    def _tick_driving(self) -> py_trees.common.Status:
        if self._latest_odom is None:
            return py_trees.common.Status.RUNNING

        pose = self._latest_odom.pose.pose.position
        dx = pose.x - self._start_x
        dy = pose.y - self._start_y
        traveled = dx * math.cos(self._start_yaw) + dy * math.sin(self._start_yaw)
        remaining = self._target_drive - traveled

        if abs(remaining) < self._dist_tolerance:
            self._cmd_pub.publish(_zero_twist(self._node))  # type: ignore[union-attr]
            self._logger.info(  # type: ignore[union-attr]
                f"{self.name}: SUCCESS — traveled {traveled:+.3f} m "
                f"(target {self._target_drive:+.3f}, residual {remaining:+.3f})"
            )
            return py_trees.common.Status.SUCCESS

        cmd = _zero_twist(self._node)  # type: ignore[arg-type]
        cmd.twist.linear.x = math.copysign(self._linear_speed, remaining)
        self._cmd_pub.publish(cmd)  # type: ignore[union-attr]
        log_throttled(
            self._logger, self._node, f"{self.name}.driving", "debug",  # type: ignore[arg-type]
            f"{self.name}: traveled={traveled:+.3f}/{self._target_drive:+.3f} m, "
            f"linear.x={cmd.twist.linear.x:+.3f}",
        )
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._cmd_pub is not None and self._node is not None:
            self._cmd_pub.publish(_zero_twist(self._node))
        if (
            new_status == py_trees.common.Status.INVALID
            and self._future is not None
            and not self._future.done()
        ):
            self._future.cancel()
        self._future = None
        if self._logger is not None:
            self._logger.info(f"{self.name}: terminate (status={new_status})")


# ── DriveTileWithCorrection ──────────────────────────────────────────────────


class DriveTileWithCorrection(py_trees.behaviour.Behaviour):
    """Drive forward `distance_m` (odom) while periodically re-querying
    /line_fit_in_direction in `lateral_direction_rad` and applying a PD
    angular correction to hold the wall at `target_lateral_m` perpendicular
    distance.

    Control law (per tick, with the latest cached error and error-rate):

        angular_cmd = side_sign * (kp * error + kd * d_error_rate)

    where `error = target_lateral - measured_perp_distance` and
    `d_error_rate = (error - prev_error) / dt_between_service_responses`.

    Why PD: pure P on position oscillates because the controller only knows
    "I'm off", not "I'm getting closer". The D term provides damping — when
    the error is rising (we're still closing in even after a previous P
    correction), kd*d_error/dt pushes harder the other way *before* we
    overshoot. The P term stays as a small bias so a steady offset still
    gets corrected.

    Sign rule (wall on LEFT, lateral_direction ≈ +π/2): too close (error>0)
    => turn right (angular<0). With `side_sign = -sign(sin(lateral_dir))`,
    `side_sign = -1`, so positive error and positive d_error_rate both
    produce negative angular — correct. The right-wall case (`side_sign=+1`)
    flips both terms together.

    `d_error_rate` is computed *on each new service response*, not every
    tick: the underlying measurement only updates ~5 Hz, and a per-tick
    derivative would divide by ~50 ms and explode the first tick after a
    response arrived. Between responses we hold the last rate.

    Future-upgrade seam: the optional `stop_predicate(traveled_m) -> bool`,
    if provided, replaces the `traveled >= distance_m` check. Lets us add a
    "drive until anomaly service stably sees a tile" variant later without
    touching this class.
    """

    def __init__(
        self,
        distance_m: float,
        target_lateral_m: float,
        lateral_direction_rad: float = math.pi / 2,
        linear_speed: float = _LINEAR_SPEED,
        kp: float = _LATERAL_KP,
        kd: float = _LATERAL_KD,
        max_angular: float = _LATERAL_MAX_ANGULAR,
        service_period_s: float = _SERVICE_PERIOD_S,
        cone_half_width: float = _LINE_FIT_CONE_HALF_WIDTH,
        max_range: float = _LINE_FIT_MAX_RANGE,
        timeout_s: float = 20.0,
        dist_tolerance: float = _DIST_TOLERANCE,
        stop_predicate: Optional[Callable[[float], bool]] = None,
        name: Optional[str] = None,
    ) -> None:
        super().__init__(name=name or f"DriveTileWithCorrection({distance_m:.2f}m)")
        self._distance = float(distance_m)
        self._target_lateral = float(target_lateral_m)
        self._lateral_direction = float(lateral_direction_rad)
        self._linear_speed = float(linear_speed)
        self._kp = float(kp)
        self._kd = float(kd)
        self._max_angular = float(max_angular)
        self._service_period_s = float(service_period_s)
        self._cone_half_width = float(cone_half_width)
        self._max_range = float(max_range)
        self._timeout_s = float(timeout_s)
        self._dist_tolerance = float(dist_tolerance)
        self._stop_predicate = stop_predicate

        # Sign that maps lateral error -> angular command. With the wall on the
        # LEFT (lateral≈+pi/2), being too close (error>0) means we must turn
        # RIGHT (angular<0). With the wall on the RIGHT, the sign flips. So:
        #   angular = kp * error * side_sign,  side_sign = -sign(sin(lateral))
        self._side_sign: float = -math.copysign(1.0, math.sin(self._lateral_direction))
        # If the lateral direction is exactly forward/back, side_sign is undefined.
        if abs(math.sin(self._lateral_direction)) < 1e-6:
            self._side_sign = -1.0  # benign default; user shouldn't pick this.

        self._node: Optional[Node] = None
        self._logger: Optional[RcutilsLogger] = None
        self._cmd_pub: Optional[Publisher] = None
        self._client: Optional[Client] = None
        self._latest_odom: Optional[Odometry] = None

        # Reset in initialise().
        self._start_x: float = 0.0
        self._start_y: float = 0.0
        self._start_yaw: float = 0.0
        self._lateral_error: float = 0.0
        self._prev_lateral_error: Optional[float] = None  # None until 2nd response
        self._prev_response_t: float = 0.0
        self._d_error_rate: float = 0.0  # latest (m/s); held between responses
        self._pending_future: Optional[Future] = None
        self._last_service_t: float = float("-inf")
        self._t_start: float = 0.0
        self._got_first_fit: bool = False

    def setup(self, **kwargs) -> None:
        self._node = kwargs["node"]
        self._logger = self._node.get_logger()
        self._cmd_pub = self._node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self._client = self._node.create_client(
            LineFitInDirection, "/line_fit_in_direction"
        )
        self._node.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._logger.debug(f"{self.name}.setup complete")

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def initialise(self) -> None:
        if self._latest_odom is not None:
            pose = self._latest_odom.pose.pose.position
            self._start_x = pose.x
            self._start_y = pose.y
            self._start_yaw = _yaw_from_odom(self._latest_odom)
        else:
            self._start_x = 0.0
            self._start_y = 0.0
            self._start_yaw = 0.0
        self._lateral_error = 0.0
        self._prev_lateral_error = None
        self._prev_response_t = 0.0
        self._d_error_rate = 0.0
        self._pending_future = None
        self._last_service_t = float("-inf")
        self._t_start = _now_seconds(self._node)  # type: ignore[arg-type]
        self._got_first_fit = False
        self._logger.info(  # type: ignore[union-attr]
            f"{self.name}: distance={self._distance:.3f} m, "
            f"target_lateral={self._target_lateral:.3f} m, "
            f"lateral_dir={self._lateral_direction:+.3f} rad (side_sign={self._side_sign:+.0f}), "
            f"kp={self._kp:.2f}, kd={self._kd:.2f}, max_angular={self._max_angular:.2f}"
        )

    def update(self) -> py_trees.common.Status:
        if self._latest_odom is None:
            log_throttled(
                self._logger, self._node, f"{self.name}.no_odom", "debug",  # type: ignore[arg-type]
                f"{self.name}: waiting for /odom",
            )
            return py_trees.common.Status.RUNNING

        elapsed = _now_seconds(self._node) - self._t_start  # type: ignore[arg-type]
        if elapsed > self._timeout_s:
            self._cmd_pub.publish(_zero_twist(self._node))  # type: ignore[union-attr]
            self._logger.warning(  # type: ignore[union-attr]
                f"{self.name}: TIMEOUT after {elapsed:.2f}s"
            )
            return py_trees.common.Status.FAILURE

        # Re-query the service on a fixed period.
        self._maybe_poll_service()

        # Stop condition.
        pose = self._latest_odom.pose.pose.position
        dx = pose.x - self._start_x
        dy = pose.y - self._start_y
        traveled = dx * math.cos(self._start_yaw) + dy * math.sin(self._start_yaw)
        # Single line, easy to swap for a different predicate later.
        done = (
            self._stop_predicate(traveled)
            if self._stop_predicate is not None
            else (traveled >= self._distance - self._dist_tolerance)
        )
        if done:
            self._cmd_pub.publish(_zero_twist(self._node))  # type: ignore[union-attr]
            self._logger.info(  # type: ignore[union-attr]
                f"{self.name}: SUCCESS — traveled {traveled:.3f}/{self._distance:.3f} m, "
                f"last_lateral_error={self._lateral_error:+.3f} m"
            )
            return py_trees.common.Status.SUCCESS

        # PD on lateral error. Both terms share `side_sign` (see class docstring).
        p_term = self._kp * self._lateral_error
        d_term = self._kd * self._d_error_rate
        angular_cmd = self._side_sign * (p_term + d_term)
        angular_cmd = max(-self._max_angular, min(self._max_angular, angular_cmd))
        cmd = _zero_twist(self._node)  # type: ignore[arg-type]
        cmd.twist.linear.x = self._linear_speed
        cmd.twist.angular.z = angular_cmd
        self._cmd_pub.publish(cmd)  # type: ignore[union-attr]
        log_throttled(
            self._logger, self._node, f"{self.name}.driving", "debug",  # type: ignore[arg-type]
            f"{self.name}: traveled={traveled:.3f}/{self._distance:.3f} m, "
            f"error={self._lateral_error:+.3f} m, "
            f"d_error/dt={self._d_error_rate:+.3f} m/s, "
            f"P={self._side_sign * p_term:+.3f}, D={self._side_sign * d_term:+.3f}, "
            f"angular.z={angular_cmd:+.3f}",
        )
        return py_trees.common.Status.RUNNING

    def _maybe_poll_service(self) -> None:
        now = _now_seconds(self._node)  # type: ignore[arg-type]

        # Reap a completed call.
        if self._pending_future is not None and self._pending_future.done():
            resp: LineFitInDirection.Response = self._pending_future.result()  # type: ignore[assignment]
            if resp is not None and resp.success:
                new_error = self._target_lateral - float(resp.perp_distance)
                # Compute d_error/dt only when we already have a previous reading.
                # Cap dt to avoid spikes if a response was delayed (e.g. RANSAC
                # failed once and we held the previous error for a while).
                if self._prev_lateral_error is not None:
                    dt = max(min(now - self._prev_response_t, _LATERAL_MAX_DT_FOR_D_S), 1e-3)
                    self._d_error_rate = (new_error - self._prev_lateral_error) / dt
                else:
                    self._d_error_rate = 0.0
                self._prev_lateral_error = new_error
                self._prev_response_t = now
                self._lateral_error = new_error
                if not self._got_first_fit:
                    self._logger.info(  # type: ignore[union-attr]
                        f"{self.name}: first line_fit perp_distance={resp.perp_distance:.3f} m "
                        f"(target {self._target_lateral:.3f}, error {self._lateral_error:+.3f})"
                    )
                    self._got_first_fit = True
                else:
                    log_throttled(
                        self._logger, self._node, f"{self.name}.fit", "debug",  # type: ignore[arg-type]
                        f"{self.name}: line_fit perp_distance={resp.perp_distance:.3f} m, "
                        f"error={self._lateral_error:+.3f}, "
                        f"d_error/dt={self._d_error_rate:+.3f} m/s",
                    )
            else:
                log_throttled(
                    self._logger, self._node, f"{self.name}.fit_fail", "warn",  # type: ignore[arg-type]
                    f"{self.name}: line_fit failed — holding last error={self._lateral_error:+.3f}",
                )
            self._pending_future = None
            self._last_service_t = now

        # Send a fresh call if it's time and there's no inflight call.
        if self._pending_future is None and (now - self._last_service_t) >= self._service_period_s:
            req = LineFitInDirection.Request()
            req.direction = self._lateral_direction
            req.cone_half_width = self._cone_half_width
            req.max_range = self._max_range
            self._pending_future = self._client.call_async(req)  # type: ignore[union-attr]

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if self._cmd_pub is not None and self._node is not None:
            self._cmd_pub.publish(_zero_twist(self._node))
        if (
            new_status == py_trees.common.Status.INVALID
            and self._pending_future is not None
            and not self._pending_future.done()
        ):
            self._pending_future.cancel()
        self._pending_future = None
        if self._logger is not None:
            self._logger.info(f"{self.name}: terminate (status={new_status})")
