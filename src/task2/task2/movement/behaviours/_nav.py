"""Shared navigation helpers and thin nav-action wrappers.
"""

import math

import py_trees_ros
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.time import Time

from task2.movement.models import Pose, Point, Vector


STANDOFF = 0.4 # How far before target the goal should be set
SERVICE_TIMEOUT_SEC = 1.0


def build_nav_goal(pose: Pose) -> NavigateToPose.Goal:
    """Construct a NavigateToPose.Goal from a Pose in the map frame."""
    goal = NavigateToPose.Goal()
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose.position.x = float(pose.x)
    ps.pose.position.y = float(pose.y)
    ps.pose.orientation.z = math.sin(pose.theta / 2.0)
    ps.pose.orientation.w = math.cos(pose.theta / 2.0)
    goal.pose = ps
    return goal


def standoff_goal_from_normal(point: Point, normal: Vector) -> NavigateToPose.Goal:
    """Given a point and a normal vector, compute a standoff goal a bit back along the normal."""
    dest_x = point.x + STANDOFF * normal.x
    dest_y = point.y + STANDOFF * normal.y
    theta = math.atan2(-normal.y, -normal.x)
    return build_nav_goal(Pose(dest_x, dest_y, theta))


def standoff_goal_from_robot(robot: Point, target: Point) -> NavigateToPose.Goal:
    """Given the robot's current (x, y) and a target (x, y), compute a standoff goal a bit back from the target."""
    dx = target.x - robot.x
    dy = target.y - robot.y
    dist = math.hypot(dx, dy) or 1.0
    ux, uy = dx / dist, dy / dist
    dest_x = target.x - STANDOFF * ux
    dest_y = target.y - STANDOFF * uy
    theta = math.atan2(dy, dx)
    return build_nav_goal(Pose(dest_x, dest_y, theta))


def lookup_robot_xy(tf_buffer: tf2_ros.Buffer, logger) -> tuple[float, float] | None:
    try:
        t = tf_buffer.lookup_transform("map", "base_link", Time())
    except Exception as exc:
        logger.warning(f"tf lookup map->base_link failed: {exc}")
        return None
    return (t.transform.translation.x, t.transform.translation.y)


def _log_pose(logger, name: str, goal: NavigateToPose.Goal) -> None:
    """Log the (x, y, θ) pose from a NavigateToPose.Goal."""
    pos = goal.pose.pose.position
    q = goal.pose.pose.orientation
    theta = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
    logger.info(
        f"{name}: navigating to ({pos.x:.2f}, {pos.y:.2f}, θ={theta:.2f})"
    )


class LoggingNavWaypoint(py_trees_ros.action_clients.FromConstant):
    """FromConstant + 'navigating to pose ...' log on each fresh attempt."""

    def __init__(self, name: str, action_goal: NavigateToPose.Goal, **kwargs):
        super().__init__(
            name=name,
            action_type=NavigateToPose,
            action_name="navigate_to_pose",
            action_goal=action_goal,
            **kwargs,
        )

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._ros_logger = kwargs["node"].get_logger()

    def initialise(self):
        super().initialise()
        _log_pose(self._ros_logger, self.name, self.blackboard.goal)


class NavigateToBlackboardGoal(py_trees_ros.action_clients.FromBlackboard):
    """FromBlackboard + 'navigating to pose ...' log on each fresh attempt."""

    def __init__(self, name: str, goal_key: str, **kwargs):
        super().__init__(
            name=name,
            action_type=NavigateToPose,
            action_name="navigate_to_pose",
            key=goal_key,
            **kwargs,
        )

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._ros_logger = kwargs["node"].get_logger()

    def initialise(self):
        super().initialise()
        if self.send_goal_future is not None:
            _log_pose(self._ros_logger, self.name, self.blackboard.goal)
