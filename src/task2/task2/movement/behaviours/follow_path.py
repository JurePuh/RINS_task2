"""FollowPath behaviour: drive the robot through a hardcoded waypoint list.

This is built as a Sequence (memory=True) of nav2 NavigateToPose action calls,
one per waypoint. Each child is a `FromConstant` action client behaviour:
its goal is baked in at construction time.

Why a Sequence with memory? memory=True means the Sequence remembers which
child it was on when interrupted. If a face detection preempts FollowPath
halfway through waypoint 3, when we come back the Sequence resumes at 3
instead of restarting at 0.

When the last waypoint succeeds the whole Sequence returns SUCCESS, and the
root Selector (memory=False) will simply tick FollowPath again, restarting
the loop. That's good enough for now; the user can tune it later.
"""

import math

import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node

from task2.movement.models import Pose


class _LoggingNavWaypoint(py_trees_ros.action_clients.FromConstant):
    """FromConstant that logs 'navigating to pose ...' on each fresh attempt."""

    def __init__(self, name: str, pose: Pose, **kwargs):
        super().__init__(name=name, **kwargs)
        self._pose = pose

        # self._ros_logger set in setup()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        try:
            self.node: Node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                f"{self.qualified_name}: 'node' missing from setup kwargs"
            ) from e
        self._ros_logger = self.node.get_logger()

    def initialise(self):
        super().initialise()

        p = self._pose
        self._ros_logger.info(
            f"{self.name}: navigating to pose "
            f"({p.x:.2f}, {p.y:.2f}, θ={p.theta:.2f})"
        )


# Placeholder waypoints — replace with task2 map waypoints once the new map
# is known. Kept as a module constant so it's easy to find and tweak.
_DEFAULT_PATH: list[Pose] = [
    Pose(-0.4, -3.3, -0.5),
    Pose( 0.3, -4.4,  2.1),
    Pose(-1.2, -2.4, -2.2),
    Pose(-4.5, -2.4,  0.7),
    Pose(-2.2,  0.3, -0.3),
    Pose( 0.0, -1.1,  1.5),
    Pose( 2.8, -0.2, -1.5),
]


def _pose_to_goal(pose: Pose) -> NavigateToPose.Goal:
    """Build a nav2 NavigateToPose goal from our simple Pose.

    nav2 takes a PoseStamped in the map frame. We bake the quaternion from
    the yaw angle here (z = sin(θ/2), w = cos(θ/2); x = y = 0 for a 2D yaw).
    """
    goal = NavigateToPose.Goal()
    ps = PoseStamped()
    ps.header.frame_id = "map"
    # NOTE: stamp left at 0 on purpose. nav2 treats a zero stamp as "latest"
    # which is what we want for a static, pre-known waypoint.
    ps.pose.position.x = pose.x
    ps.pose.position.y = pose.y
    ps.pose.orientation.z = math.sin(pose.theta / 2.0)
    ps.pose.orientation.w = math.cos(pose.theta / 2.0)
    goal.pose = ps
    return goal


def build(path: list[Pose] | None = None) -> py_trees.composites.Sequence:
    """Construct the FollowPath sub-tree."""
    waypoints = path if path is not None else _DEFAULT_PATH

    seq = py_trees.composites.Sequence(name="FollowPath", memory=True)
    for i, p in enumerate(waypoints):
        child = _LoggingNavWaypoint(
            name=f"WP{i}({p.x:.2f},{p.y:.2f})",
            pose=p,
            action_type=NavigateToPose,
            action_name="navigate_to_pose",
            action_goal=_pose_to_goal(p),
        )
        seq.add_child(child)
    return seq
