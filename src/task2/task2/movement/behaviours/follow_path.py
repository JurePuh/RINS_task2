"""FollowPath behaviour: drive the robot through a hardcoded waypoint list.
"""

import py_trees
from py_trees.common import OneShotPolicy

from task2.movement import blackboard as bb
from task2.movement.behaviours._nav import LoggingNavWaypoint, build_nav_goal
from task2.movement.models import Pose


_DEFAULT_PATH: list[Pose] = [
    Pose(-0.16, -0.91, -0.61),
    Pose( 0.33, -4.08,  1.77),
    Pose(-1.14, -4.53,  2.77),
    Pose(-2.99, -2.45, -0.82),
    Pose(-4.51, -0.70,  1.25),
    Pose(-4.52, -0.69, -0.46),
    Pose(-2.20, -0.39,  2.29),
    Pose(-2.37,  0.39, -0.05),
    Pose(-2.42, -0.26, -1.64),
    Pose(-1.15, -1.46,  1.60),
    Pose( 0.05, -0.90,  2.40),
    Pose( 2.18,  0.30, -2.90),

]


def build(path: list[Pose] | None = None) -> py_trees.composites.Sequence:
    """Construct the FollowPath sub-tree."""
    waypoints = path if path is not None else _DEFAULT_PATH

    seq = py_trees.composites.Sequence(name="FollowPath", memory=True)
    for i, p in enumerate(waypoints):
        wp = LoggingNavWaypoint(
            name=f"WP{i}({p.x:.2f},{p.y:.2f})",
            action_goal=build_nav_goal(p),
        )
        seq.add_child(py_trees.decorators.OneShot(
            name=f"Once[WP{i}]",
            child=wp,
            policy=OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
        ))
    return seq
