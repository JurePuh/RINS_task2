"""FollowPath behaviour: drive the robot through a hardcoded waypoint list.
"""

import py_trees
from py_trees.common import OneShotPolicy

from task2.movement import blackboard as bb
from task2.movement.behaviours._nav import LoggingNavWaypoint, build_nav_goal
from task2.movement.models import Pose


_DEFAULT_PATH: list[Pose] = [
    Pose(-0.4, -3.3, -0.5),
    Pose( 0.3, -4.4,  2.1),
    Pose(-1.2, -2.4, -2.2),
    Pose(-4.5, -2.4,  0.7),
    Pose(-2.2,  0.3, -0.3),
    Pose( 0.0, -1.1,  1.5),
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
