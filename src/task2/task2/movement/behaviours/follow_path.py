"""FollowPath behaviour: drive the robot through a hardcoded waypoint list.
"""

import py_trees
from py_trees.common import OneShotPolicy

from task2.movement import blackboard as bb
from task2.movement.behaviours._nav import LoggingNavWaypoint, build_nav_goal
from task2.movement.models import Pose


_DEFAULT_PATH: list[Pose] = [
    # Pose( 0.16, -4.31,  0.0), # B
    # Pose( 0.16, -4.31,  1.61), # B
    Pose( 0.02, -1.04, -0.67), # A
    Pose(-3.27, -0.37,  0.35), # A
    Pose(-4.23, -0.28,  1.54), # A
    Pose(-4.23, -0.28, -1.54), # A
    Pose(-4.17, -2.42, -0.13), # A
    Pose(-0.95, -4.44, -3.13), # A
    Pose( 0.09, -4.35,  0.02), # A
    Pose(-0.95, -4.44, -0.32), # A
    Pose( 0.76, -1.29,  2.33), # A
    Pose( 2.43,  0.18, -2.86), # A
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
