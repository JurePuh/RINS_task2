"""Room 1: explore, faces, red/green anomaly, barrel check."""

import py_trees

from task2.movement.behaviours import (
    anomaly,
    barrel,
    follow_path,
    person,
)
from task2.movement.behaviours.conditions import (
    AnomalyGreenActive,
    AnomalyRedActive,
    BarrelVisitPending,
    HasUnhandledPerson,
)

def build() -> py_trees.composites.Selector:
    run_red = py_trees.composites.Sequence(name="RunAnomalyRed", memory=True)
    run_red.add_children([AnomalyRedActive(), anomaly.build_red()])

    run_green = py_trees.composites.Sequence(name="RunAnomalyGreen", memory=True)
    run_green.add_children([AnomalyGreenActive(), anomaly.build_green()])

    visit_barrel = py_trees.composites.Sequence(name="VisitHorizontalBarrel", memory=True)
    visit_barrel.add_children([BarrelVisitPending(), barrel.build()])

    approach_face = py_trees.composites.Sequence(name="ApproachUnhandledPerson", memory=True)
    approach_face.add_children([HasUnhandledPerson(), person.build()])

    room1 = py_trees.composites.Selector(name="Room1", memory=False)
    room1.add_children([
        run_red,
        run_green,
        visit_barrel,
        approach_face,
        follow_path.build(),
    ])
    return room1
