"""Tiny leaf behaviours that read/mutate the face queue.

These exist so the tree's *structure* (a Sequence guarded by a condition,
followed by a cleanup step) carries the logic, instead of one big monolithic
behaviour doing everything internally. That's the whole point of py_trees.
"""

import py_trees

from task2.movement import blackboard as bb


class HasUnhandledFace(py_trees.behaviour.Behaviour):
    """Condition leaf: SUCCESS iff the pending_faces queue is non-empty.

    Pure read, no side effects, no RUNNING state — a condition either holds
    right now or it doesn't. When it FAILS, the parent Sequence fails and the
    parent Selector falls through to FollowPath.
    """

    def __init__(self, name: str = "HasUnhandledFace"):
        super().__init__(name=name)
        # Blackboard access must be declared up front (py_trees enforces this).
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_FACES, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        # tick: SUCCESS if there's something to do, FAILURE otherwise.
        queue = self.bb.get(bb.PENDING_FACES)
        if queue:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class MarkFaceHandled(py_trees.behaviour.Behaviour):
    """Cleanup leaf: pop the visited face off the queue and record its id.

    Runs as the final child of `ApproachUnhandledFace`, so it only fires after
    a successful GoToFace. Always returns SUCCESS — the cleanup itself can't
    fail (an empty queue here would mean the tree is in an impossible state).
    """

    def __init__(self, name: str = "MarkFaceHandled"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        # Read+write because we mutate the deque and the set in-place.
        self.bb.register_key(key=bb.PENDING_FACES, access=py_trees.common.Access.WRITE)
        self.bb.register_key(key=bb.HANDLED_FACES, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_FACES)
        handled = self.bb.get(bb.HANDLED_FACES)

        face = queue.popleft()  # head of the deque is the one we just visited
        handled.add(face.id)
        self.logger.info(f"face {face.id} marked handled")
        return py_trees.common.Status.SUCCESS
