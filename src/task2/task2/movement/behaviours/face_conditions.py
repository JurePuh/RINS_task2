import py_trees

from task2.movement import blackboard as bb


class HasUnhandledFace(py_trees.behaviour.Behaviour):
    """Condition leaf: SUCCESS iff the pending_faces queue is non-empty, else FAILURE.
    """

    def __init__(self, name: str = "HasUnhandledFace"):
        super().__init__(name=name)

        # Blackboard access must be declared up front (py_trees enforces this).
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_FACES, access=py_trees.common.Access.READ)

        # self._ros_logger set in setup()

    def setup(self, **kwargs):
        # Get the ros logger
        try:
            node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                f"{self.qualified_name}: 'node' missing from setup kwargs"
            ) from e
        self._ros_logger = node.get_logger()

    def update(self) -> py_trees.common.Status:
        # tick: SUCCESS if there's something to do, FAILURE otherwise.
        queue = self.bb.get(bb.PENDING_FACES)
        if queue:
            self._ros_logger.debug(f"{len(queue)} pending faces in queue")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class MarkFaceHandled(py_trees.behaviour.Behaviour):
    """Cleanup leaf: pop the visited face off the queue and record its id.
    """

    def __init__(self, name: str = "MarkFaceHandled"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        # Read+write because we mutate the deque and the set in-place.
        self.bb.register_key(key=bb.PENDING_FACES, access=py_trees.common.Access.WRITE)
        self.bb.register_key(key=bb.HANDLED_FACES, access=py_trees.common.Access.WRITE)

        # self._ros_logger set in setup()

    def setup(self, **kwargs):
        # Get the ros logger
        try:
            node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                f"{self.qualified_name}: 'node' missing from setup kwargs"
            ) from e
        self._ros_logger = node.get_logger()

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_FACES)
        handled = self.bb.get(bb.HANDLED_FACES)

        face = queue.popleft()  # head of the deque is the one we just visited
        handled.add(face.id)
        self._ros_logger.info(f"face {face.id} marked handled")
        return py_trees.common.Status.SUCCESS
