import py_trees
import rclpy.client
from task2.detect_faces.main import Face
import tf2_ros
from rclpy.node import Node

from msg_types.srv import WallNormalAt
from task2.movement import blackboard as bb
from task2.movement.behaviours._nav import (
    NavigateToBlackboardGoal,
    SERVICE_TIMEOUT_SEC,
    build_nav_goal,
    lookup_robot_xy,
    standoff_goal_from_normal,
    standoff_goal_from_robot,
)


class ComputeFaceDestination(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "ComputeFaceDestination"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_FACES, access=py_trees.common.Access.READ)
        self.bb.register_key(key=bb.FACE_DESTINATION, access=py_trees.common.Access.WRITE)
        self.bb.register_key(key=bb.RECOMPUTE_DESTINATION, access=py_trees.common.Access.WRITE)

        self.node: Node
        self.client: rclpy.client.Client
        self.tf_buffer: tf2_ros.Buffer
        self.tf_listener: tf2_ros.TransformListener

        self._future = None
        self._face: Face | None = None
        self._start_time: float = 0.0

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self._ros_logger = self.node.get_logger()
        # Set up wall_normal_at client
        self.client = self.node.create_client(WallNormalAt, "wall_normal_at")
        # Set up TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        queue = self.bb.get(bb.PENDING_FACES); assert queue, f"{self.qualified_name}: initialise called with empty pending_faces queue"
        self._face = queue[0]; assert self._face, f"{self.qualified_name}: initialise set _face to None"
        self.bb.set(bb.RECOMPUTE_DESTINATION, False)

        self._future = None
        self._start_time = self.node.get_clock().now().nanoseconds * 1e-9
        face = self._face
        if self.client.service_is_ready():
            req = WallNormalAt.Request()
            req.x = float(face.x)
            req.y = float(face.y)
            self._future = self.client.call_async(req)
            self._ros_logger.info(
                f"querying wall_normal_at({face.x:.2f}, {face.y:.2f}) "
                f"for face {face.id}"
            )
        else:
            self._ros_logger.info(
                f"wall_normal_at not available; using fallback for face {face.id}"
            )

    def update(self) -> py_trees.common.Status:
        face = self._face; assert face, f"{self.qualified_name}: update called before initialise set _face"

        # Check if wall_normal_at returned yet
        if self._future is not None:
            if self._future.done(): # Compute goal from response
                resp = self._future.result()
                if resp is not None and getattr(resp, "success", False):
                    goal = standoff_goal_from_normal(
                        resp.point_x, resp.point_y, resp.normal_x, resp.normal_y
                    )
                    self.bb.set(bb.FACE_DESTINATION, goal)
                    return py_trees.common.Status.SUCCESS
                self._ros_logger.warning(
                    f"wall_normal_at returned failure for face {face.id}; fallback"
                )
                self._future = None
            else: # Check for timeout
                now = self.node.get_clock().now().nanoseconds * 1e-9
                if now - self._start_time < SERVICE_TIMEOUT_SEC:
                    return py_trees.common.Status.RUNNING
                self._ros_logger.warning(
                    f"wall_normal_at timed out for face {face.id}; fallback"
                )
                self._future = None

        # Fallback: navigate directly to face position with a fixed standoff
        robot_xy = lookup_robot_xy(self.tf_buffer, self._ros_logger)
        if robot_xy is None:
            # Fallback 2: navigate straight to face
            self._ros_logger.warning(
                f"no robot pose; navigating directly to face {face.id}"
            )
            self.bb.set(bb.FACE_DESTINATION, build_nav_goal(face.x, face.y, 0.0))
            return py_trees.common.Status.SUCCESS

        self.bb.set(bb.FACE_DESTINATION, standoff_goal_from_robot(robot_xy, face.x, face.y))
        return py_trees.common.Status.SUCCESS


def build() -> py_trees.composites.Sequence:
    """Construct the GoToFace sub-tree (compute destination, then drive there)."""
    seq = py_trees.composites.Sequence(name="GoToFace", memory=True)
    seq.add_children([
        ComputeFaceDestination(),
        NavigateToBlackboardGoal(
            name="NavigateToFaceDestination",
            goal_key=bb.FACE_DESTINATION,
            recompute_key=bb.RECOMPUTE_DESTINATION,
        ),
    ])
    return seq
