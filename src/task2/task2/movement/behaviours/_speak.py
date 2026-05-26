"""Shared Speak behavior — fire-and-forget publish to /speak."""

import py_trees
from rclpy.publisher import Publisher
from std_msgs.msg import String


class Speak(py_trees.behaviour.Behaviour):
    """Fire-and-forget publish to /speak; SUCCESS immediately."""

    def __init__(self, line: str, name: str = "Speak"):
        super().__init__(name=name)
        self._line = line

    def setup(self, **kwargs):
        node = kwargs["node"]
        self._ros_logger = node.get_logger()
        self._pub: Publisher = node.create_publisher(String, "/speak", 10)

    def update(self) -> py_trees.common.Status:
        msg = String()
        msg.data = self._line
        self._pub.publish(msg)
        self._ros_logger.info(f"{self.name}: '{self._line}'")
        return py_trees.common.Status.SUCCESS
