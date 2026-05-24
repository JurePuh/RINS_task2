"""Shared arm-command behaviour. Publishes a pose name on /arm_command."""

import time

import py_trees
from std_msgs.msg import String
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger

from task2.movement.log_utils import log_throttled


_ARM_SETTLE_SEC = 4.0


class SetArmPosition(py_trees.behaviour.Behaviour):
    """Publish an arm pose name on /arm_command, then RUNNING for ~1s to settle."""

    def __init__(self, pose_string: str, name: str | None = None):
        super().__init__(name=name or f"SetArmPosition({pose_string})")
        self._pose_string = pose_string
        self._deadline: float | None = None

    def setup(self, **kwargs):
        self._node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self._node.get_logger()
        self._publisher: Publisher = self._node.create_publisher(String, "/arm_command", 10)

    def initialise(self):
        msg = String()
        msg.data = self._pose_string
        self._publisher.publish(msg)
        self._ros_logger.info(f"{self.name}: published '{self._pose_string}'")
        self._ros_logger.debug(
            f"{self.name}: settle_sec={_ARM_SETTLE_SEC} pose_string='{self._pose_string}'"
        )
        self._deadline = time.monotonic() + _ARM_SETTLE_SEC

    def update(self) -> py_trees.common.Status:
        if self._deadline is None or time.monotonic() >= self._deadline:
            self._ros_logger.info(f"{self.name}: settle complete")
            return py_trees.common.Status.SUCCESS
        log_throttled(
            self._ros_logger, self._node, f"{self.name}.settling", "debug",
            f"{self.name}: settling, "
            f"remaining={self._deadline - time.monotonic():.2f}s",
        )
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._ros_logger.info(
            f"{self.name}: terminate status={new_status.name}"
        )
        self._deadline = None
