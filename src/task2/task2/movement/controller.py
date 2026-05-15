import math
import threading
import time

from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from task1.movement.models import Face, Pose, Ring
from task1.movement.movement import Movement
from msg_types.msg import FaceDetect, RingDetect, SubjectClear
from msg_types.srv import WallNormalAt
from task1.movement.states.base import Params, State
from task1.movement.states.follow_path import FollowPath, FollowPathParams



class MovementController(Node):
    def __init__(self):
        super().__init__('movement')
        self._logger = self.get_logger()

        # Encapsulation of movement logic
        self._movement = Movement(self)
        self.navigate_through_poses = self._movement.navigate_through_poses
        self.follow_waypoints = self._movement.follow_waypoints
        self.navigate_to_pose = self._movement.navigate_to_pose
        self.spin = self._movement.spin

        # State management
        self._state_registry: dict[type[State], State] = {}
        self._state_lock = threading.RLock()
        self._state: State = self._state_registry.setdefault(FollowPath, FollowPath(self)) # type: ignore
        
        # Start in FollowPath state
        self._state.on_enter(FollowPathParams()) # Start in FollowPath state

        # Face detection
        self._face_detected_subscriber = self.create_subscription(FaceDetect, '/face_detect', self._on_face_detect, 10)
        self._face_clear_publisher = self.create_publisher(SubjectClear, '/face_clear', 10)
        # Ring detection
        self._ring_detected_subscriber = self.create_subscription(RingDetect, '/ring_detect', self._on_ring_detect, 10)
        self._ring_clear_publisher = self.create_publisher(SubjectClear, '/ring_clear', 10)

        # Speaking
        self._speak_publisher = self.create_publisher(String, '/speak', 10)

        # Wall query service client
        self._wall_normal_client = self.create_client(WallNormalAt, 'wall_normal_at', callback_group=ReentrantCallbackGroup())

        # Pose
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def is_active(self, state: State) -> bool:
        with self._state_lock:
            return state == self._state

    def _on_face_detect(self, msg: FaceDetect):
        with self._state_lock:
            self._state.on_face_detect(msg)

    def _on_ring_detect(self, msg: RingDetect):
        with self._state_lock:
            self._state.on_ring_detect(msg)

    def _quaternion_to_yaw(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_robot_pose(self) -> Pose | None:
        try:
            trans = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time() # type: ignore
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            theta = self._quaternion_to_yaw(trans.transform.rotation)
            
            return Pose(x, y, theta)
        except Exception as e:
            self._logger.error(f"Error occurred while fetching robot pose: {e}")
            return None

    def clear_face(self, face: Face):
        clear_msg = SubjectClear()
        clear_msg.id = face.id
        self._face_clear_publisher.publish(clear_msg)

    def clear_ring(self, ring: Ring):
        clear_msg = SubjectClear()
        clear_msg.id = ring.id
        self._ring_clear_publisher.publish(clear_msg)

    def speak(self, text: str):
        msg = String()
        msg.data = text
        self._speak_publisher.publish(msg)

    def change_state(self, new_state: type[State], old_state: type[State], params: Params):
        with self._state_lock:
            if not isinstance(self._state, old_state):
                self._logger.debug(f'{old_state.__name__} requesting state change to {new_state.__name__}, but is not the current state ({self._state.name}), ignoring')
                return
        
            self._state = self._state_registry.setdefault(new_state, new_state(self))  # type: ignore[arg-type]
            self._state.on_enter(params)

    def query_wall_normal(self, x: float, y: float) -> WallNormalAt.Response | None:
        """Call the wall_normal_at service synchronously. Returns None if unavailable or failed."""
        if not self._wall_normal_client.service_is_ready():
            self._logger.warning('wall_normal_at service not available')
            return None
        request = WallNormalAt.Request()
        request.x = float(x)
        request.y = float(y)
        future = self._wall_normal_client.call_async(request)
        # FIX THIS HORIBLE !!!
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            self._logger.warning('wall_normal_at service call timed out')
            return None
        response = future.result()
        if response is None or not response.success:
            self._logger.warning(f'wall_normal_at query failed for ({x:.2f}, {y:.2f})')
            return None
        return response

    def shutdown(self):
        self._logger.info('Shutting down movement controller...')
        super().destroy_node()
        rclpy.shutdown()
