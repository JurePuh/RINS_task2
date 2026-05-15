import math
import threading
import time
from typing import TYPE_CHECKING

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped

from task1.movement.models import Face, Pose, Ring, Subject
from msg_types.msg import FaceDetect, RingDetect
from task1.movement.states.base import Params, State

if TYPE_CHECKING:
    from task1.movement.controller import MovementController
    from task1.movement.movement import MovGoal


class GoToSubjectParams(Params):
    def __init__(self, subject: Subject, ):
        super().__init__()
        self.subject = subject

class GoToSubject(State[GoToSubjectParams]):
    def __init__(self, controller: 'MovementController'):
        super().__init__(controller)

        self._goal: MovGoal | None = None # Current active goal, if any
        self._goal_lock = threading.RLock() # Lock for accessing current

        self._subject: Subject | None = None # Current subject we are going to

        # per-goal flags
        self._force_update_pose = False # Flag to force updating pose on next detection, used when goal gets aborted.
        self._current_accuracy_level = 0 # Flag to help update destination accuracy at closer distances.

    def on_enter(self, params: GoToSubjectParams):
        # Reset some variables
        self._current_accuracy_level = 0 # To update destination accuracy as we get closer
        with self._goal_lock:
            self._goal = None

        self._subject = params.subject

        self._logger.info(f'Entering GoToSubject, traveling to {self._subject.type_str} {self._subject.id} at ({self._subject.pose.x:.2f}, {self._subject.pose.y:.2f})')

        self._start_update_nav_goal(self._subject)

    def _start_update_nav_goal(self, subject: Subject, robot_pose: Pose | None = None):
        """Start or update navigation goal to given subject, with optional robot pose to compute more accurate destination."""
        from task1.movement.states.greet_subject import GreetSubject, GreetSubjectParams

        # Feedback called during navigation
        def feedback_callback(distance_remaining: float, estimated_time_remaining: Duration, current_pose_ts: PoseStamped):
            if not self._controller.is_active(self): # Check if still current state
                self._logger.debug(f'Feedback received for GoToSubject, but isnt current state, ignoring.')
                return

            if (time.time() % 1) < 0.01: # Log every second
                self._logger.debug(f'Going to {subject.type_str} {subject.id}, distance remaining: {distance_remaining:.2f}m, estimated time remaining: {estimated_time_remaining.sec}s')

            if distance_remaining < 1.0 and self._current_accuracy_level == 0: # Update destination to be more accurite
                self._logger.debug(f'Within 1m of {subject.type_str} {subject.id}, updating navigation goal to be more accurate')
                self._current_accuracy_level = 1
                cur_pose = Pose(current_pose_ts.pose.position.x, current_pose_ts.pose.position.y, None)
                self._start_update_nav_goal(subject, cur_pose) # Restart goal with updated destination
            elif 0.05 < distance_remaining < 0.1: # Start greeting, 0.05 as nav2 sometimes says 0 when far away?
                # FIX THIS; NOT NEEDED ANYMORE IF WE GOT WALL NORMAL !!!
                self._logger.info(f'Close to {subject.type_str} {subject.id}: {distance_remaining:.2f}m, exiting navigation early, switching to GreetSubject state')
                # Cancel goal
                with self._goal_lock:
                    assert self._goal is not None, 'Feedback callback for GoToSubject received but self._goal is None'
                    self._goal.cancel()
                    self._goal = None
                # Change state
                params = GreetSubjectParams(subject)
                self._controller.change_state(new_state=GreetSubject, old_state=self.__class__, params=params)

        # Result called when navigation is complete
        def result_callback(result: 'MovGoal.Result'):
            from task1.movement.movement import MovGoal

            assert subject is not None, 'Result callback for GoToSubject received but subject is None'

            if result == MovGoal.Result.SUCCEEDED:
                self._logger.info(f'Arrived at {subject.type_str} {subject.id}. Switching to GreetSubject state.')
                params = GreetSubjectParams(subject)
                self._controller.change_state(new_state=GreetSubject, old_state=self.__class__, params=params)
            elif result == MovGoal.Result.CANCELED:
                self._logger.info(f'Navigation to {subject.type_str} {subject.id} canceled, likely due to state change or updating navigation goal.')
            elif result == MovGoal.Result.ABORTED:
                self._logger.warning(f'Navigation to {subject.type_str} {subject.id} aborted by server. Retrying on next detection, if still current state.')
                if self._controller.is_active(self): # Only set flag if still current state, otherwise next state will handle it
                    self._force_update_pose = True # Force updating pose on next detection.
            else:
                raise RuntimeError(f'Navigation to subject {subject.id} ended with unknown result: {result}')

        destination = self._compute_destination(subject, robot_pose)

        self._logger.info(f'Starting / updating navigation to {subject.type_str} {subject.id}, with destination ({destination.x:.2f}, {destination.y:.2f})')

        # Run navigation + log
        with self._goal_lock:
            if self._goal is not None and not self._goal.has_ended:
                self._logger.debug(f'Canceling old navigation goal to {subject.type_str} {subject.id} before starting new one')
                self._goal.cancel() # Cancel any previous goals
            else:
                self._logger.debug(f'Starting new navigation goal to {subject.type_str} {subject.id}')

            self._goal = self._controller.navigate_to_pose(destination, feedback_callback, result_callback, caller_name=f'GoToSubject({subject.id})')

    def _compute_destination(self, subject: Subject, robot_pose: Pose | None = None) -> Pose:
        """Compute navigation destination pose based on subject position and optionally robot position for more accuracy."""
        subject_pose = subject.pose # thread-safe access to subject pose
        if robot_pose is None: # Get it directly, can still be None, if it is unavailable!
            self._logger.debug('Robot pose is None, figuring out pose manually')
            robot_pose = self._controller.get_robot_pose()

        # Query wall normal at the subject's position and go 40cm in front of the wall
        wall = self._controller.query_wall_normal(subject_pose.x, subject_pose.y)
        if wall is not None:
            dest_x = wall.point_x + 0.4 * wall.normal_x
            dest_y = wall.point_y + 0.4 * wall.normal_y
            theta = math.atan2(-wall.normal_y, -wall.normal_x)  # face the wall
            self._logger.info(f'Wall normal found: wall=({wall.point_x:.2f}, {wall.point_y:.2f}), normal=({wall.normal_x:.2f}, {wall.normal_y:.2f}), destination=({dest_x:.2f}, {dest_y:.2f}, {theta:.2f})')
            return Pose(dest_x, dest_y, theta)

        # 

        # Fallback: go to a point slightly before the subject along the robot→subject vector
        if robot_pose is not None:
            distance = math.sqrt((subject_pose.x - robot_pose.x)**2 + (subject_pose.y - robot_pose.y)**2)
            dx = (subject_pose.x - robot_pose.x) / distance
            dy = (subject_pose.y - robot_pose.y) / distance
            return Pose(subject_pose.x - 0.4 * dx, subject_pose.y - 0.4 * dy, 0.0)

        self._logger.warning('Couldnt get robot pose or wall normal, going directly to subject pose')
        return Pose(subject_pose.x, subject_pose.y, 0.0)

    def on_face_detect(self, msg: FaceDetect):
        if not self._controller.is_active(self): # Cant fire if not current, but still...
             self._logger.debug(f'Face detected message received but state is {self.name}, ignoring')
             return
        if isinstance(self._subject, Ring):
            self._logger.debug(f'Face detected message received but current subject is ring, ignoring')
            return

        # Silly type asserts
        assert self._subject is not None, 'Face detected message received but self._subject is None'

        # Update subject position if it had shifted, restart goal
        subject_pose = self._subject.pose # thread-safe access to subject pose
        if abs(subject_pose.x - msg.x) > 0.1 or abs(subject_pose.y - msg.y) > 0.1 or self._force_update_pose: # force, when goal gets aborted
            self._force_update_pose = False # Reset flag after handling

            self._logger.debug(f'Updating position of face {self._subject.id} to ({msg.x:.2f}, {msg.y:.2f}) based on new detection')

            self._subject.pose = Pose(msg.x, msg.y, None) # ! can block
            self._start_update_nav_goal(self._subject) # Start new goal with updated position

    def on_ring_detect(self, msg: RingDetect):
        if not self._controller.is_active(self): # Cant fire if not current, but still...
            self._logger.debug(f'Ring detected message received but state is {self.name}, ignoring')
            return
        if isinstance(self._subject, Face):
            self._logger.debug(f'Ring detected message received but current subject is face, ignoring')
            return

        # Silly asserts
        assert isinstance(self._subject, Ring), 'Cannot run on_ring_detect if current subject is not a ring'
        assert self._subject is not None, 'Ring detected message received but self._subject is None'

        # Update subject position if it had shifted, restart goal
        subject_pose = self._subject.pose # thread-safe access to subject pose
        if abs(subject_pose.x - msg.x) > 0.1 or abs(subject_pose.y - msg.y) > 0.1 or self._force_update_pose: # force, when goal gets aborted
            self._logger.debug(f'Updating position of ring {self._subject.id} to ({msg.x:.2f}, {msg.y:.2f}) based on new detection')

            self._force_update_pose = False # Reset flag after handling

            self._subject.pose = Pose(msg.x, msg.y, None) # ! can block
            self._start_update_nav_goal(self._subject) # Start new goal with updated position

        # If color changed, update it
        if self._subject.color != Ring.Color(msg.color):
            self._logger.debug(f'Updating color of ring {self._subject.id} to {msg.color} based on new detection')
            self._subject.color = Ring.Color(msg.color) # type: ignore
