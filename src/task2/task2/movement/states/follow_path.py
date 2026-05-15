import threading
from typing import TYPE_CHECKING

from task1.movement.models import Face, Pose, Ring
from msg_types.msg import FaceDetect, RingDetect
from task1.movement.states.base import Params, State

if TYPE_CHECKING:
    from task1.movement.controller import MovementController
    from task1.movement.movement import MovGoal

# Deferred import to avoid circular import at module load:
#   follow_path -> go_to_subject -> (nothing problematic, but keep pattern consistent)
# Import GoToSubject inside on_face_detect / on_ring_detect methods.


class FollowPathParams(Params):
    def __init__(self):
        super().__init__()

class FollowPath(State[FollowPathParams]):
    def __init__(self, controller: 'MovementController'):
        super().__init__(controller)

        # Path to follow
        default_path = [
            Pose(-0.15, 0.15,  1.5 ),
            Pose(-1.00, 0.20,  1.74),
            Pose(-1.35, 1.06, -1.45),
            Pose(-2.00, 1.08, -0.00),
            Pose(-2.10, 0.20,  2.40),
            Pose(-3.50, 1.00, -0.30),
            Pose(-3.50, 0.10,  1.20),
            Pose(-4.50, 0.13,  0.00),
        ]

        self._path: list[Pose] = default_path
        self._next_pose_idx: int = 0 # Index of last visited waypoint in path
        self._ros_goal_idx_offset = 0 # Offset to convert from path waypoint to _next_pose_idx

        self._goal: MovGoal | None = None # Current active goal, if any
        self._goal_lock = threading.RLock() # Lock for accessing current goal, to avoid race conditions

    def on_enter(self, params: FollowPathParams):
        # Set some variables
        self._goal = None
        self._ros_goal_idx_offset = self._next_pose_idx # Where we left off

        self._logger.info(f'Entering FollowPath state')

        path = self._path[self._next_pose_idx:]

        # Fedback called during navigation
        def feedback_callback(next_pose_idx: int):
            if not self._controller.is_active(self): # Check if still current state
                self._logger.debug(f'Feedback received for FollowPath, but isnt the current state, ignoring.')
                return
            
            if self._next_pose_idx != next_pose_idx + self._ros_goal_idx_offset:
                self._logger.info(f'Navigating around map: Visited pose {self._next_pose_idx}, going to pose {next_pose_idx + self._ros_goal_idx_offset}/{len(self._path)}')
                self._next_pose_idx = next_pose_idx + self._ros_goal_idx_offset # what ros thinks + what we already visited

        # Result called when navigation is complete
        def result_callback(result: 'MovGoal.Result'):
            from task1.movement.movement import MovGoal

            if result == MovGoal.Result.SUCCEEDED: 
                self._logger.info('FollowPath navigation complete!')
                self._on_complete()
            elif result == MovGoal.Result.CANCELED:
                self._logger.info('FollowPath navigation canceled, likely due to state change.')
            elif result == MovGoal.Result.ABORTED:
                self._logger.warning('FollowPath navigation aborted.')
                if self._controller.is_active(self):
                    self._logger.info('FollowPath still active, restarting navigation after abort.')
                    self.on_enter(params) # Restart navigation
            else:
                raise RuntimeError(f'FollowPath navigation ended with unknown result: {result}')
        
        # Run navigation
        with self._goal_lock:
            self._goal = self._controller.follow_waypoints(path, feedback_callback, result_callback, caller_name='FollowPath')

    def _on_complete(self):
        self._logger.info('FollowPath complete, shutting off altho not all faces/rings found.')
        self._controller.speak('I cant find any more faces or rings! My dissapointment is immeasurable and my day is ruined! Goodbye!')
        self._controller.shutdown()
    
    def on_face_detect(self, msg: FaceDetect):
        from task1.movement.states.go_to_subject import GoToSubject, GoToSubjectParams
        if not self._controller.is_active(self): # Shouldnt fire but still...
            self._logger.debug(f'Face detected message received but state is {self.name}, ignoring')
            return

        self._logger.info(f'Face detected with id {msg.id}, switching to GoToSubject state')

        # End goal 
        with self._goal_lock:
            if self._goal is not None:
                self._goal.cancel()
                self._goal = None
        
        # Change state
        params = GoToSubjectParams(Face(id=msg.id, pose=Pose(msg.x, msg.y, 0.0)))
        self._controller.change_state(new_state=GoToSubject, old_state=self.__class__, params=params)

    def on_ring_detect(self, msg: RingDetect):
        from task1.movement.states.go_to_subject import GoToSubject, GoToSubjectParams
        if not self._controller.is_active(self): # Shouldnt fire but still...
            self._logger.debug(f'Ring detected message received but state is {self.name}, ignoring')
            return

        self._logger.info(f'Ring detected with id {msg.id}, switching to GoToSubject state')

        # End goal
        with self._goal_lock:
            if self._goal is not None:
                self._goal.cancel()
                self._goal = None
        
        # Change state
        params = GoToSubjectParams(Ring(id=msg.id, pose=Pose(msg.x, msg.y, 0.0), color=Ring.Color(msg.color)))
        self._controller.change_state(new_state=GoToSubject, old_state=self.__class__, params=params)
