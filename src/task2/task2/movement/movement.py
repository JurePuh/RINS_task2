import math
import threading
import time
from collections import deque
from collections.abc import Callable
from enum import Enum
from typing import TYPE_CHECKING

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import (
    DriveOnHeading,
    FollowWaypoints,
    NavigateThroughPoses,
    NavigateToPose,
    Spin,
)

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

from task1.movement.models import Pose

if TYPE_CHECKING:
    from controller import MovementController

class MovGoal:

    class Result(Enum):
        UNKNOWN = 0
        SUCCEEDED = 1
        CANCELED = 2
        ABORTED = 3
    
    # Goal ids
    _id_counter = 0
    _id_lock = threading.Lock()

    def __init__(self, on_end_func: Callable[['MovGoal'], None], logger, caller_name: str = 'unknown_caller'):
        self._logger = logger
        self._on_end_func = on_end_func
        self.caller_name = caller_name

        with MovGoal._id_lock:
            self.id = MovGoal._id_counter
            MovGoal._id_counter += 1
        
        # Set when goal is accepted
        self._goal_handle_recieved_event = threading.Event()
        self._goal_handle: ClientGoalHandle | None = None

        # Set when result is received
        self._result_received_event = threading.Event()
        self._result_lock = threading.RLock()
        self.result: MovGoal.Result | None = None

    @property
    def has_ended(self) -> bool:
        with self._result_lock:
            return self.result is not None

    def _assign_goal_handle(self, goal_handle):
        self._goal_handle = goal_handle
        self._goal_handle_recieved_event.set() # Mark as cancelable
    
    def _end_goal(self, result: Result):
        with self._result_lock:
            if self.result is not None and self.result != result:
                raise AssertionError(f'Goal {self.id} ending with result {result} but already has result {self.result}?')
            if self.result == result:
                self._logger.debug(f'Goal {self.id} ({self.caller_name}) already ended with result {result}, ignoring duplicate end call')
                return

            self.result = result
        
        self._result_received_event.set() # Mark as completed
        self._on_end_func(self)

    def cancel(self):
        self._goal_handle_recieved_event.wait() # Wait for goal handle to be received
        assert self._goal_handle is not None, f'Goal {self.id} ({self.caller_name}) cancel called but goal handle is None'

        self._logger.debug(f'Canceling goal {self.id} ({self.caller_name}) through MovGoal.cancel()')

        # Dont cancel cancelled goal
        if self.has_ended:
            self._logger.debug(f'Goal {self.id} ({self.caller_name}) already has result {self.result}, cannot cancel, ignoring MogGoal.cancel() call')
            return
        
        def cancel_callback(future):
            cancel_response: CancelGoal.Response = future.result()
            if cancel_response.return_code == CancelGoal.Response.ERROR_NONE:
                self._logger.debug(f'Goal {self.id} ({self.caller_name}) cancelation request submitted successfully.')
                # Goal gets removed from queue when cancel response is received
            elif cancel_response.return_code == CancelGoal.Response.ERROR_REJECTED:
                self._logger.warning(f'Goal {self.id} ({self.caller_name}) cancelation request rejected, retrying...')
                self.cancel() # Retry cancelation
            elif cancel_response.return_code == CancelGoal.Response.ERROR_GOAL_TERMINATED:
                self._logger.warning(f'Goal {self.id} ({self.caller_name}) cancelation request failed, due to goal being terminated. Skipping.')
            else:
                raise RuntimeError(f'Got unknown error response to cancelation request for goal {self.id} ({self.caller_name}): {cancel_response.return_code}')
        
        cancel_future: Future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(cancel_callback)

    def wait_for_result(self, timeout: float | None = None) -> Result | None:
        if self._result_received_event.wait(timeout):
            with self._result_lock:
                assert self.result is not None, f'Goal {self.id} ({self.caller_name}) wait_for_result unblocked but result is None'
                return self.result
        else:
            self._logger.warning(f'Goal {self.id} ({self.caller_name}) wait_for_result timed out after {timeout} seconds, skipping waiting.')
            return None

class MovGoalQueue:
    def __init__(self, logger):
        self._logger = logger

        # Goal queue management
        self._queue_lock = threading.RLock()
        self._goal_queue: deque[MovGoal] = deque() # ordered goal objects, [0] is active goal, others are waiting in queue
        self._goal_events: dict[MovGoal, threading.Event] = {} # id -> event that fires when it's this goal's turn
    
    @property
    def _active_goal(self) -> MovGoal | None:
        with self._queue_lock:
            return self._goal_queue[0] if self._goal_queue else None

    def get_goal(self, caller_name = 'unknown_caller'):
        return MovGoal(self._on_goal_end, self._logger, caller_name=caller_name)

    def _on_goal_end(self, goal: MovGoal):
        with self._queue_lock:
            assert self._active_goal == goal, f'Goal {goal.id} ({goal.caller_name}) ended but it is not the active goal in the queue'
            
            # Clean up finished goal
            self._goal_queue.remove(goal) # Remove from queue
            self._goal_events.pop(goal) # Remove event

            # Activate next goal in queue if any
            if self._goal_queue:
                next = self._goal_queue[0]
                self._goal_events[next].set() # Unblock waiting thread
                self._logger.debug(f'Goal {goal.id} ({goal.caller_name}) finished, activating next goal {next.id} ({next.caller_name}) in queue')
            else:
                self._logger.debug(f'Goal {goal.id} ({goal.caller_name}) finished, queue empty')

    def wait_for_queue(self, goal: 'MovGoal'):
        ready_event = threading.Event()

        with self._queue_lock:
            self._goal_events[goal] = ready_event
            self._goal_queue.append(goal)

            if self._active_goal == goal: # No active goal, set this one
                self._logger.debug(f'Goal {goal.id} ({goal.caller_name}) has no active goal ahead, activating immediately')
                ready_event.set()
            else:
                self._logger.debug(f'Goal {self._active_goal.id} ({self._active_goal.caller_name}) currently active, {goal.id} ({goal.caller_name}) waiting for turn...') # type: ignore
        
        ready_event.wait()  # Blocks until it's this goal's turn
        time.sleep(0.3) # !!!
        self._logger.debug(f'Goal {goal.id} ({goal.caller_name}) is now active, proceeding...')

class Movement:

    def __init__(self, controller: 'MovementController'):
        self._controller = controller
        self._logger = controller.get_logger()

        # Goal queue management wrapper
        self._goal_queue = MovGoalQueue(self._logger)

        # Create action clients
        self._action_cb_group = ReentrantCallbackGroup()
        # Create action client for spinning
        self._spin_client = ActionClient(
            self._controller, Spin, '/spin',
            callback_group=self._action_cb_group
        )
        self._spin_client.wait_for_server() # Wait for server to be available
        # Create action client for navigating to a single pose
        self._to_pose_client = ActionClient(
            self._controller, NavigateToPose, '/navigate_to_pose',
            callback_group=self._action_cb_group
        )
        self._to_pose_client.wait_for_server()
        # Create action client for navigating through poses
        self._through_client = ActionClient(
            self._controller, NavigateThroughPoses, '/navigate_through_poses',
            callback_group=self._action_cb_group
        )
        self._through_client.wait_for_server() # Wait for server to be available
        # Create action client for following waypoints
        self._waypoints_client = ActionClient(
            self._controller, FollowWaypoints, '/follow_waypoints',
            callback_group=self._action_cb_group
        )
        self._waypoints_client.wait_for_server()
        # Create action client for driving on heading
        self._drive_on_heading_client = ActionClient(
            self._controller, DriveOnHeading, '/drive_on_heading',
            callback_group=self._action_cb_group
        )
        self._drive_on_heading_client.wait_for_server()

    def _translate_coordinate_to_pose(self, pose: Pose) -> PoseStamped:
        assert pose.theta is not None, 'Pose theta cannot be None when translating to PoseStamped'

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self._controller.get_clock().now().to_msg()
        pose_stamped.pose.position.x = pose.x
        pose_stamped.pose.position.y = pose.y
        pose_stamped.pose.position.z = 0.0 # ground robot
        # Convert theta to quaternion
        qz = math.sin(pose.theta / 2.0)
        qw = math.cos(pose.theta / 2.0)
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw

        return pose_stamped
    
    # Called on goal accept, same for all movement types!
    def _shared_goal_response_callback(self, future, goal: MovGoal, result_callback: Callable[[MovGoal.Result], None]):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._logger.warning(f'Goal {goal.id} ({goal.caller_name}) rejected by server, aborting goal...')
            goal._end_goal(MovGoal.Result.ABORTED)
            result_callback(MovGoal.Result.ABORTED)
            return

        
        self._logger.debug(f'Goal {goal.id} ({goal.caller_name}) accepted by server, robot moving...')

        goal._assign_goal_handle(goal_handle) # Make goal cancelable now that we have a goal handle

        def result_wrapper(future):
            res = future.result()
            status = res.status
            nonlocal goal

            if status == GoalStatus.STATUS_SUCCEEDED:
                self._logger.debug(f'Goal {goal.id} ({goal.caller_name}) ended with SUCCEEDED status.')
                goal._end_goal(MovGoal.Result.SUCCEEDED)
                result_callback(MovGoal.Result.SUCCEEDED)
            elif status == GoalStatus.STATUS_CANCELED:
                self._logger.debug(f'Goal {goal.id} ({goal.caller_name}) ended with CANCELED status.')
                goal._end_goal(MovGoal.Result.CANCELED) # !!!
                result_callback(MovGoal.Result.CANCELED)
            elif status == GoalStatus.STATUS_ABORTED:
                self._logger.warning(f'Goal {goal.id} ({goal.caller_name}) ended with ABORTED status.')
                goal._end_goal(MovGoal.Result.ABORTED)
                result_callback(MovGoal.Result.ABORTED)
            else:
                raise RuntimeError(f'Goal {goal.id} ({goal.caller_name}) ended with unknown status: {status}')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(result_wrapper)

    def spin(self,
        angle: float,
        feedback_callback: Callable[[float], None],
        result_callback: Callable[[MovGoal.Result], None],
        caller_name = 'unknown_caller',
    ) -> MovGoal:
        """Spin in place by a given angle
            Args:
                angle: Target yaw in radians (positive = counter-clockwise, negative = clockwise)
                feedback_callback: (angular_distance_traveled: float)
                result_callback: (status - GoalResult)
            Returns:
                MovGoal object you can cancel, wait on..."""
        # Create goal
        goal = self._goal_queue.get_goal(caller_name=caller_name)
        self._goal_queue.wait_for_queue(goal) # Wait for turn in queue

        # Create nav2 goal message
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = angle

        # Callback wrapper for goal feedback
        def feedback_wrapper(feedback_msg):
            fb: Spin.Feedback = feedback_msg.feedback
            feedback_callback(fb.angular_distance_traveled)

        # Send nav2 goal
        self._logger.debug(f"Sending {goal.id} ({goal.caller_name}) to 'spin' ({math.degrees(angle):.1f}°)")
        send_future = self._spin_client.send_goal_async(
            goal_msg, feedback_callback=feedback_wrapper
        )
        send_future.add_done_callback(lambda future: self._shared_goal_response_callback(future, goal, result_callback))

        return goal

    def navigate_to_pose(self, 
        pose: Pose, 
        feedback_callback: Callable[[float, Duration, PoseStamped], None], 
        result_callback: Callable[[MovGoal.Result], None],
        caller_name = 'unknown_caller',
    ) -> MovGoal:
        """Navigate to a single pose
            Args:
                pose: Pose to navigate to
                feedback_callback: (distance remaining: float, estimated time remaining: Duration, current pose: PoseStamped)
                result_callback: (status - GoalResult)
            Returns:
                MovGoal object you can cancel, wait on..."""
        pose_stamped = self._translate_coordinate_to_pose(pose)

        # Create goal
        goal = self._goal_queue.get_goal(caller_name=caller_name)
        self._goal_queue.wait_for_queue(goal) # Wait for turn in queue

        # Create nav2 goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self._logger.debug(f"Goal {goal.id} ({goal.caller_name}) sent to 'navigate to pose' ({pose.x:.2f}, {pose.y:.2f}, {pose.theta:.2f})")

        def feedback_wrapper(feedback_msg):
            fb: NavigateToPose.Feedback = feedback_msg.feedback
            feedback_callback(fb.distance_remaining, fb.estimated_time_remaining, fb.current_pose)

        # Send goal
        send_future = self._to_pose_client.send_goal_async(
            goal_msg, feedback_callback=feedback_wrapper
        )
        send_future.add_done_callback(lambda future: self._shared_goal_response_callback(future, goal, result_callback))

        return goal

    def navigate_through_poses(self, 
        path: list[Pose], 
        feedback_callback: Callable[[int, float, Duration, PoseStamped], None], 
        result_callback: Callable[[MovGoal.Result], None],
        caller_name = 'unknown_caller',
    ) -> MovGoal:
        """Navigate, dont stop at waypoints
            Args:
                path: List of Pose
                feedback_callback: (num poses remaining: int, distance remaining: float, estimated time remaining: Duration, current pose: PoseStamped)
                result_callback: (status - GoalResult)
            Returns:
                MovGoal object you can cancel, wait on..."""
        poses = [self._translate_coordinate_to_pose(pose) for pose in path]

        # Create goal
        goal = self._goal_queue.get_goal(caller_name=caller_name)
        self._goal_queue.wait_for_queue(goal) # Wait for turn in queue

        # Create nav2 goal message
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self._logger.debug(f"Goal {goal.id} ({goal.caller_name}) sent with {len(poses)} poses to 'navigate through poses'")

        # Wrapper for feedback, logging + extracting relevant info
        def feedback_wrapper(feedback_msg):
            fb: NavigateThroughPoses.Feedback = feedback_msg.feedback
            feedback_callback(fb.number_of_poses_remaining, fb.distance_remaining, fb.estimated_time_remaining, fb.current_pose)

        # Send goal
        send_future = self._through_client.send_goal_async(
            goal_msg, feedback_callback=feedback_wrapper
        )
        send_future.add_done_callback(lambda future: self._shared_goal_response_callback(future, goal, result_callback))

        return goal

    def follow_waypoints(self, 
        path: list[Pose], 
        feedback_callback: Callable[[int], None], 
        result_callback: Callable[[MovGoal.Result], None],
        caller_name: str = 'unknown_caller'
    ) -> MovGoal:
        """Follow waypoints, stopping at each one
            Args:
                path: List of Pose
                feedback_callback: (current waypoint index: int)
                result_callback: (status - GoalResult)
            Returns:
                MovGoal object you can cancel, wait on...
            """

        poses = [self._translate_coordinate_to_pose(pose) for pose in path]

        # Create goal
        goal = self._goal_queue.get_goal(caller_name=caller_name)
        self._goal_queue.wait_for_queue(goal) # Wait for turn in queue

        # Create nav2 goal message
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self._logger.debug(f"Goal {goal.id} ({goal.caller_name}) sent with {len(poses)} waypoints to 'follow waypoints'")

        # Wrapper for feedback - FollowWaypoints feedback only has current_waypoint index
        def feedback_wrapper(feedback_msg):
            fb: FollowWaypoints.Feedback = feedback_msg.feedback
            feedback_callback(fb.current_waypoint)

        # Send goal
        send_future = self._waypoints_client.send_goal_async(
            goal_msg, feedback_callback=feedback_wrapper
        )
        send_future.add_done_callback(lambda future: self._shared_goal_response_callback(future, goal, result_callback))

        return goal

    def drive_on_heading(self,
        distance: float,
        feedback_callback: Callable[[float], None] = lambda _: None,
        result_callback: Callable[['MovGoal.Result'], None] = lambda _: None,
        caller_name: str = 'unknown_caller',
    ) -> 'MovGoal':
        """Drive forward or backward in a straight line along the current heading.
            Args:
                distance: Distance in meters. Positive = forward, negative = backward.
                speed: Speed in m/s (always positive, direction is determined by distance sign).
                feedback_callback: (distance_traveled: float)
                result_callback: (status - GoalResult)
            Returns:
                MovGoal object you can cancel, wait on..."""
        # Create goal
        goal = self._goal_queue.get_goal(caller_name=caller_name)
        self._goal_queue.wait_for_queue(goal)

        speed = 0.2

        # Create nav2 goal message
        goal_msg = DriveOnHeading.Goal()
        goal_msg.target.x = distance
        goal_msg.speed = speed # m/s, TODO tune
        goal_msg.time_allowance = Duration(
            sec=int(abs(distance) / speed * 3),  # 3x expected time as timeout
            nanosec=0
        )

        self._logger.debug(
            f"Goal {goal.id} ({goal.caller_name}) sent to 'drive_on_heading' "
            f"({distance:.2f}m at {goal_msg.speed:.2f}m/s)"
        )

        def feedback_wrapper(feedback_msg):
            fb: DriveOnHeading.Feedback = feedback_msg.feedback
            feedback_callback(fb.distance_traveled)

        # Send goal
        send_future = self._drive_on_heading_client.send_goal_async(
            goal_msg, feedback_callback=feedback_wrapper
        )
        send_future.add_done_callback(
            lambda future: self._shared_goal_response_callback(future, goal, result_callback)
        )

        return goal
