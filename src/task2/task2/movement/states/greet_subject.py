import math
import time
from typing import TYPE_CHECKING

from task1.movement.models import Face, Ring, Subject
from task1.movement.states.base import Params, State

if TYPE_CHECKING:
    from task1.movement.controller import MovementController
    from task1.movement.movement import MovGoal


class GreetSubjectParams(Params):
    def __init__(self, subject: Subject):
        super().__init__()
        self.subject = subject

class GreetSubject(State[GreetSubjectParams]):
    def __init__(self, controller: 'MovementController'):
        super().__init__(controller)

        # Counting num of greeted faces/rings.
        self._num_greeted = {
            Face: 0,
            Ring: 0,
        }

        self._names = [
            'Allice', 'Bob', 'Alehandro', 'Charlie', 'Diana', 'Eve', 'Frank', 'Grace', 'Heidi', 'Ivan',
        ]

        self._numbers = [
            'first', 'second', 'third', 'fourth', 'fifth', 'sixth', 'seventh', 'eighth', 'ninth', 'tenth',
        ]

    def on_enter(self, params: GreetSubjectParams):
        # Reset some variables
        subject = params.subject

        self._logger.info(f'Entering GreetSubject state, dancing for {subject.type_str} {subject.id}')

        self._clear_subject(subject)
        self._num_greeted[type(subject)] += 1 # Check if last in back_to_path

        # self._dance() do not dance

        if isinstance(subject, Face):
            self._controller.speak(f'Hello {self._names[self._num_greeted[Face] - 1]}! You are the {self._numbers[self._num_greeted[Face] - 1]} face I have greeted!')
        elif isinstance(subject, Ring):
            self._controller.speak(f'Hello {subject.color.value} ring! You are the {self._numbers[self._num_greeted[Ring] - 1]} ring I have greeted!')

        time.sleep(2)

        self._continue() # Continue as we dont dance!!! (remove if dancing)

    def _clear_subject(self, subject: Subject):
        self._logger.debug(f'Clearing {subject.type_str} {subject.id}.')

        if isinstance(subject, Face):
            self._controller.clear_face(subject)
        elif isinstance(subject, Ring):
            self._controller.clear_ring(subject)
        else:
            raise RuntimeError(f'Unknown subject type: {subject.type_str}')

    def _dance(self):
        spin_move_iter = iter([
            lambda: self._controller.spin(math.radians(15), spin_feedback_callback, result_callback, caller_name='GreetSubject_spin1'),
            lambda: self._controller.spin(math.radians(-30), spin_feedback_callback, result_callback, caller_name='GreetSubject_spin2'),
            lambda: self._controller.spin(math.radians(30), spin_feedback_callback, result_callback, caller_name='GreetSubject_spin3'),
            lambda: self._controller.spin(math.radians(-15), spin_feedback_callback, result_callback, caller_name='GreetSubject_spin4'),
        ])

        def spin_feedback_callback(angle_traveled: float):
            if not self._controller.is_active(self): # Check if still current state
                self._logger.debug(f'Feedback received for GreetSubject, but isnt current state, ignoring.')
                return
            
            if (time.time() % 1) < 0.04: # Log every second
                self._logger.debug(f'Dancing, angle traveled: {math.degrees(angle_traveled):.1f} rad')
        
        def result_callback(result: 'MovGoal.Result'):
            from task1.movement.movement import MovGoal
            
            if result == MovGoal.Result.SUCCEEDED:
                next_move = next(spin_move_iter, None)
                if next_move is not None:
                    self._logger.debug(f'GreetSubject spin move succeeded! Starting next move.')
                    next_move()
                else:
                    self._logger.info('GreetSubject dance complete!')
                    self._continue()
            elif result == MovGoal.Result.CANCELED:
                self._logger.warning(f'GreetSubject spin move canceled somehow, skipping dancing')
                self._continue()
            elif result == MovGoal.Result.ABORTED:
                self._logger.warning(f'GreetSubject spin move aborted by server, skipping dancing.')
                self._continue()
            else:
                raise RuntimeError(f'GreetSubject spin move ended with unknown result: {result}')

        self._logger.info(f'Starting dance for subject!')

        next(spin_move_iter)() # Start first move

    def _continue(self):
        from task1.movement.states.follow_path import FollowPath, FollowPathParams
        if self._num_greeted[Face] >= 3 and self._num_greeted[Ring] >= 2:
            self._logger.info('Greeted enough faces and rings, shutting down')
            time.sleep(2) # Wait for last speak to finish
            self._controller.speak('I have greeted enough faces and rings, goodbye!')
            self._controller.spin(math.radians(720), lambda x: None, lambda result: None, caller_name='FinalSpin') # Do a final spin for fun
            self._controller.shutdown()
            return
        
        # Back to path
        self._logger.info('GreetSubject complete, going back to path, more subjects to come!')
        self._controller.change_state(new_state=FollowPath, old_state=self.__class__, params=FollowPathParams())
