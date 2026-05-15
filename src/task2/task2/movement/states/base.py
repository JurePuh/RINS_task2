from typing import TYPE_CHECKING, Generic, TypeVar

from msg_types.msg import FaceDetect, RingDetect

if TYPE_CHECKING:
    from task1.movement.controller import MovementController


# States
class Params:
    def __init__(self):
        pass

TParams = TypeVar('TParams', bound=Params)

class State(Generic[TParams]):
    def __init__(self, controller: 'MovementController'):
        self.name = self.__class__.__name__
        self._controller: MovementController = controller
        self._logger = self._controller.get_logger()

    def on_enter(self, params: TParams):
        raise NotImplementedError

    def on_face_detect(self, msg: FaceDetect):
        self._logger.debug(f'Ignoring face detected message, state is {self.name}')
    
    def on_ring_detect(self, msg: RingDetect):
        self._logger.debug(f'Ignoring ring detected message, state is {self.name}')

