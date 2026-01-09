from enum import Enum, auto
from typing import List
import logging

from .abstract_state_machine import AbstractStateMachine, Movement, MovementSequence

LOGGER = logging.getLogger(__name__)

class SimpleState(Enum):
    INIT = auto()
    RUNNING = auto()
    COMPLETE = auto()
    ERROR = auto()


class SimpleMovementStateMachine(AbstractStateMachine[SimpleState]):
    """Wrap a user-provided list of Movement objects in a minimal state machine
    to reuse AbstractStateMachine execution if needed.
    """

    def __init__(self, bot: 'InterbotixManipulatorXS', movements: List[Movement], default_wait_time: float = 1.0):
        self._provided_movements = movements
        super().__init__(bot, default_wait_time)

    def get_initial_state(self):
        return SimpleState.INIT

    def get_complete_state(self):
        return SimpleState.COMPLETE

    def get_error_state(self):
        return SimpleState.ERROR

    def _create_sequences(self):
        return {"user_sequence": MovementSequence("user_sequence", self._provided_movements)}

    def _process_current_state(self) -> bool:
        try:
            if self.state == SimpleState.INIT:
                self._transition_to_sequence("user_sequence", SimpleState.RUNNING)
            elif self.state == SimpleState.RUNNING:
                if not self._execute_current_sequence():
                    return False
                if self.current_sequence and self.current_sequence.is_complete():
                    self.state = SimpleState.COMPLETE
            elif self.state == SimpleState.ERROR:
                return False
            return True
        except Exception as e:
            self.state = SimpleState.ERROR
            LOGGER.info(f"Exception SimpleMovementStateMachine: {e}")
            return False
