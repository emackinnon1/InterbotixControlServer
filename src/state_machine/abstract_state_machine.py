#!/usr/bin/env python3
import time
from abc import ABC, abstractmethod
from enum import Enum, auto
from typing import Dict, List, Any, Optional, TypeVar, Generic
from dataclasses import dataclass
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup

StateType = TypeVar('StateType', bound=Enum)

class MovementType(Enum):
    GO_HOME = auto()
    JOINT_MOVE = auto()
    CARTESIAN_MOVE = auto()
    POSE_COMPONENTS = auto()
    GRIPPER_ACTION = auto()
    WAIT = auto()

@dataclass
class Movement:
    type: MovementType
    params: Dict[str, Any]
    description: str
    timeout: float = 10.0
    skip_default_wait: bool = False

class MovementSequence:
    def __init__(self, name: str, movements: List[Movement]):
        self.name = name
        self.movements = movements
        self.current_step = 0
        
    def get_current_movement(self) -> Optional[Movement]:
        if self.current_step < len(self.movements):
            return self.movements[self.current_step]
        return None
    
    def advance(self) -> bool:
        self.current_step += 1
        return self.current_step < len(self.movements)
    
    def is_complete(self) -> bool:
        return self.current_step >= len(self.movements)
    
    def reset(self):
        self.current_step = 0

class MovementExecutor:
    def __init__(self, bot: InterbotixManipulatorXS, default_wait_time: float = 2.0):
        self.bot = bot
        self.default_wait_time = default_wait_time
        
    def execute_movement(self, movement: Movement) -> bool:
        """Execute a single movement and return success status"""
        try:
            print(f"Executing: {movement.description}")
            
            if movement.type == MovementType.GO_HOME:
                self.bot.arm.go_to_home_pose(**movement.params)
                
            elif movement.type == MovementType.JOINT_MOVE:
                self.bot.arm.set_single_joint_position(**movement.params)
                
            elif movement.type == MovementType.CARTESIAN_MOVE:
                self.bot.arm.set_ee_cartesian_trajectory(**movement.params)
                
            elif movement.type == MovementType.POSE_COMPONENTS:
                self.bot.arm.set_ee_pose_components(**movement.params)
                
            elif movement.type == MovementType.GRIPPER_ACTION:
                action = movement.params.get('action')
                if action == 'grasp':
                    self.bot.gripper.grasp()
                elif action == 'release':
                    self.bot.gripper.release()
                    
            elif movement.type == MovementType.WAIT:
                time.sleep(movement.params.get('duration', 1.0))
                return True  # Return early for WAIT movements to avoid double waiting
            
            # TODO: may need to revisit the wait logic to reduce total time it takes to run through process.
            # Add automatic wait only if not skipped and not a WAIT movement
            if not movement.skip_default_wait and movement.type != MovementType.WAIT:
                time.sleep(self.default_wait_time)
            return True
            
        except Exception as e:
            print(f"Movement failed: {e}")
            return False

class AbstractStateMachine(Generic[StateType], ABC):
    def __init__(self, bot: InterbotixManipulatorXS, default_wait_time: float = 2.0):
        self.bot = bot
        self.executor = MovementExecutor(bot, default_wait_time)
        self.current_sequence: Optional[MovementSequence] = None
        self.state = self.get_initial_state()
        
        # Initialize sequences
        self.sequences = self._create_sequences()
    
    @abstractmethod
    def get_initial_state(self) -> StateType:
        """Return the initial state for this state machine"""
        pass
    
    @abstractmethod
    def get_complete_state(self) -> StateType:
        """Return the complete state for this state machine"""
        pass
    
    @abstractmethod
    def get_error_state(self) -> StateType:
        """Return the error state for this state machine"""
        pass
    
    @abstractmethod
    def _create_sequences(self) -> Dict[str, MovementSequence]:
        """Create and return all movement sequences for this state machine"""
        pass
    
    @abstractmethod
    def _process_current_state(self) -> bool:
        """Process the current state and advance if needed"""
        pass
    
    def run(self) -> bool:
        """Run the complete state machine"""
        while (self.state != self.get_complete_state() and 
               self.state != self.get_error_state()):
            if not self._process_current_state():
                self.state = self.get_error_state()
                return False
                
        return self.state == self.get_complete_state()
    
    def run_with_robot_management(self) -> bool:
        """Run the complete state machine with proper robot initialization and cleanup"""
        try:
            # Initialize robot
            # robot_startup()
            self.bot.core.robot_torque_enable(cmd_type='group', name='all', enable=True)
            
            # Run the state machine
            success = self.run()
            
            if success:
                print(f"{self.__class__.__name__} completed successfully!")
            else:
                print(f"{self.__class__.__name__} failed!")
            
            # Return to safe positions
            self.bot.arm.go_to_home_pose()
            self.bot.arm.go_to_sleep_pose()
            time.sleep(0.5)
            
            return success
            
        except Exception as e:
            print(f"Error during {self.__class__.__name__} execution: {e}")
            return False
            
        finally:
            # Always reboot motors, disable torque and shutdown
            try:
                self.bot.core.robot_reboot_motors(cmd_type='group', name='all', enable=False, smart_reboot=True)
                self.bot.core.robot_torque_enable(cmd_type='group', name='all', enable=False)
            except Exception as e:
                print(f"Error during robot shutdown: {e}")
    
    def _execute_current_sequence(self) -> bool:
        """Execute the current movement in the current sequence"""
        if not self.current_sequence:
            return False
            
        movement = self.current_sequence.get_current_movement()
        if not movement:
            return True
            
        success = self.executor.execute_movement(movement)
        if success:
            self.current_sequence.advance()
            
        return success
    
    def _transition_to_sequence(self, sequence_name: str, next_state: StateType):
        """Helper method to transition to a new sequence and state"""
        if sequence_name in self.sequences:
            self.current_sequence = self.sequences[sequence_name]
            self.current_sequence.reset()
            self.state = next_state
        else:
            raise ValueError(f"Unknown sequence: {sequence_name}")