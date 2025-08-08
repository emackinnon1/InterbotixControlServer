#!/usr/bin/env python3
import time
from enum import Enum, auto
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np

# Constants
WAIST_PICKUP_POSITION = -np.pi/1.89
WAIST_BOTTLE_POSITION = -np.pi/5.65
WRIST_ROTATE_GRASP = np.pi/2.11
WRIST_ROTATE_INITIAL = np.pi/3.5
WRIST_ROTATE_OPEN = np.pi/1.2
LOWER_DISTANCE = -0.15
REVERSE_DISTANCE = -0.193
GRIPPER_LOWER_DISTANCE = -0.16
RAISE_DISTANCE = 0.19
BOTTLE_LOWER_DISTANCE = -0.2
BOTTLE_RAISE_DISTANCE = 0.1

# Brand configurations
BRAND_CONFIGS = {
    "sapporo": {
        "waist_rotation": -np.pi/5
    },
    "heineken": {
        "waist_rotation": -np.pi/4.75
    }
}

class BeerOpenerState(Enum):
    INIT = auto()
    PICKUP_OPENER = auto()
    APPROACH_BOTTLE = auto()
    OPEN_BOTTLE = auto()
    RETURN_OPENER = auto()
    COMPLETE = auto()
    ERROR = auto()

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
            
            # Add automatic wait after all non-WAIT movements
            time.sleep(self.default_wait_time)
            return True
            
        except Exception as e:
            print(f"Movement failed: {e}")
            return False

class BeerOpenerStateMachine:
    def __init__(self, bot: InterbotixManipulatorXS, brand: str, default_wait_time: float = 2.0):
        self.bot = bot
        self.brand = brand
        self.state = BeerOpenerState.INIT
        self.executor = MovementExecutor(bot, default_wait_time)
        self.current_sequence: Optional[MovementSequence] = None
        
        # Define movement sequences
        self.sequences = self._create_sequences()
        
    def _create_sequences(self) -> Dict[str, MovementSequence]:
        pickup_opener_sequence = MovementSequence("pickup_opener", [
            Movement(MovementType.GRIPPER_ACTION, {'action': 'release'}, "Release gripper"),
            Movement(MovementType.GO_HOME, {'moving_time': 1.75}, "Go to home pose"),
            Movement(MovementType.JOINT_MOVE, {'joint_name': 'waist', 'position': WAIST_PICKUP_POSITION}, "Rotate to pickup position"),
            Movement(MovementType.CARTESIAN_MOVE, {'z': LOWER_DISTANCE, 'x': REVERSE_DISTANCE}, "Lower and move to opener"),
            Movement(MovementType.WAIT, {'duration': 2.0}, "Wait for stabilization"),
            Movement(MovementType.CARTESIAN_MOVE, {'x': REVERSE_DISTANCE}, "Align with opener"),
            Movement(MovementType.WAIT, {'duration': 2.0}, "Wait for alignment"),
            Movement(MovementType.CARTESIAN_MOVE, {'pitch': 1.5}, "Adjust wrist pitch"),
            Movement(MovementType.WAIT, {'duration': 1.5}, "Wait for pitch adjustment"),
            Movement(MovementType.JOINT_MOVE, {'joint_name': 'wrist_rotate', 'position': WRIST_ROTATE_GRASP}, "Rotate wrist for grasp"),
            Movement(MovementType.WAIT, {'duration': 1.0}, "Wait for wrist rotation"),
            Movement(MovementType.CARTESIAN_MOVE, {'z': GRIPPER_LOWER_DISTANCE}, "Lower to grasp opener"),
            Movement(MovementType.WAIT, {'duration': 2.0}, "Wait before grasping"),
            Movement(MovementType.GRIPPER_ACTION, {'action': 'grasp'}, "Grasp bottle opener"),
            Movement(MovementType.CARTESIAN_MOVE, {'z': RAISE_DISTANCE}, "Raise with opener"),
            Movement(MovementType.WAIT, {'duration': 1.5}, "Wait after pickup")
        ])
        
        approach_bottle_sequence = MovementSequence("approach_bottle", [
            Movement(MovementType.GO_HOME, {}, "Go to home pose"),
            Movement(MovementType.POSE_COMPONENTS, {'x': 0.21, 'z': 0.35}, "Move to bottle approach position"),
            Movement(MovementType.WAIT, {'duration': 1.0}, "Wait at approach position"),
            Movement(MovementType.JOINT_MOVE, {'joint_name': 'wrist_rotate', 'position': WRIST_ROTATE_INITIAL}, "Rotate wrist for bottle"),
            Movement(MovementType.WAIT, {'duration': 1.0}, "Wait for wrist rotation"),
            Movement(MovementType.JOINT_MOVE, {'joint_name': 'waist', 'position': WAIST_BOTTLE_POSITION}, "Rotate waist to bottle"),
            Movement(MovementType.WAIT, {'duration': 1.0}, "Wait for waist rotation"),
            Movement(MovementType.CARTESIAN_MOVE, {'z': BOTTLE_LOWER_DISTANCE}, "Lower to bottle level"),
            Movement(MovementType.WAIT, {'duration': 2.0}, "Wait at bottle level")
        ])
        
        return_opener_sequence = MovementSequence("return_opener", [
            Movement(MovementType.GO_HOME, {'moving_time': 1.75}, "Go to home pose"),
            Movement(MovementType.JOINT_MOVE, {'joint_name': 'waist', 'position': WAIST_PICKUP_POSITION}, "Rotate to return position"),
            Movement(MovementType.CARTESIAN_MOVE, {'z': LOWER_DISTANCE, 'x': REVERSE_DISTANCE}, "Lower to return opener"),
            Movement(MovementType.WAIT, {'duration': 2.0}, "Wait for positioning"),
            Movement(MovementType.CARTESIAN_MOVE, {'x': REVERSE_DISTANCE}, "Align for return"),
            Movement(MovementType.WAIT, {'duration': 2.0}, "Wait for alignment"),
            Movement(MovementType.CARTESIAN_MOVE, {'pitch': 1.5}, "Adjust pitch for return"),
            Movement(MovementType.WAIT, {'duration': 1.5}, "Wait for pitch"),
            Movement(MovementType.JOINT_MOVE, {'joint_name': 'wrist_rotate', 'position': WRIST_ROTATE_GRASP}, "Rotate wrist for return"),
            Movement(MovementType.WAIT, {'duration': 1.0}, "Wait for rotation"),
            Movement(MovementType.CARTESIAN_MOVE, {'z': GRIPPER_LOWER_DISTANCE}, "Lower to return position"),
            Movement(MovementType.WAIT, {'duration': 2.0}, "Wait before releasing"),
            Movement(MovementType.GRIPPER_ACTION, {'action': 'release'}, "Release bottle opener"),
            Movement(MovementType.CARTESIAN_MOVE, {'z': RAISE_DISTANCE}, "Raise after return"),
            Movement(MovementType.WAIT, {'duration': 1.5}, "Wait after return")
        ])
        
        return {
            "pickup_opener": pickup_opener_sequence,
            "approach_bottle": approach_bottle_sequence,
            "return_opener": return_opener_sequence
        }
    
    def _create_open_bottle_sequence(self) -> MovementSequence:
        """Create brand-specific bottle opening sequence"""
        waist_rotation = BRAND_CONFIGS.get(self.brand, {}).get("waist_rotation")
        if not waist_rotation:
            raise ValueError(f"Unknown brand: {self.brand}")
            
        return MovementSequence("open_bottle", [
            Movement(MovementType.JOINT_MOVE, {'joint_name': 'waist', 'position': waist_rotation}, "Position opener on bottle cap"),
            Movement(MovementType.WAIT, {'duration': 1.0}, "Wait for positioning"),
            Movement(MovementType.JOINT_MOVE, {'joint_name': 'wrist_rotate', 'position': WRIST_ROTATE_OPEN, 'moving_time': 0.5}, "Rotate wrist to open"),
            Movement(MovementType.CARTESIAN_MOVE, {'z': BOTTLE_RAISE_DISTANCE}, "Raise while opening"),
            Movement(MovementType.JOINT_MOVE, {'joint_name': 'waist', 'position': WAIST_BOTTLE_POSITION, 'moving_time': 1.5}, "Complete opening motion"),
            Movement(MovementType.WAIT, {'duration': 0.5}, "Wait for opening completion")
        ])
    
    def run(self) -> bool:
        """Run the complete beer opening state machine"""
        while self.state != BeerOpenerState.COMPLETE and self.state != BeerOpenerState.ERROR:
            if not self._process_current_state():
                self.state = BeerOpenerState.ERROR
                return False
                
        return self.state == BeerOpenerState.COMPLETE
    
    def _process_current_state(self) -> bool:
        """Process the current state and advance if needed"""
        if self.state == BeerOpenerState.INIT:
            print("Starting beer opener sequence")
            self.current_sequence = self.sequences["pickup_opener"]
            self.current_sequence.reset()
            self.state = BeerOpenerState.PICKUP_OPENER
            
        elif self.state == BeerOpenerState.PICKUP_OPENER:
            if not self._execute_current_sequence():
                return False
            if self.current_sequence.is_complete():
                self.current_sequence = self.sequences["approach_bottle"]
                self.current_sequence.reset()
                self.state = BeerOpenerState.APPROACH_BOTTLE
                
        elif self.state == BeerOpenerState.APPROACH_BOTTLE:
            if not self._execute_current_sequence():
                return False
            if self.current_sequence.is_complete():
                self.current_sequence = self._create_open_bottle_sequence()
                self.current_sequence.reset()
                self.state = BeerOpenerState.OPEN_BOTTLE
                
        elif self.state == BeerOpenerState.OPEN_BOTTLE:
            if not self._execute_current_sequence():
                return False
            if self.current_sequence.is_complete():
                self.current_sequence = self.sequences["return_opener"]
                self.current_sequence.reset()
                self.state = BeerOpenerState.RETURN_OPENER
                
        elif self.state == BeerOpenerState.RETURN_OPENER:
            if not self._execute_current_sequence():
                return False
            if self.current_sequence.is_complete():
                self.state = BeerOpenerState.COMPLETE
                
        return True
    
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

def open_beer_state_machine(brand: str, wait_time: float = 2.0):
    """Main function to open a beer bottle"""
    bot = InterbotixManipulatorXS(
        robot_model='wx250',
        group_name='arm',
        gripper_name='gripper',
        moving_time=1.5,
        gripper_pressure=0.85           
    )

    try:
        robot_startup()
        bot.core.robot_torque_enable(cmd_type='group', name='all', enable=True)
        
        # Create and run state machine
        state_machine = BeerOpenerStateMachine(bot, brand, wait_time)
        success = state_machine.run()
        
        if success:
            print("Beer opening completed successfully!")
        else:
            print("Beer opening failed!")
            
        # Return to home and shutdown
        bot.arm.go_to_home_pose()
        bot.arm.go_to_sleep_pose()
        time.sleep(0.5)
        
    except Exception as e:
        print(f"Error during beer opening: {e}")
        
    finally:
        bot.core.robot_torque_enable(cmd_type='group', name='all', enable=False)
        robot_shutdown()

if __name__ == '__main__':
    open_beer_state_machine("heineken")