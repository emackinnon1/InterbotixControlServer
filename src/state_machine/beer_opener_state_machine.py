#!/usr/bin/env python3
import time
from enum import Enum, auto
from typing import Dict
import numpy as np
from .abstract_state_machine import (
    AbstractStateMachine, Movement, MovementSequence, MovementType
)

# Constants and BRAND_CONFIGS from your existing implementation
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

BRAND_CONFIGS = {
    "sapporo": {
        "waist_rotation": -np.pi/5,
        "bottle_lower_distance": BOTTLE_LOWER_DISTANCE
    },
    "heineken": {
        "waist_rotation": -np.pi/4.75,
        "bottle_lower_distance": BOTTLE_LOWER_DISTANCE
    },
    "corona": {
        "waist_rotation": -np.pi/4.7,
        "bottle_lower_distance": -0.182
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

class BeerOpenerStateMachine(AbstractStateMachine[BeerOpenerState]):
    def __init__(self, bot, brand: str, default_wait_time: float = 2.0):
        self.brand = brand
        super().__init__(bot, default_wait_time)
    
    def get_initial_state(self) -> BeerOpenerState:
        return BeerOpenerState.INIT
    
    def get_complete_state(self) -> BeerOpenerState:
        return BeerOpenerState.COMPLETE
    
    def get_error_state(self) -> BeerOpenerState:
        return BeerOpenerState.ERROR
    
    def _safe_shutdown_sequence(self) -> bool:
        """
        Safely return arm to home, then sleep, then turn off torque.
        Returns True if successful, False if any step fails.
        """
        try:
            print("Error detected - initiating safe shutdown sequence...")
            
            # Step 1: Release gripper in case something is grasped
            try:
                self.bot.gripper.release()
                print("Released gripper")
                time.sleep(0.5)
            except Exception as e:
                print(f"Warning: Could not release gripper: {e}")
            
            # Step 2: Go to home position
            try:
                self.bot.arm.go_to_home_pose(moving_time=2.0)
                print("Returned to home pose")
                time.sleep(1.0)
            except Exception as e:
                print(f"Warning: Could not go to home pose: {e}")
            
            # Step 3: Go to sleep position
            try:
                self.bot.arm.go_to_sleep_pose(moving_time=2.0)
                print("Moved to sleep pose")
                time.sleep(1.0)
            except Exception as e:
                print(f"Warning: Could not go to sleep pose: {e}")
            
            # Step 4: Turn off torque
            try:
                self.bot.core.robot_reboot_motors(cmd_type='group', name='all', enable=False, smart_reboot=True)
                self.bot.core.robot_torque_enable(cmd_type='group', name='all', enable=False)
                print("Torque disabled - arm is now safe")
                return True
            except Exception as e:
                print(f"Warning: Could not disable torque: {e}")
                return False
                
        except Exception as e:
            print(f"Critical error during safe shutdown: {e}")
            return False
    
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

        brand_config = BRAND_CONFIGS.get(self.brand)
        if not brand_config:
            raise ValueError(f"Unknown brand: {self.brand}")
            
        waist_rotation = brand_config.get("waist_rotation")
        bottle_lower_distance = brand_config.get("bottle_lower_distance")
        
        if waist_rotation is None or bottle_lower_distance is None:
            raise ValueError(f"Incomplete configuration for brand: {self.brand}")
            
        return MovementSequence("open_bottle", [
            Movement(MovementType.CARTESIAN_MOVE, {'z': bottle_lower_distance}, "Lower to bottle level"),
            Movement(MovementType.WAIT, {'duration': 2.0}, "Wait at bottle level"),
            Movement(MovementType.JOINT_MOVE, {'joint_name': 'waist', 'position': waist_rotation}, "Position opener on bottle cap"),
            Movement(MovementType.WAIT, {'duration': 1.0}, "Wait for positioning"),
            Movement(MovementType.JOINT_MOVE, {'joint_name': 'wrist_rotate', 'position': WRIST_ROTATE_OPEN, 'moving_time': 0.25}, "Rotate wrist to open"),
            Movement(MovementType.CARTESIAN_MOVE, {'z': BOTTLE_RAISE_DISTANCE}, "Raise while opening"),
            Movement(MovementType.JOINT_MOVE, {'joint_name': 'waist', 'position': WAIST_BOTTLE_POSITION, 'moving_time': 1.25}, "Complete opening motion"),
            Movement(MovementType.WAIT, {'duration': 0.5}, "Wait for opening completion")
        ])
    
    def _process_current_state(self) -> bool:
        """Process the current state and advance if needed"""
        try:
            if self.state == BeerOpenerState.INIT:
                print("Starting beer opener sequence")
                self._transition_to_sequence("pickup_opener", BeerOpenerState.PICKUP_OPENER)
                
            elif self.state == BeerOpenerState.PICKUP_OPENER:
                if not self._execute_current_sequence():
                    return False
                if self.current_sequence.is_complete():
                    self._transition_to_sequence("approach_bottle", BeerOpenerState.APPROACH_BOTTLE)
                    
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
                    self._transition_to_sequence("return_opener", BeerOpenerState.RETURN_OPENER)
                    
            elif self.state == BeerOpenerState.RETURN_OPENER:
                if not self._execute_current_sequence():
                    return False
                if self.current_sequence.is_complete():
                    self.state = BeerOpenerState.COMPLETE
                    self.current_sequence = None
                    
            elif self.state == BeerOpenerState.ERROR:
                print("State machine in ERROR state - executing safe shutdown")
                self._safe_shutdown_sequence()
                return False  # Stop execution after error handling
                    
            return True
            
        except Exception as e:
            print(f"Error in state machine: {e}")
            self.state = BeerOpenerState.ERROR
            self._safe_shutdown_sequence()
            return False

def open_beer_state_machine(brand: str, wait_time: float = 2.0):
    """Main function to open a beer bottle using the abstract state machine"""
    from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
    
    bot = InterbotixManipulatorXS(
        robot_model='wx250',
        group_name='arm',
        gripper_name='gripper',
        moving_time=1.25,
        gripper_pressure=0.85           
    )

    # Create and run state machine with robot management
    state_machine = BeerOpenerStateMachine(bot, brand, wait_time)
    return state_machine.run_with_robot_management()

if __name__ == '__main__':
    open_beer_state_machine("heineken")