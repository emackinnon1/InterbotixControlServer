#!/usr/bin/env python3
import threading
from typing import Optional, Dict, Any
from enum import Enum
from dataclasses import dataclass, asdict
from datetime import datetime
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup

from src.dependencies.robot_manager import get_robot_manager
from .abstract_state_machine import AbstractStateMachine
from .beer_opener_state_machine import DEFAULT_MOVING_TIME

@dataclass
class StateMachineStatus:
    """Status information about the current state machine"""
    is_running: bool = False
    current_state: Optional[str] = None
    current_sequence: Optional[str] = None
    current_step: Optional[int] = None
    total_steps: Optional[int] = None
    progress_percentage: float = 0.0
    last_error: Optional[str] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    execution_time_seconds: Optional[float] = None

class StateMachineManager:
    """Singleton manager for state machines"""
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if not self._initialized:
            self._initialized = True
            self._current_state_machine: Optional[AbstractStateMachine] = None
            self._robot: Optional[InterbotixManipulatorXS] = None
            self._status = StateMachineStatus()
            self._execution_thread: Optional[threading.Thread] = None
            self._stop_requested = False
    
    async def initialize_robot(self, robot_model: str = 'wx250') -> bool:
        """Initialize the robot connection"""
        try:
            if self._robot is not None:
                return True

            self._manager = get_robot_manager()
            self._robot = await self._manager.get_robot()
            
            return True
            
        except Exception as e:
            self._status.last_error = f"Failed to initialize robot: {str(e)}"
            return False
    
    def shutdown_robot(self):
        """Shutdown the robot connection"""
        try:
            if self._robot is not None:
                self._robot.arm.go_to_home_pose()
                self._robot.arm.go_to_sleep_pose()
                self._robot.core.robot_torque_enable(cmd_type='group', name='all', enable=False)
                robot_shutdown()
                self._robot = None
        except Exception as e:
            self._status.last_error = f"Error shutting down robot: {str(e)}"
    
    def create_state_machine(self, state_machine_class, **kwargs) -> bool:
        """Create a new state machine instance"""
        try:
            if not self.initialize_robot():
                return False
            
            if self._status.is_running:
                raise RuntimeError("Cannot create new state machine while another is running")
            
            self._current_state_machine = state_machine_class(self._robot, **kwargs)
            self._update_status()
            return True
            
        except Exception as e:
            self._status.last_error = f"Failed to create state machine: {str(e)}"
            return False
    
    def start_execution(self) -> bool:
        """Start executing the current state machine in a separate thread"""
        try:
            if self._current_state_machine is None:
                raise RuntimeError("No state machine created")
            
            if self._status.is_running:
                raise RuntimeError("State machine is already running")
            
            self._stop_requested = False
            self._status.is_running = True
            self._status.start_time = datetime.now()
            self._status.end_time = None
            self._status.last_error = None
            
            self._execution_thread = threading.Thread(target=self._execute_state_machine)
            self._execution_thread.start()
            
            return True
            
        except Exception as e:
            self._status.last_error = f"Failed to start execution: {str(e)}"
            self._status.is_running = False
            return False
    
    def stop_execution(self) -> bool:
        """Request to stop the current execution"""
        try:
            if not self._status.is_running:
                return True
            
            self._stop_requested = True
            
            if self._execution_thread and self._execution_thread.is_alive():
                self._execution_thread.join(timeout=5.0)
            
            self._status.is_running = False
            self._status.end_time = datetime.now()
            if self._status.start_time:
                self._status.execution_time_seconds = (
                    self._status.end_time - self._status.start_time
                ).total_seconds()
            
            return True
            
        except Exception as e:
            self._status.last_error = f"Failed to stop execution: {str(e)}"
            return False
    
    def _execute_state_machine(self):
        """Execute the state machine (runs in separate thread)"""
        try:
            success = self._current_state_machine.run_with_robot_management()
            
            self._status.is_running = False
            self._status.end_time = datetime.now()
            
            if self._status.start_time:
                self._status.execution_time_seconds = (
                    self._status.end_time - self._status.start_time
                ).total_seconds()
            
            if not success and not self._stop_requested:
                self._status.last_error = "State machine execution failed"
                
        except Exception as e:
            self._status.is_running = False
            self._status.end_time = datetime.now()
            self._status.last_error = f"Execution error: {str(e)}"
    
    def _update_status(self):
        """Update the current status based on state machine state"""
        if self._current_state_machine is None:
            return
        
        self._status.current_state = str(self._current_state_machine.state.name)
        
        if hasattr(self._current_state_machine, 'current_sequence') and self._current_state_machine.current_sequence:
            seq = self._current_state_machine.current_sequence
            self._status.current_sequence = seq.name
            self._status.current_step = seq.current_step
            self._status.total_steps = len(seq.movements)
            
            if self._status.total_steps > 0:
                self._status.progress_percentage = (
                    self._status.current_step / self._status.total_steps * 100
                )
    
    def get_status(self) -> Dict[str, Any]:
        """Get current status as dictionary"""
        self._update_status()
        return asdict(self._status)
    
    def is_ready(self) -> bool:
        """Check if the manager is ready for new operations"""
        return (
            self._robot is not None and 
            not self._status.is_running and 
            not self._stop_requested
        )
    
    def cleanup(self):
        """Cleanup resources"""
        self.stop_execution()
        self.shutdown_robot()
        self._current_state_machine = None