from fastapi import Depends, HTTPException
from src.state_machine.state_machine_deps import StateMachineManager

def get_state_machine_manager() -> StateMachineManager:
    """Dependency to get the singleton state machine manager"""
    return StateMachineManager()

def get_ready_state_machine_manager(
    manager: StateMachineManager = Depends(get_state_machine_manager)
) -> StateMachineManager:
    """Dependency that ensures the state machine manager is ready"""
    if not manager.is_ready():
        raise HTTPException(
            status_code=409, 
            detail="State machine manager is not ready or is currently executing"
        )
    return manager