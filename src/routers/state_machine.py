from enum import Enum
from typing import Dict, Any
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

from src.dependencies.state_machine_deps import (
    get_state_machine_manager, 
    get_ready_state_machine_manager
)
from src.state_machine.state_machine_manager import StateMachineManager
from src.state_machine.beer_opener_state_machine import BeerOpenerStateMachine

state_machine_router = APIRouter(
    prefix="/state-machine",
    tags=["state-machine"]
)

# TODO: make an experiment route that lets me pass in the movement sequences
# TODO: create a method to control torque and add it as a state. Should also check torque state before every process.
class BeerBrandEnum(Enum):
    corona = "corona"
    sapporo = "sapporo"
    heineken = "heineken"

class StartBeerOpenerRequest(BaseModel):
    brand: BeerBrandEnum
    wait_time: float = 2.5

@state_machine_router.get("/status")
def get_state_machine_status(
    manager: StateMachineManager = Depends(get_state_machine_manager)
) -> Dict[str, Any]:
    """Get the current status of the state machine"""
    return manager.get_status()

@state_machine_router.post("/beer-opener/start")
async def start_beer_opener(
    request: StartBeerOpenerRequest,
    manager: StateMachineManager = Depends(get_ready_state_machine_manager)
) -> Dict[str, str]:
    """Start the beer opener state machine"""
    
    # Create the state machine
    success = manager.create_state_machine(
        BeerOpenerStateMachine,
        brand=request.brand.value,
        default_wait_time=request.wait_time
    )
    
    if not success:
        raise HTTPException(status_code=500, detail="Failed to create beer opener state machine")
    
    # Start execution
    success = manager.start_execution()
    
    if not success:
        raise HTTPException(status_code=500, detail="Failed to start state machine execution")
    
    return {"message": f"Beer opener started for {request.brand.value}"}

@state_machine_router.post("/stop")
async def stop_state_machine(
    manager: StateMachineManager = Depends(get_state_machine_manager)
) -> Dict[str, str]:
    """Stop the currently running state machine"""
    
    success = manager.stop_execution()
    
    if not success:
        raise HTTPException(status_code=500, detail="Failed to stop state machine")
    
    return {"message": "State machine stop requested"}

@state_machine_router.get("/is-ready")
def is_state_machine_ready(
    manager: StateMachineManager = Depends(get_state_machine_manager)
) -> Dict[str, bool]:
    """Check if the state machine manager is ready for new operations"""
    return {"ready": manager.is_ready()}

@state_machine_router.post("/initialize")
async def initialize_robot(
    manager: StateMachineManager = Depends(get_state_machine_manager),
    robot_model: str = "wx250"
) -> Dict[str, str]:
    """Initialize the robot connection"""
    
    success = manager.initialize_robot(robot_model)
    
    if not success:
        raise HTTPException(status_code=500, detail="Failed to initialize robot")
    
    return {"message": "Robot initialized successfully"}

@state_machine_router.post("/shutdown")
async def shutdown_robot(
    manager: StateMachineManager = Depends(get_state_machine_manager)
) -> Dict[str, str]:
    """Shutdown the robot connection"""
    
    manager.shutdown_robot()
    
    return {"message": "Robot shutdown initiated"}