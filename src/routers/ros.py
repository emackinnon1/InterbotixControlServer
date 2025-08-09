# endpoints to report ros status, restart ros2, or kill pids
# create dependency to double check ROS status before running commands
from fastapi import APIRouter, Depends, HTTPException
from src.dependencies.ros_manager import ROSLaunchManager, get_ros_manager

ros_router = APIRouter(
    prefix="/ros",
    responses={404: {"description": "Not found"}},
    tags=["ros"]
)

@ros_router.get("/running")
async def ros_running(manager: ROSLaunchManager = Depends(get_ros_manager)):
    return {"ros_is_running": manager.check_ros2_running()}

@ros_router.get("/status")
async def ros_status(manager: ROSLaunchManager = Depends(get_ros_manager)):
    """Get the current status of the ROS launch process"""
    status, error = manager.get_status()
    return {
        "status": status.value,
        "error": error,
        "ros_launch_process_running": manager.is_running()
    }

@ros_router.post("/start")
async def start_ros(manager: ROSLaunchManager = Depends(get_ros_manager)):
    """Start the ROS launch process"""
    success = manager.start_ros_launch()
    if not success:
        status, error = manager.get_status()
        raise HTTPException(status_code=500, detail=f"Failed to start ROS: {error}")
    
    return {"message": "ROS launch started successfully", "status": "starting"}

@ros_router.post("/stop")
async def stop_ros(manager: ROSLaunchManager = Depends(get_ros_manager)):
    """Stop the ROS launch process"""
    success = manager.stop_ros_launch()
    if not success:
        status, error = manager.get_status()
        raise HTTPException(status_code=500, detail=f"Failed to stop ROS: {error}")
    
    return {"message": "ROS launch stopped successfully", "status": "stopped"}

@ros_router.post("/restart")
async def restart_ros(manager: ROSLaunchManager = Depends(get_ros_manager)):
    """Restart the ROS launch process"""
    success = manager.restart()
    if not success:
        status, error = manager.get_status()
        raise HTTPException(status_code=500, detail=f"Failed to restart ROS: {error}")
    
    return {"message": "ROS launch restarted successfully", "status": "restarting"}