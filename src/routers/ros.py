# endpoints to report ros status, restart ros2, or kill pids
# create dependency to double check ROS status before running commands
from fastapi import APIRouter, Depends, HTTPException
from src.dependencies.ros_status import ROSLaunchManager

ros_router = APIRouter(
    prefix="/ros",
    responses={404: {"description": "Not found"}},
    tags=["ros"]
)

@ros_router.post("/running")
async def ros_running():
    manager = ROSLaunchManager("wx250")
    return {"is_running": manager.check_ros2_running()}