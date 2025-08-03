# endpoints to report ros status, restart ros2, or kill pids
# create dependency to double check ROS status before running commands
from fastapi import APIRouter, Depends, HTTPException

ros_router = APIRouter(
    prefix="/ros",
    responses={404: {"description": "Not found"}},
    tags=["ros"]
)

@ros_router.post("/status")
async def ros_status():
    return {"status": "running"}