from fastapi import APIRouter, Depends, HTTPException
from src.interbotix_ros_manipulators.interbotix_ros_xsarms.interbotix_xsarm_control.scripts.open_beer import main
# src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/scripts/open_beer.py
scripts_router = APIRouter(
    prefix="/scripts",
    responses={404: {"description": "Script not found"}},
    tags=["scripts"]
)

@scripts_router.post("/run/{script_name}")
async def run_script(
    script_name: str
):
    {"open_beer": main}[script_name]()
    return {"script_called": script_name}