from enum import Enum

from fastapi import APIRouter, Depends, HTTPException
from src.interbotix_ros_manipulators.interbotix_ros_xsarms.interbotix_xsarm_control.scripts.open_beer import open_beer
# src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/scripts/open_beer.py
scripts_router = APIRouter(
    prefix="/scripts",
    responses={404: {"description": "Script not found"}},
    tags=["scripts"]
)

class TasksEnum(Enum):
    open_beer_bottle="open_beer_bottle"

class BeerBrandEnum(Enum):
    sapporo="sapporo"
    heineken="heineken"

# POST /task/open_beer_bottle?beer_brand=sapporo
@scripts_router.post("/task/{task_name}")
def perform_task(
    task_name: TasksEnum,
    beer_brand: BeerBrandEnum
):
    {"open_beer_bottle": open_beer}[task_name.value](beer_brand.value)
    return {"task_name": task_name}