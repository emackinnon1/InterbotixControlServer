from enum import Enum

from fastapi import APIRouter, Depends, HTTPException
from scripts.open_beer import open_beer
from scripts.open_beer_state_machine import open_beer_state_machine

tasks_router = APIRouter(
    prefix="/tasks",
    responses={404: {"description": "Task not found"}},
    tags=["tasks"]
)

class TasksEnum(Enum):
    open_beer_bottle = "open_beer_bottle"
    open_beer_state_machine = "open_beer_state_machine"

class BeerBrandEnum(Enum):
    sapporo = "sapporo"
    heineken = "heineken"
    corona = "corona"

# Task function mapping
TASK_FUNCTIONS = {
    "open_beer_bottle": open_beer,
    "open_beer_state_machine": open_beer_state_machine
}

@tasks_router.post("/perform/{task_name}")
async def perform_task(
    task_name: TasksEnum,
    beer_brand: BeerBrandEnum
):
    task_func = TASK_FUNCTIONS.get(task_name.value)
    if not task_func:
        raise HTTPException(status_code=404, detail=f"Task {task_name.value} not found")
    
    task_func(beer_brand.value)
    return {"task_name": task_name}
