import logging
from fastapi import FastAPI
from contextlib import asynccontextmanager

from src.routers.task import tasks_router
from src.routers.arm import arm_router
from src.routers.ros import ros_router
from src.routers.state_machine import state_machine_router

from src.dependencies.ros_manager import get_ros_manager



# Configure logging to show INFO level and above
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

@asynccontextmanager
async def lifespan(app: FastAPI):
    ros_manager = get_ros_manager()
    await ros_manager.start_ros_launch()
    yield
    await ros_manager.stop_ros_launch()


app = FastAPI()

app.include_router(tasks_router)
app.include_router(arm_router)
app.include_router(ros_router)
app.include_router(state_machine_router)

@app.get("/")
def read_root():
    return {"message": "Interbotix Control Server is running"}

# Cleanup on shutdown
@app.on_event("shutdown")
def shutdown_event():
    from src.state_machine.state_machine_manager import StateMachineManager
    manager = StateMachineManager()
    manager.cleanup()
