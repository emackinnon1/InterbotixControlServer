import logging
from fastapi import FastAPI
from contextlib import asynccontextmanager

from src.dependencies.ros_manager import get_ros_manager
from src.dependencies.robot_manager import get_robot_manager
from src.routers.task import tasks_router
from src.routers.arm import arm_router, initialize_arm_resources, shutdown_arm_resources
from src.routers.ros import ros_router
from src.routers.state_machine import state_machine_router
from src.dependencies.state_machine_deps import get_state_machine_manager



# Configure logging to show INFO level and above
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

@asynccontextmanager
async def lifespan(app: FastAPI):
    ros_manager = get_ros_manager()
    await ros_manager.start_ros_launch()
    bot_manager = get_robot_manager()
    state_machine_manager = get_state_machine_manager()

    try:
        initialized = await state_machine_manager.initialize_robot()
        if not initialized:
            raise RuntimeError("Failed to initialize robot")
        await initialize_arm_resources()
    except Exception:
        try:
            await bot_manager.safe_shutdown()
        finally:
            await ros_manager.stop_ros_launch()
        raise
    
    try:
        yield
    finally:
        state_machine_manager.stop_execution()
        try:
            await shutdown_arm_resources()
        finally:
            try:
                await bot_manager.safe_shutdown()
            finally:
                await ros_manager.stop_ros_launch()


app = FastAPI(lifespan=lifespan)

app.include_router(tasks_router)
app.include_router(arm_router)
app.include_router(ros_router)
app.include_router(state_machine_router)

@app.get("/")
def read_root():
    return {"message": "Interbotix Control Server is running"}
