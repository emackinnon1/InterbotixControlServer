import asyncio
import logging
from fastapi import FastAPI
from contextlib import asynccontextmanager

from src.dependencies.ros_manager import get_ros_manager
from src.dependencies.robot_manager import get_robot_manager
from src.routers.task import tasks_router
from src.routers.arm import arm_router
from src.routers.ros import ros_router
from src.routers.state_machine import state_machine_router
from src.state_machine.safe_shutdown import safe_shutdown_sync
from src.dependencies.state_machine_manager import get_state_machine_manager



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
    
    # stop state machine
    state_machine_manager = get_state_machine_manager()
    state_machine_manager.stop_execution()
    
    # shutdown bot
    bot_manager = get_robot_manager()
    bot = await bot_manager.get_robot()
    await asyncio.to_thread(safe_shutdown_sync, bot)
    
    # stop ROS
    await ros_manager.stop_ros_launch()


app = FastAPI(lifespan=lifespan)

app.include_router(tasks_router)
app.include_router(arm_router)
app.include_router(ros_router)
app.include_router(state_machine_router)

@app.get("/")
def read_root():
    return {"message": "Interbotix Control Server is running"}
