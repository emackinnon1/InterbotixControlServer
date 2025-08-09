import logging
from fastapi import FastAPI
from src.routers.task import tasks_router
from src.routers.arm import arm_router
from src.routers.ros import ros_router
from src.routers.state_machine import state_machine_router

# Configure logging to show INFO level and above
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

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
