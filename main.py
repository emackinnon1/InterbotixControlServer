from fastapi import FastAPI
from src.routers.task import tasks_router
from src.routers.arm import arm_router
from src.routers.ros import ros_router
from src.routers.state_machine import state_machine_router

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
    from src.services.state_machine_manager import StateMachineManager
    manager = StateMachineManager()
    manager.cleanup()
