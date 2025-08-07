from fastapi import FastAPI
from routers.task import scripts_router
from src.routers.arm import arm_router
from src.routers.ros import ros_router

app = FastAPI()

app.include_router(scripts_router)
app.include_router(arm_router)
app.include_router(ros_router)

@app.get("/")
async def root():
    return {"message": "Hello World"}


