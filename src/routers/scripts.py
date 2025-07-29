from fastapi import APIRouter, Depends, HTTPException

scripts_router = APIRouter(
    prefix="/scripts",
    responses={404: {"description": "Script not found"}},
    tags=["scripts"]
)

@scripts_router.post("/run/{script_name}")
async def run_script(
    script_name: str
):
    return {"script": script_name}