from fastapi import APIRouter
# from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
# from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


arm_router = APIRouter(
  prefix="/arm",
  tags=["arm"]
)

# bot = InterbotixManipulatorXS(
#   robot_model='wx250',
#   group_name='arm',
#   gripper_name='gripper',
#   moving_time=1.5,
#   gripper_pressure=0.85           
# )

@arm_router.get("/position")
async def arm_position():
  return {"joint_1": 2}