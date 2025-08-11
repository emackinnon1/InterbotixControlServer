from fastapi import APIRouter
import subprocess
from enum import Enum
# from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
# from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

class TorqueStateEnum(Enum):
  enable = "true"
  disable = "false"


arm_router = APIRouter(
  prefix="/arm",
  tags=["arm"]
)

@arm_router.get("/position")
async def arm_position():
  return {"joint_1": 2}

# TODO: change this route to state machine and use state machine to control torque
@arm_router.get("/torque/{desired_state}")
async def torque(desired_state: TorqueStateEnum):
  """
  Enable or disable torque for the robot arm.
  
  Args:
    desired_state: Either 'enable' or 'disable' to control torque state
  
  Returns:
    dict: Status of the torque command execution
  """
  try:
    # Use enum value directly for the ROS service call
    enable_value = desired_state.value
    
    # Construct the ROS2 service call command
    command = [
      "ros2", "service", "call",
      "/wx250/torque_enable",
      "interbotix_xs_msgs/srv/TorqueEnable",
      f"{{cmd_type: 'group', name: 'all', enable: {enable_value}}}"
    ]
    
    # Execute the command and wait for completion
    result = subprocess.run(
      command,
      capture_output=True,
      text=True,
      timeout=10  # 10 second timeout
    )
    
    if result.returncode == 0:
      return {
        "status": "success",
        "action": desired_state.value,
        "message": f"Torque {desired_state.value}d successfully",
        "output": result.stdout.strip() if result.stdout else None
      }
    else:
      return {
        "status": "error",
        "action": desired_state.value,
        "message": f"Failed to {desired_state.value} torque",
        "error": result.stderr.strip() if result.stderr else "Unknown error",
        "return_code": result.returncode
      }
      
  except subprocess.TimeoutExpired:
    return {
      "status": "error",
      "action": desired_state.value,
      "message": "Command timed out",
      "error": "ROS2 service call took too long to respond"
    }
  except Exception as e:
    return {
      "status": "error", 
      "action": desired_state.value,
      "message": f"Failed to execute torque command",
      "error": str(e)
    }