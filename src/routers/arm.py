from fastapi import APIRouter, HTTPException
import subprocess
from enum import Enum
from pydantic import BaseModel, Field, model_validator
from typing import Dict, Any, Optional, List
import asyncio
import time
import uuid
import logging

from src.state_machine.abstract_state_machine import Movement, MovementType, MovementExecutor
from src.state_machine.safe_shutdown import safe_shutdown_sync
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

LOGGER = logging.getLogger(__name__)

class TorqueStateEnum(Enum):
  enable = "true"
  disable = "false"


arm_router = APIRouter(
  prefix="/arm",
  tags=["arm"]
)

############################
# Movement job infrastructure
############################

class MovementCommandType(str, Enum):
  set_single_joint_position = "set_single_joint_position"
  set_ee_cartesian_trajectory = "set_ee_cartesian_trajectory"
  set_ee_pose_components = "set_ee_pose_components"
  gripper_release = "gripper_release"
  gripper_grasp = "gripper_grasp"

class MovementCommandInput(BaseModel):
  type: MovementCommandType
  params: Dict[str, Any] = Field(default_factory=dict)
  timeout: float = 10.0
  skip_default_wait: bool = False

class ArmMoveRequest(BaseModel):
  go_home_first: bool = False
  moving_time: float = 2.0
  command: Optional[MovementCommandInput] = None
  commands: Optional[List[MovementCommandInput]] = None

  @model_validator(mode="after")
  def validate_exclusive(self):
    if (self.command and self.commands) or (not self.command and not self.commands):
      raise ValueError("Provide either 'command' or 'commands'.")
    if self.moving_time <= 0:
      raise ValueError("moving_time must be > 0")
    return self

class JobState(str, Enum):
  queued = "queued"
  running = "running"
  succeeded = "succeeded"
  failed = "failed"

class JobStep(BaseModel):
  index: int
  type: str
  description: str
  success: bool = False
  error: Optional[str] = None

class ArmMoveJobStatus(BaseModel):
  job_id: str
  state: JobState
  submitted_at: float
  started_at: Optional[float] = None
  finished_at: Optional[float] = None
  steps: List[JobStep] = []
  stopped_at_index: Optional[int] = None
  elapsed_ms: Optional[int] = None

# Globals
_robot: Optional[InterbotixManipulatorXS] = None
_job_queue: "asyncio.Queue[dict]" = asyncio.Queue()
_jobs: Dict[str, ArmMoveJobStatus] = {}
_worker_task: Optional[asyncio.Task] = None
_worker_lock = asyncio.Lock()

def _init_robot():
  global _robot
  if _robot is None:
    _robot = InterbotixManipulatorXS(
      robot_model='wx250',
      group_name='arm',
      gripper_name='gripper',
      moving_time=2.0,
      gripper_pressure=0.9
    )
    # Ensure torque is enabled for operation
    try:
      _robot.core.robot_torque_enable(cmd_type='group', name='all', enable=True)
    except Exception as e:
      LOGGER.info(f"Exception enabling torque: {e}")
      raise e

async def _start_worker():
  global _worker_task
  async with _worker_lock:
    if _worker_task is None:
      _worker_task = asyncio.create_task(_jobs_worker())

def _validate_params(cmd: MovementCommandInput):
  t = cmd.type
  p = cmd.params
  if t == MovementCommandType.set_single_joint_position:
    if "joint_name" not in p or "position" not in p:
      raise ValueError("set_single_joint_position requires joint_name and position")
  elif t == MovementCommandType.set_ee_cartesian_trajectory:
    if not any(k in p for k in ("x", "y", "z")):
      raise ValueError("set_ee_cartesian_trajectory requires at least one of x,y,z")
    # Only XYZ; silently ignore other fields if provided
    allowed = {"x", "y", "z", "moving_time", "accel_time", "blocking"}
    for k in list(p.keys()):
      if k not in allowed:
        p.pop(k, None)
  elif t == MovementCommandType.set_ee_pose_components:
    if not any(k in p for k in ("x", "y", "z")):
      raise ValueError("set_ee_pose_components requires at least one of x,y,z")
    # If user provided roll/pitch/yaw we keep them for future, but executor will use what it supports
  elif t in (MovementCommandType.gripper_grasp, MovementCommandType.gripper_release):
    cmd.params = {}
  else:
    raise ValueError(f"Unknown command type {t}")

def _map_to_movements(cmds: List[MovementCommandInput], default_moving_time: float, go_home_first: bool) -> List[Movement]:
  movements: List[Movement] = []
  if go_home_first:
    movements.append(Movement(MovementType.GO_HOME, {'moving_time': default_moving_time}, "Go home first"))
  for c in cmds:
    _validate_params(c)
    params = dict(c.params)
    if c.type in {
      MovementCommandType.set_single_joint_position,
      MovementCommandType.set_ee_cartesian_trajectory,
      MovementCommandType.set_ee_pose_components,
    } and 'moving_time' not in params:
      params['moving_time'] = default_moving_time

    if c.type == MovementCommandType.set_single_joint_position:
      movements.append(Movement(MovementType.JOINT_MOVE, params, f"Joint move {params.get('joint_name')}", timeout=c.timeout, skip_default_wait=c.skip_default_wait))
    elif c.type == MovementCommandType.set_ee_cartesian_trajectory:
      movements.append(Movement(MovementType.CARTESIAN_MOVE, params, "Cartesian trajectory", timeout=c.timeout, skip_default_wait=c.skip_default_wait))
    elif c.type == MovementCommandType.set_ee_pose_components:
      movements.append(Movement(MovementType.POSE_COMPONENTS, params, "Pose components move", timeout=c.timeout, skip_default_wait=c.skip_default_wait))
    elif c.type == MovementCommandType.gripper_grasp:
      movements.append(Movement(MovementType.GRIPPER_ACTION, {'action': 'grasp'}, "Gripper grasp", timeout=c.timeout, skip_default_wait=c.skip_default_wait))
    elif c.type == MovementCommandType.gripper_release:
      movements.append(Movement(MovementType.GRIPPER_ACTION, {'action': 'release'}, "Gripper release", timeout=c.timeout, skip_default_wait=c.skip_default_wait))
  return movements

async def _jobs_worker():
  if _robot is None:
    return
  executor = MovementExecutor(_robot, default_wait_time=1.0)
  while True:
    payload = await _job_queue.get()
    job_id = payload['job_id']
    movements: List[Movement] = payload['movements']
    job = _jobs[job_id]
    job.state = JobState.running
    job.started_at = time.time()
    steps: List[JobStep] = []
    failure = False
    try:
      try:
        _robot.core.robot_torque_enable(cmd_type='group', name='all', enable=True)
      except Exception:
        pass

      for i, m in enumerate(movements):
        step = JobStep(index=i, type=m.type.name, description=m.description)
        steps.append(step)
        ok = await asyncio.to_thread(executor.execute_movement, m)
        step.success = ok
        if not ok:
          step.error = "Movement failed"
          job.state = JobState.failed
          job.stopped_at_index = i
          failure = True
          await asyncio.to_thread(safe_shutdown_sync, _robot)
          break
      if not failure:
        job.state = JobState.succeeded
    except Exception as e:
      if steps:
        steps[-1].error = str(e)
      job.state = JobState.failed
      job.stopped_at_index = steps[-1].index if steps else 0
      await asyncio.to_thread(safe_shutdown_sync, _robot)
    finally:
      job.steps = steps
      job.finished_at = time.time()
      job.elapsed_ms = int((job.finished_at - job.submitted_at) * 1000)
      _job_queue.task_done()
      if job.state in (JobState.succeeded, JobState.failed):
        _jobs.pop(job_id, None)

@arm_router.on_event("startup")
async def _on_startup():
  _init_robot()
  await _start_worker()

@arm_router.on_event("shutdown")
async def _on_shutdown():
  if _robot:
    await asyncio.to_thread(safe_shutdown_sync, _robot)

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

@arm_router.post("/move", status_code=202)
async def submit_arm_move(req: ArmMoveRequest):
  if _robot is None:
    raise HTTPException(503, "Robot not initialized")
  try:
    cmds = [req.command] if req.command else req.commands
    movements = _map_to_movements(cmds, req.moving_time, req.go_home_first)
  except Exception as e:
    raise HTTPException(400, f"Invalid commands: {e}")
  job_id = uuid.uuid4().hex
  job = ArmMoveJobStatus(
    job_id=job_id,
    state=JobState.queued,
    submitted_at=time.time(),
    steps=[]
  )
  _jobs[job_id] = job
  await _job_queue.put({'job_id': job_id, 'movements': movements})
  return {"job_id": job_id, "state": job.state}

@arm_router.get("/jobs/{job_id}")
async def get_job(job_id: str):
  job = _jobs.get(job_id)
  if not job:
    raise HTTPException(404, "Job not found")
  return job

@arm_router.get("/jobs")
async def list_jobs():
  return list(_jobs.values())[-50:]