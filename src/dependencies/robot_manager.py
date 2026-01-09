import asyncio
import logging
from typing import Optional

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from .ros_manager import get_ros_manager

LOGGER = logging.getLogger(__name__)


class RobotConnectionManager:
    """Ensures ROS is running before creating the robot instance."""

    def __init__(self, robot_model: str = "wx250"):
        self._lock = asyncio.Lock()
        self._robot: Optional[InterbotixManipulatorXS] = None
        self._robot_model = robot_model

    async def get_robot(self) -> InterbotixManipulatorXS:
        async with self._lock:
            if self._robot is None:
                await self._ensure_ros_running()
                self._robot = InterbotixManipulatorXS(
                    robot_model=self._robot_model,
                    group_name="arm",
                    gripper_name="gripper",
                    moving_time=2.0,
                    gripper_pressure=0.9,
                )
                try:
                    self._robot.core.robot_torque_enable(cmd_type="group", name="all", enable=True)
                except Exception as exc:
                    LOGGER.info("Exception enabling torque: %s", exc)
                    self._robot = None
                    raise
            return self._robot

    async def _ensure_ros_running(self):
        ros_manager = get_ros_manager()
        if ros_manager.is_running():
            return

        ros_running = await ros_manager.check_ros2_running()
        if ros_running:
            return

        started = await ros_manager.start_ros_launch()
        if not started:
            status, error = ros_manager.get_status()
            detail = error if error else "unknown error"
            raise RuntimeError(f"Failed to start ROS launch (status={status.value}): {detail}")


def get_robot_manager() -> RobotConnectionManager:
    if not hasattr(get_robot_manager, "_instance"):
        get_robot_manager._instance = RobotConnectionManager()
    return get_robot_manager._instance
