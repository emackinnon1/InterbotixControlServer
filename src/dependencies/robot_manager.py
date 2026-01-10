import asyncio
import logging
import time
from typing import Optional

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from .ros_manager import get_ros_manager

LOGGER = logging.getLogger(__name__)


class RobotConnectionManager:
    """Ensures ROS is running and ready before creating the robot instance."""

    def __init__(self, robot_model: str = "wx250"):
        self._lock = asyncio.Lock()
        self._robot: Optional[InterbotixManipulatorXS] = None
        self._robot_model = robot_model
        self._namespace = f"/{robot_model}"

    async def get_robot(self) -> InterbotixManipulatorXS:
        async with self._lock:
            if self._robot is None:
                await self._ensure_ros_ready()
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

    async def _ensure_ros_ready(self):
        ros_manager = get_ros_manager()
        if ros_manager.is_running():
            await self._wait_for_namespace()
            return

        ros_running = await ros_manager.check_ros2_running()
        if ros_running:
            await self._wait_for_namespace()
            return

        started = await ros_manager.start_ros_launch()
        if not started:
            status, error = ros_manager.get_status()
            detail = error if error else "unknown error"
            raise RuntimeError(f"Failed to start ROS launch (status={status.value}): {detail}")
        await self._wait_for_namespace()

    async def _wait_for_namespace(self, timeout: float = 30.0, poll_interval: float = 1.0):
        """Wait until ROS services under the robot namespace are available."""
        deadline = time.monotonic() + timeout
        namespace_prefix = f"{self._namespace}/"
        last_error = None

        while time.monotonic() < deadline:
            try:
                services = await self._list_ros_services()
            except Exception as exc:  # noqa: BLE001
                last_error = str(exc)
                await asyncio.sleep(poll_interval)
                continue

            if any(s.startswith(namespace_prefix) for s in services):
                return

            await asyncio.sleep(poll_interval)

        detail = last_error or "timeout waiting for ROS services"
        raise RuntimeError(f"ROS namespace {self._namespace} not ready: {detail}")

    async def _list_ros_services(self) -> list[str]:
        process = await asyncio.create_subprocess_exec(
            "ros2",
            "service",
            "list",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await asyncio.wait_for(process.communicate(), timeout=5)

        if process.returncode != 0:
            raise RuntimeError(stderr.decode().strip() or "ros2 service list failed")

        services = [line.strip() for line in stdout.decode().splitlines() if line.strip()]
        return services


def get_robot_manager() -> RobotConnectionManager:
    if not hasattr(get_robot_manager, "_instance"):
        get_robot_manager._instance = RobotConnectionManager()
    return get_robot_manager._instance
