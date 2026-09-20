import sys
import types
from enum import Enum

import pytest
from fastapi.testclient import TestClient

# Stub robot modules so the app can be imported in CI/unit-test environments without the
# hardware package being present. The app behavior under test here does not require the real robot.
xs_modules = types.ModuleType("interbotix_xs_modules")
xs_robot_pkg = types.ModuleType("interbotix_xs_modules.xs_robot")
xs_arm_mod = types.ModuleType("interbotix_xs_modules.xs_robot.arm")


class _DummyManipulatorXS:
    pass


xs_arm_mod.InterbotixManipulatorXS = _DummyManipulatorXS
xs_robot_pkg.arm = xs_arm_mod
xs_modules.xs_robot = xs_robot_pkg

sys.modules.setdefault("interbotix_xs_modules", xs_modules)
sys.modules.setdefault("interbotix_xs_modules.xs_robot", xs_robot_pkg)
sys.modules.setdefault("interbotix_xs_modules.xs_robot.arm", xs_arm_mod)

common_modules = types.ModuleType("interbotix_common_modules")
common_robot_pkg = types.ModuleType("interbotix_common_modules.common_robot")
common_robot_mod = types.ModuleType("interbotix_common_modules.common_robot.robot")
common_robot_mod.robot_shutdown = lambda *args, **kwargs: None
common_robot_mod.robot_startup = lambda *args, **kwargs: None
common_robot_pkg.robot = common_robot_mod
common_modules.common_robot = common_robot_pkg

sys.modules.setdefault("interbotix_common_modules", common_modules)
sys.modules.setdefault("interbotix_common_modules.common_robot", common_robot_pkg)
sys.modules.setdefault("interbotix_common_modules.common_robot.robot", common_robot_mod)

import main as app_module
from src.dependencies.ros_manager import get_ros_manager as ros_getter
from src.dependencies.state_machine_deps import get_state_machine_manager as state_getter
from src.dependencies.state_machine_deps import get_ready_state_machine_manager as ready_state_getter
from src.routers import task as task_module


class FakeROSStatus(Enum):
    RUNNING = "running"


class FakeROSManager:
    async def check_ros2_running(self):
        return True

    def get_status(self):
        return (FakeROSStatus.RUNNING, None)

    def is_running(self):
        return True

    async def start_ros_launch(self):
        return True

    async def stop_ros_launch(self):
        return True

    async def restart(self):
        return True


class FakeBotManager:
    async def safe_shutdown(self):
        return True


class FakeStateMachineManager:
    def __init__(self):
        self.ready = True

    def get_status(self):
        return {
            "is_running": False,
            "current_state": None,
            "current_sequence": None,
            "current_step": 0,
            "total_steps": None,
            "progress_percentage": 0.0,
            "last_error": None,
            "start_time": None,
            "end_time": None,
            "execution_time_seconds": None,
        }

    def create_state_machine(self, state_machine_class, **kwargs):
        self.created_class = state_machine_class.__name__
        self.created_kwargs = kwargs
        return True

    def start_execution(self):
        self.ready = False
        return True

    def stop_execution(self):
        self.ready = True
        return True

    def is_ready(self):
        return self.ready

    async def initialize_robot(self, robot_model: str = "wx250"):
        return True

    def shutdown_robot(self):
        return None


@pytest.fixture
def client(monkeypatch):
    fake_ros = FakeROSManager()
    fake_bot = FakeBotManager()
    fake_state_manager = FakeStateMachineManager()

    monkeypatch.setattr(app_module, "get_ros_manager", lambda: fake_ros)
    monkeypatch.setattr(app_module, "get_robot_manager", lambda: fake_bot)
    monkeypatch.setattr(app_module, "get_state_machine_manager", lambda: fake_state_manager)

    async def _async_initialize_arm_resources():
        return None

    async def _async_shutdown_arm_resources():
        return None

    monkeypatch.setattr(app_module, "initialize_arm_resources", _async_initialize_arm_resources)
    monkeypatch.setattr(app_module, "shutdown_arm_resources", _async_shutdown_arm_resources)

    app_module.app.dependency_overrides.clear()
    app_module.app.dependency_overrides[ros_getter] = lambda: fake_ros
    app_module.app.dependency_overrides[state_getter] = lambda: fake_state_manager
    app_module.app.dependency_overrides[ready_state_getter] = lambda: fake_state_manager

    original_task_functions = dict(task_module.TASK_FUNCTIONS)
    task_module.TASK_FUNCTIONS["open_beer_bottle"] = lambda beer_brand: {"beer_brand": beer_brand}
    task_module.TASK_FUNCTIONS["open_beer_state_machine"] = lambda beer_brand: {"beer_brand": beer_brand}

    try:
        with TestClient(app_module.app) as test_client:
            yield test_client
    finally:
        app_module.app.dependency_overrides.clear()
        task_module.TASK_FUNCTIONS.clear()
        task_module.TASK_FUNCTIONS.update(original_task_functions)
