import importlib
import sys
import types
from pathlib import Path

import pytest


@pytest.fixture()
def spot_gripper_module(monkeypatch):
    calls = []

    class RobotCommandBuilder:
        @staticmethod
        def claw_gripper_open_command():
            calls.append("build_open")
            return "open_command"

        @staticmethod
        def claw_gripper_close_command():
            calls.append("build_close")
            return "close_command"

    class GripperCameraParams:
        MODE_640_480 = 11
        MODE_1280_720 = 1
        MODE_1920_1080 = 14
        MODE_3840_2160 = 15
        MODE_4096_2160 = 17
        MODE_4208_3120 = 16

        def __init__(self, camera_mode=None):
            self.camera_mode = camera_mode

    class GripperCameraParamRequest:
        def __init__(self, params=None):
            self.params = params

    class GripperCameraParamClient:
        default_service_name = "gripper-camera-param"

    gripper_camera_param_pb2 = types.ModuleType("bosdyn.api.gripper_camera_param_pb2")
    gripper_camera_param_pb2.GripperCameraParams = GripperCameraParams
    gripper_camera_param_pb2.GripperCameraParamRequest = GripperCameraParamRequest

    bosdyn = types.ModuleType("bosdyn")
    bosdyn_api = types.ModuleType("bosdyn.api")
    bosdyn_api.gripper_camera_param_pb2 = gripper_camera_param_pb2
    bosdyn_client = types.ModuleType("bosdyn.client")
    bosdyn_client_robot_command = types.ModuleType("bosdyn.client.robot_command")
    bosdyn_client_robot_command.RobotCommandBuilder = RobotCommandBuilder
    bosdyn_client_gripper = types.ModuleType("bosdyn.client.gripper_camera_param")
    bosdyn_client_gripper.GripperCameraParamClient = GripperCameraParamClient

    modules = {
        "bosdyn": bosdyn,
        "bosdyn.api": bosdyn_api,
        "bosdyn.api.gripper_camera_param_pb2": gripper_camera_param_pb2,
        "bosdyn.client": bosdyn_client,
        "bosdyn.client.robot_command": bosdyn_client_robot_command,
        "bosdyn.client.gripper_camera_param": bosdyn_client_gripper,
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    package_root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(package_root))
    sys.modules.pop("spot_driver.spot_gripper", None)
    return importlib.import_module("spot_driver.spot_gripper"), calls


class FakeRobotCommandClient:
    def __init__(self):
        self.commands = []

    def robot_command(self, command):
        self.commands.append(command)
        return f"command-{len(self.commands)}"


class FakeGripperCameraParamClient:
    def __init__(self):
        self.requests = []

    def set_camera_params(self, request):
        self.requests.append(request)
        return "ok"


class FakeRobot:
    def __init__(self, client):
        self.client = client
        self.requested_service_names = []

    def ensure_client(self, service_name):
        self.requested_service_names.append(service_name)
        return self.client


def test_open_gripper_sends_spot_sdk_open_command(spot_gripper_module):
    module, calls = spot_gripper_module
    command_client = FakeRobotCommandClient()

    command_id = module.open_gripper(command_client)

    assert calls == ["build_open"]
    assert command_client.commands == ["open_command"]
    assert command_id == "command-1"


def test_close_gripper_sends_spot_sdk_close_command(spot_gripper_module):
    module, calls = spot_gripper_module
    command_client = FakeRobotCommandClient()

    command_id = module.close_gripper(command_client)

    assert calls == ["build_close"]
    assert command_client.commands == ["close_command"]
    assert command_id == "command-1"


def test_default_gripper_camera_resolution_is_1920x1080(spot_gripper_module):
    module, _calls = spot_gripper_module

    assert module.DEFAULT_GRIPPER_CAMERA_RESOLUTION == "1920x1080"
    assert module.gripper_camera_mode_from_resolution("1920x1080") == 14


def test_set_gripper_camera_resolution_uses_spot_sdk_param_service(spot_gripper_module):
    module, _calls = spot_gripper_module
    client = FakeGripperCameraParamClient()
    robot = FakeRobot(client)

    module.set_gripper_camera_resolution(robot, "1920x1080")

    assert robot.requested_service_names == ["gripper-camera-param"]
    assert len(client.requests) == 1
    assert client.requests[0].params.camera_mode == 14


def test_invalid_gripper_camera_resolution_raises(spot_gripper_module):
    module, _calls = spot_gripper_module

    with pytest.raises(ValueError, match="Unsupported gripper camera resolution"):
        module.gripper_camera_mode_from_resolution("2048x1080")
