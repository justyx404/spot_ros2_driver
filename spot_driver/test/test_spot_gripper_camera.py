import importlib
import sys
import types
from pathlib import Path

import pytest


@pytest.fixture()
def gripper_camera_module(monkeypatch):
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

    gripper_camera_param_pb2 = types.ModuleType("bosdyn.api.gripper_camera_param_pb2")
    gripper_camera_param_pb2.GripperCameraParams = GripperCameraParams
    gripper_camera_param_pb2.GripperCameraParamRequest = GripperCameraParamRequest

    class GripperCameraParamClient:
        default_service_name = "gripper-camera-param"

    bosdyn = types.ModuleType("bosdyn")
    bosdyn_api = types.ModuleType("bosdyn.api")
    bosdyn_api.gripper_camera_param_pb2 = gripper_camera_param_pb2
    bosdyn_client = types.ModuleType("bosdyn.client")
    bosdyn_client_gripper = types.ModuleType("bosdyn.client.gripper_camera_param")
    bosdyn_client_gripper.GripperCameraParamClient = GripperCameraParamClient

    modules = {
        "bosdyn": bosdyn,
        "bosdyn.api": bosdyn_api,
        "bosdyn.api.gripper_camera_param_pb2": gripper_camera_param_pb2,
        "bosdyn.client": bosdyn_client,
        "bosdyn.client.gripper_camera_param": bosdyn_client_gripper,
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    package_root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(package_root))
    sys.modules.pop("spot_driver.spot_gripper_camera", None)
    return importlib.import_module("spot_driver.spot_gripper_camera")


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


def test_default_gripper_camera_resolution_is_1920x1080(gripper_camera_module):
    assert gripper_camera_module.DEFAULT_GRIPPER_CAMERA_RESOLUTION == "1920x1080"
    assert gripper_camera_module.gripper_camera_mode_from_resolution("1920x1080") == 14


def test_set_gripper_camera_resolution_uses_spot_sdk_param_service(gripper_camera_module):
    client = FakeGripperCameraParamClient()
    robot = FakeRobot(client)

    gripper_camera_module.set_gripper_camera_resolution(robot, "1920x1080")

    assert robot.requested_service_names == ["gripper-camera-param"]
    assert len(client.requests) == 1
    assert client.requests[0].params.camera_mode == 14


def test_invalid_gripper_camera_resolution_raises(gripper_camera_module):
    with pytest.raises(ValueError, match="Unsupported gripper camera resolution"):
        gripper_camera_module.gripper_camera_mode_from_resolution("2048x1080")
