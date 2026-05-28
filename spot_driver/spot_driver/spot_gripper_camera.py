# Copyright 2025 Yixiang Gao
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from bosdyn.api import gripper_camera_param_pb2
from bosdyn.client.gripper_camera_param import GripperCameraParamClient

DEFAULT_GRIPPER_CAMERA_RESOLUTION = "1920x1080"

GRIPPER_CAMERA_RESOLUTION_MODES = {
    "640x480": gripper_camera_param_pb2.GripperCameraParams.MODE_640_480,
    "1280x720": gripper_camera_param_pb2.GripperCameraParams.MODE_1280_720,
    "1920x1080": gripper_camera_param_pb2.GripperCameraParams.MODE_1920_1080,
    "3840x2160": gripper_camera_param_pb2.GripperCameraParams.MODE_3840_2160,
    "4096x2160": gripper_camera_param_pb2.GripperCameraParams.MODE_4096_2160,
    "4208x3120": gripper_camera_param_pb2.GripperCameraParams.MODE_4208_3120,
}


def gripper_camera_mode_from_resolution(resolution: str) -> int:
    """Return the Spot SDK gripper camera mode for a resolution string."""
    try:
        return GRIPPER_CAMERA_RESOLUTION_MODES[resolution]
    except KeyError as exc:
        supported = ", ".join(sorted(GRIPPER_CAMERA_RESOLUTION_MODES))
        raise ValueError(f"Unsupported gripper camera resolution: {resolution}. Supported: {supported}") from exc


def set_gripper_camera_resolution(robot, resolution: str = DEFAULT_GRIPPER_CAMERA_RESOLUTION):
    """Set the Spot gripper camera resolution using the Spot SDK param service."""
    camera_mode = gripper_camera_mode_from_resolution(resolution)
    gripper_camera_param_client = robot.ensure_client(GripperCameraParamClient.default_service_name)
    request = gripper_camera_param_pb2.GripperCameraParamRequest(
        params=gripper_camera_param_pb2.GripperCameraParams(camera_mode=camera_mode)
    )
    return gripper_camera_param_client.set_camera_params(request)
