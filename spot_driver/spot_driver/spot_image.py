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

import numpy as np
import cv2

from bosdyn.api import image_pb2
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, get_a_tform_b # Added imports
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.math_helpers import SE3Pose # Added import
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class SpotImagePublisher:
    """
    This class is responsible for publishing all image data from Spot.
    """

    def __init__(self, node: Node):
        self._node = node
        self.sources = ["frontleft_fisheye_image", "frontright_fisheye_image"]
        self.pubs = {}
        for source in self.sources:
            name = source.replace("_image", "")
            self.pubs[source] = {
                "image": self._node.create_publisher(Image, f"camera/{name}/image", 10),
                "info": self._node.create_publisher(CameraInfo, f"camera/{name}/camera_info", 10)
            }

    def get_camera_transform_from_body(self, image_client: ImageClient, source: str) -> SE3Pose:
        """
        Retrieves the static transform from the robot's body frame to the camera frame.
        """
        request = build_image_request(
            source,
            pixel_format=image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8,
            image_format=image_pb2.Image.FORMAT_RAW,
        )
        image_response = image_client.get_image([request])
        # Assuming the first image response contains the necessary transform
        cam_tform_body = get_a_tform_b(
            image_response[0].shot.transforms_snapshot, BODY_FRAME_NAME, image_response[0].shot.frame_name_image_sensor
        )
        return cam_tform_body

    def publish_image_and_info(self, image_client: ImageClient):
        """Get an image from the robot and publish it with its camera info."""
        requests = []
        for source in self.pubs.keys():
            requests.append(build_image_request(
                source,
                pixel_format=image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8,
                image_format=image_pb2.Image.FORMAT_JPEG,
            ))
        
        image_responses = image_client.get_image(requests)
        
        for response in image_responses:
            source = response.source.name
            if source in self.pubs:
                self._publish_image(response, self.pubs[source]["image"])
                self._publish_camera_info(response, self.pubs[source]["info"])

    def _publish_camera_info(self, image_response, publisher):
        frame_id = image_response.shot.frame_name_image_sensor

        msg = CameraInfo()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = image_response.source.rows
        msg.width = image_response.source.cols
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Assuming no distortion; replace with actual values if available
        fx = image_response.source.pinhole.intrinsics.focal_length.x
        fy = image_response.source.pinhole.intrinsics.focal_length.y
        cx = image_response.source.pinhole.intrinsics.principal_point.x
        cy = image_response.source.pinhole.intrinsics.principal_point.y
        msg.k = [fx,  0.0, cx,
                 0.0, fy,  cy,
                 0.0, 0.0, 1.0]

        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
    
        msg.p = [fx,  0.0, cx,  0.0,
                 0.0, fy,  cy,  0.0,
                 0.0, 0.0, 1.0, 0.0]
        
        publisher.publish(msg)

    def _publish_image(self, image_response, publisher):
        """
        Converts a Spot SDK GREYSCALE_U8 image_response to a ROS 2 Image message and publishes it.
        """
        image = image_response.shot.image
        frame_id = image_response.shot.frame_name_image_sensor

        image_msg = Image()
        image_msg.header.stamp = self._node.get_clock().now().to_msg()
        image_msg.header.frame_id = frame_id
        
        np_arr = np.frombuffer(image.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        image_msg.height = cv_image.shape[0]
        image_msg.width = cv_image.shape[1]
        image_msg.encoding = "mono8" 
        image_msg.step = cv_image.shape[1]  # Width * 1 byte per pixel
        image_msg.data = cv_image.tobytes()

        # Publish the message
        publisher.publish(image_msg)