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

from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class SpotImagePublisher:
    """
    This class is responsible for publishing all image data from Spot.
    """

    def __init__(self, node: Node):
        self._node = node
        self._image_publisher = self._node.create_publisher(Image, "camera/image_raw", 10)
        self._cam_info_publisher = self._node.create_publisher(CameraInfo, "camera/camera_info", 10)

    def publish_image_and_info(self, image_client: ImageClient):
        """Get an image from the robot and publish it with its camera info."""
        request = build_image_request(
            "frontleft_fisheye_image",
            pixel_format=image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8,
            image_format=image_pb2.Image.FORMAT_RAW,
        )
        image_response = image_client.get_image([request])
        self._publish_image(image_response[0])
        self._publish_camera_info(image_response[0])

    def _publish_camera_info(self, image_response):
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
        
        self._cam_info_publisher.publish(msg)

    def _publish_image(self, image_response):
        """
        Converts a Spot SDK GREYSCALE_U8 image_response to a ROS 2 Image message and publishes it.
        """
        image = image_response.shot.image
        frame_id = image_response.shot.frame_name_image_sensor

        # Create the ROS 2 Image message
        image_msg = Image()
        image_msg.header.stamp = self._node.get_clock().now().to_msg()
        image_msg.header.frame_id = frame_id
        image_msg.height = image.rows
        image_msg.width = image.cols
        image_msg.encoding = "mono8"
        image_msg.step = image.cols
        image_msg.is_bigendian = False
        image_msg.data = image.data

        # Publish the message
        self._image_publisher.publish(image_msg)
