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
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, get_a_tform_b
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.math_helpers import SE3Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage


class SpotImagePublisher:
    """
    This class is responsible for publishing all image data from Spot.
    """

    BODY_IMAGE_SOURCES = ("frontleft_fisheye_image", "frontright_fisheye_image")
    GRIPPER_IMAGE_SOURCE = "hand_color_image"

    SOURCE_PIXEL_FORMATS = {
        "frontleft_fisheye_image": image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8,
        "frontright_fisheye_image": image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8,
        "hand_color_image": image_pb2.Image.PIXEL_FORMAT_RGB_U8,
    }

    def __init__(
        self,
        node: Node,
        image_client: ImageClient | None = None,
        include_gripper_camera: bool = False,
        image_qos=10,
        jpeg_quality: int = 75,
    ):
        self._node = node
        self.image_qos = image_qos
        self.jpeg_quality = jpeg_quality
        self.sources = list(self.BODY_IMAGE_SOURCES)

        if include_gripper_camera:
            self._add_gripper_camera_source(image_client)

        self.pubs = {}
        for source in self.sources:
            name = self.topic_name_from_source(source)
            self.pubs[source] = {
                "compressed": self._node.create_publisher(
                    CompressedImage, f"camera/{name}/image/compressed", self.image_qos
                ),
                "info": self._node.create_publisher(CameraInfo, f"camera/{name}/camera_info", self.image_qos),
            }

    @classmethod
    def topic_name_from_source(cls, source: str) -> str:
        return source.replace("_image", "")

    @classmethod
    def is_static_source(cls, source: str) -> bool:
        return source != cls.GRIPPER_IMAGE_SOURCE

    def _add_gripper_camera_source(self, image_client: ImageClient | None):
        if self.GRIPPER_IMAGE_SOURCE in self.sources:
            return

        if image_client is None:
            self.sources.append(self.GRIPPER_IMAGE_SOURCE)
            self._node.get_logger().info("Enabling gripper camera publisher for hand_color_image.")
            return

        try:
            source_names = {source.name for source in image_client.list_image_sources()}
        except Exception as exc:
            self.sources.append(self.GRIPPER_IMAGE_SOURCE)
            self._node.get_logger().warn(
                f"Could not list Spot image sources; enabling hand_color_image based on arm detection: {exc}"
            )
            return

        if self.GRIPPER_IMAGE_SOURCE in source_names:
            self.sources.append(self.GRIPPER_IMAGE_SOURCE)
            self._node.get_logger().info("Detected hand_color_image source; enabling gripper camera publisher.")
        else:
            self._node.get_logger().warn(
                "Spot arm/gripper detected, but hand_color_image is not advertised by the image service. "
                "Gripper camera publisher disabled."
            )

    def _pixel_format_for_source(self, source: str):
        return self.SOURCE_PIXEL_FORMATS[source]

    def _active_sources(self, sources=None) -> list[str]:
        if sources is None:
            sources = self.sources
        return [source for source in sources if source in self.pubs]

    def _build_image_request(self, source: str):
        return build_image_request(
            source,
            quality_percent=self.jpeg_quality,
            pixel_format=self._pixel_format_for_source(source),
            image_format=image_pb2.Image.FORMAT_JPEG,
        )

    def get_camera_transform_from_body(self, image_client: ImageClient, source: str) -> SE3Pose:
        """
        Retrieves the transform from the robot's body frame to the camera frame.
        """
        body_tform_camera, _ = self.get_camera_transform_and_frame_from_body(image_client, source)
        return body_tform_camera

    def get_camera_transform_and_frame_from_body(self, image_client: ImageClient, source: str) -> tuple[SE3Pose, str]:
        """
        Retrieves the transform and frame name from the robot's body frame to the camera frame.
        """
        image_response = image_client.get_image([self._build_image_request(source)])[0]
        frame_name = image_response.shot.frame_name_image_sensor
        body_tform_camera = get_a_tform_b(image_response.shot.transforms_snapshot, BODY_FRAME_NAME, frame_name)
        return body_tform_camera, frame_name

    def publish_image_and_info(self, image_client: ImageClient, tf_publisher=None, sources=None):
        """Get images from the robot and publish them with camera info."""
        active_sources = self._active_sources(sources)
        if not active_sources:
            return

        requests = [self._build_image_request(source) for source in active_sources]

        try:
            image_responses = image_client.get_image(requests)
        except Exception as exc:
            self._node.get_logger().warn(f"Failed to get Spot images: {exc}")
            return

        for response in image_responses:
            source = response.source.name
            if source not in self.pubs:
                continue

            stamp = self._node.get_clock().now().to_msg()
            if tf_publisher is not None and not self.is_static_source(source):
                self._publish_dynamic_camera_tf(response, tf_publisher)

            self._publish_compressed_image(response, self.pubs[source]["compressed"], stamp)
            self._publish_camera_info(response, self.pubs[source]["info"], stamp)

    def _publish_dynamic_camera_tf(self, image_response, tf_publisher):
        frame_name = image_response.shot.frame_name_image_sensor
        try:
            body_tform_camera = get_a_tform_b(image_response.shot.transforms_snapshot, BODY_FRAME_NAME, frame_name)
        except Exception as exc:
            self._node.get_logger().warn(
                f"Failed to compute dynamic TF for camera source {image_response.source.name}: {exc}"
            )
            return

        if body_tform_camera is None:
            self._node.get_logger().warn(
                f"No body transform found for dynamic camera source {image_response.source.name}."
            )
            return

        tf_publisher.publish_transform(body_tform_camera, "base_link", frame_name)

    def _publish_camera_info(self, image_response, publisher, stamp=None):
        frame_id = image_response.shot.frame_name_image_sensor

        msg = CameraInfo()
        msg.header.stamp = stamp if stamp is not None else self._node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = image_response.source.rows
        msg.width = image_response.source.cols
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Assuming no distortion; replace with actual values if available
        fx = image_response.source.pinhole.intrinsics.focal_length.x
        fy = image_response.source.pinhole.intrinsics.focal_length.y
        cx = image_response.source.pinhole.intrinsics.principal_point.x
        cy = image_response.source.pinhole.intrinsics.principal_point.y
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        publisher.publish(msg)

    def _publish_compressed_image(self, image_response, publisher, stamp=None):
        image = image_response.shot.image
        if image.format != image_pb2.Image.FORMAT_JPEG:
            self._node.get_logger().warn(
                f"Skipping non-JPEG image from source {image_response.source.name}; requested FORMAT_JPEG."
            )
            return

        frame_id = image_response.shot.frame_name_image_sensor
        image_msg = CompressedImage()
        image_msg.header.stamp = stamp if stamp is not None else self._node.get_clock().now().to_msg()
        image_msg.header.frame_id = frame_id
        image_msg.format = "jpeg"
        image_msg.data = bytes(image.data)

        publisher.publish(image_msg)
