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

from bosdyn.client.math_helpers import SE3Pose
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class SpotTFPublisher:
    """
    This class is responsible for publishing all TF frames for the Spot driver.
    """

    def __init__(self, node: Node):
        self._node = node
        self._tf_broadcaster = TransformBroadcaster(self._node)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self._node)

    def publish_transform(self, tfrom: SE3Pose, header: str, child: str):
        """Publish the transform from ODOM to BODY frame."""
        t = TransformStamped()
        # TODO: sync with the robot's internal time
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = header
        t.child_frame_id = child
        t.transform.translation.x = tfrom.position.x
        t.transform.translation.y = tfrom.position.y
        t.transform.translation.z = tfrom.position.z
        t.transform.rotation.x = tfrom.rotation.x
        t.transform.rotation.y = tfrom.rotation.y
        t.transform.rotation.z = tfrom.rotation.z
        t.transform.rotation.w = tfrom.rotation.w
        self._tf_broadcaster.sendTransform(t)

    def publish_static_transform(self, tfrom: SE3Pose, header: str, child: str):
        """Publish the transform from ODOM to BODY frame."""
        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = header
        t.child_frame_id = child
        t.transform.translation.x = tfrom.position.x
        t.transform.translation.y = tfrom.position.y
        t.transform.translation.z = tfrom.position.z
        t.transform.rotation.x = tfrom.rotation.x
        t.transform.rotation.y = tfrom.rotation.y
        t.transform.rotation.z = tfrom.rotation.z
        t.transform.rotation.w = tfrom.rotation.w
        self._static_tf_broadcaster.sendTransform(t)

    def publish_static_transforms(self, transforms):
        """Publish a list of static transforms.

        Args:
            transforms: List of tuples (tfrom: SE3Pose, header: str, child: str)
        """
        tf_msgs = []
        now = self._node.get_clock().now().to_msg()
        for tfrom, header, child in transforms:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = header
            t.child_frame_id = child
            t.transform.translation.x = tfrom.position.x
            t.transform.translation.y = tfrom.position.y
            t.transform.translation.z = tfrom.position.z
            t.transform.rotation.x = tfrom.rotation.x
            t.transform.rotation.y = tfrom.rotation.y
            t.transform.rotation.z = tfrom.rotation.z
            t.transform.rotation.w = tfrom.rotation.w
            tf_msgs.append(t)
        
        self._static_tf_broadcaster.sendTransform(tf_msgs)
