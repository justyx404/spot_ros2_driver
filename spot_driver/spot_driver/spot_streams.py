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

import threading

from bosdyn.api.robot_state_pb2 import ImuState
from bosdyn.client.robot_state import RobotStateStreamingClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Imu


class SpotStreamer:
    """
    This class is responsible for handling high-frequency data streams from Spot.
    """

    def __init__(self, node: Node, shutdown_event: threading.Event):
        self._node = node
        self._shutdown_event = shutdown_event
        self._robot_state_stream = None
        self._stream_lock = threading.Lock()
        self._robot_state_stream_thread = None

        # Publishers
        self._imu_publisher = self._node.create_publisher(Imu, "imu", 10)

        # Timers
        robot_state_stream_group = MutuallyExclusiveCallbackGroup()
        self._robot_state_stream_publisher = self._node.create_timer(
            0.01, self._stream_robot_state, callback_group=robot_state_stream_group
        )

    def start(self, streaming_client: RobotStateStreamingClient):
        self._robot_state_stream_thread = threading.Thread(
            target=self._handle_state_streaming, args=[streaming_client], daemon=True
        )
        self._robot_state_stream_thread.start()

    def _handle_state_streaming(self, streaming_client: RobotStateStreamingClient):
        """Stream robot state from the robot at 333Hz"""
        try:
            robot_state_stream = streaming_client.get_robot_state_stream()
            self._node.get_logger().info("Started robot state streaming...")
            for robot_state in robot_state_stream:
                if self._shutdown_event.is_set():
                    break

                if robot_state.inertial_state and robot_state.inertial_state.packets:
                    with self._stream_lock:
                        self._robot_state_stream = robot_state
        except Exception as e:
            self._node.get_logger().error(f"Robot state streaming error: {e}")

    def _stream_robot_state(self):
        """Publish the latest robot state at 100Hz."""
        if self._robot_state_stream is None:
            return

        with self._stream_lock:
            imu_state = self._robot_state_stream.inertial_state
            self._publish_imu(imu_state)

    def _publish_imu(self, imu_state: ImuState):
        """Publish the IMU data."""
        packet = imu_state.packets[-1]

        imu_msg = Imu()
        imu_msg.header.stamp = self._node.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"

        imu_msg.orientation.x = packet.odom_rot_link.x
        imu_msg.orientation.y = packet.odom_rot_link.y
        imu_msg.orientation.z = packet.odom_rot_link.z
        imu_msg.orientation.w = packet.odom_rot_link.w

        imu_msg.angular_velocity.x = packet.angular_velocity_rt_odom_in_link_frame.x
        imu_msg.angular_velocity.y = packet.angular_velocity_rt_odom_in_link_frame.y
        imu_msg.angular_velocity.z = packet.angular_velocity_rt_odom_in_link_frame.z

        imu_msg.linear_acceleration.x = packet.acceleration_rt_odom_in_link_frame.x
        imu_msg.linear_acceleration.y = packet.acceleration_rt_odom_in_link_frame.y
        imu_msg.linear_acceleration.z = packet.acceleration_rt_odom_in_link_frame.z

        self._imu_publisher.publish(imu_msg)
