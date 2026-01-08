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

"""A minimal ROS 2 driver for Boston Dynamics Spot robot."""

import math
import threading
import time
from typing import Optional

import bosdyn.client
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.api.robot_state_pb2 import ImuState, RobotState
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    ODOM_FRAME_NAME,
    VISION_FRAME_NAME,
    get_a_tform_b,
    get_se2_a_tform_b,
)
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import Error as LeaseError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.math_helpers import SE2Pose, SE3Pose, SE3Velocity, Quat
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient, RobotStateStreamingClient
from bosdyn.client.world_object import WorldObjectClient, world_object_pb2

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.timer import Timer
from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu

from spot_action.action import MoveRelativeXY
from spot_driver.spot_commander import SpotCommander
from spot_driver.spot_image import SpotImagePublisher
from spot_driver.spot_streams import SpotStreamer
from spot_driver.spot_tf import SpotTFPublisher


class SpotROS2Driver(Node):
    """A minimal ROS 2 driver for Boston Dynamics Spot robot."""

    def __init__(self):
        """Initialize the Spot ROS 2 driver node."""
        super().__init__("spot_driver_node")

        self.declare_parameter("hostname", "192.168.80.3")
        hostname = self.get_parameter("hostname").get_parameter_value().string_value
        self.declare_parameter("username", "user")
        username = self.get_parameter("username").get_parameter_value().string_value
        self.declare_parameter("password", "password")
        password = self.get_parameter("password").get_parameter_value().string_value

        # load the user-defined odometry frame
        # ODOM_FRAME_NAME -> spot odometry using kinematic -> /odom_kinematic
        # VISION_FRAME_NAME -> spot odometry using kinematic -> /odom_vision
        # odom_lidar -> odometry from an external LiDAR
        self.declare_parameter("odometry_frame", "kinematic")
        self.odom_choice = self.get_parameter("odometry_frame").get_parameter_value().string_value
        if self.odom_choice == "kinematic":
            self.odom_frame = ODOM_FRAME_NAME
        elif self.odom_choice == "vision":
            self.odom_frame = VISION_FRAME_NAME
        elif self.odom_choice == "lidar":
            self.odom_frame = "lidar"
        else:
            self.get_logger().error(f'Invalid odometry frame: {self.odom_choice}. Using default "kinematic".')
            self.odom_choice = "kinematic"
            self.odom_frame = ODOM_FRAME_NAME

        # if the user has a streaming client license, use it to get IMU data at 333Hz
        self.declare_parameter("use_streaming_client", False)
        self.use_streaming_client = self.get_parameter("use_streaming_client").get_parameter_value().bool_value

        self._shutdown_event = threading.Event()

        self.robot: Optional[bosdyn.client.robot.Robot] = None
        self.lease_keep_alive: Optional[LeaseKeepAlive] = None
        self.estop_keep_alive: Optional[EstopKeepAlive] = None
        self.robot_state_client: Optional[RobotStateClient] = None
        self.command_client: Optional[RobotCommandClient] = None
        self.world_object_client: Optional[WorldObjectClient] = None

        try:
            # Robot initialization
            sdk = bosdyn.client.create_standard_sdk("SpotROS2DriverClient")

            if self.use_streaming_client:
                self.get_logger().info("Using licensed streaming client for high-frequency IMU data.")
                sdk.register_service_client(RobotStateStreamingClient)
            else:
                self.get_logger().info(
                    "Streaming client is disabled by default. In order to use it, you need to purchase an additional license from Boston Dynamics."
                )

            self.robot = sdk.create_robot(hostname)

            # NOTE: username and password are manually provided
            self.robot.authenticate(username, password)

            self.robot.sync_with_directory()
            self.robot.time_sync.wait_for_sync()

            # NOTE: Not sure if this is necessary
            assert not self.robot.is_estopped(), (
                "Robot is estopped. Please use an external E-Stop client, "
                "such as the estop SDK example, to configure E-Stop."
            )

            self.get_logger().info("Successfully authenticated and connected to the robot.")

            # Create clients
            self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
            if self.use_streaming_client:
                self.robot_state_streaming_client = self.robot.ensure_client(
                    RobotStateStreamingClient.default_service_name
                )
            self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            self.world_object_client = self.robot.ensure_client(WorldObjectClient.default_service_name)
            self.image_client = self.robot.ensure_client(ImageClient.default_service_name)
            self.get_logger().info("Robot clients created.")

            # Lease management
            lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
            self.lease_keep_alive = LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True)
            self.get_logger().info("Acquired lease.")

            # Acquire E-Stop
            estop_client = self.robot.ensure_client(EstopClient.default_service_name)
            estop_endpoint = EstopEndpoint(estop_client, "SpotROS2DriverEStop", 10.0)
            estop_endpoint.force_simple_setup()
            self.estop_keep_alive = EstopKeepAlive(estop_endpoint)
            self.get_logger().info("Acquired E-Stop.")

            time.sleep(5.0)

            # Power on and Stand Robot
            self.robot.power_on(timeout_sec=20)
            assert self.robot.is_powered_on(), "Robot power on failed."
            self.get_logger().info("Robot powered on.")

            blocking_stand(self.command_client, timeout_sec=10)
            self.get_logger().info("Robot standing.")

        except (RpcError, ResponseError, LeaseError) as e:
            self.get_logger().error(f"Failed to connect to the robot: {e}")
            raise

        # ROS 2 publishers and subscribers
        self.tf_publisher = SpotTFPublisher(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.image_component = SpotImagePublisher(self)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        # self.fiducial_pose_publisher = self.create_publisher(PoseStamped, "fiducial_pose", 10)
        self.fiducial_pose_array_publisher = self.create_publisher(PoseArray, "fiducial_poses", 10)
        self.fiducial_cache = {}

        # Create commander
        self.commander = SpotCommander(self, self.command_client, self.robot_state_client, self.odom_frame, self.tf_buffer)
        self.cmd_vel_subscriber = self.create_subscription(Twist, "cmd_vel", self.commander.cmd_vel_callback, 10)

        robot_state_pub_group = MutuallyExclusiveCallbackGroup()
        self.robot_state_publisher = self.create_timer(
            0.1, self.publish_robot_state, callback_group=robot_state_pub_group
        )

        # broadcast camera frame as static TF
        static_transforms = []
        
        cam_tform_body = self.image_component.get_camera_transform_from_body(self.image_client, "frontleft_fisheye_image")
        static_transforms.append((cam_tform_body, "base_link", "frontleft_fisheye"))
        self.get_logger().info("Collected frontleft_fisheye TF.")

        cam_tform_body = self.image_component.get_camera_transform_from_body(self.image_client, "frontright_fisheye_image")
        static_transforms.append((cam_tform_body, "base_link", "frontright_fisheye"))
        self.get_logger().info("Collected frontright_fisheye TF.")
        
        self.tf_publisher.publish_static_transforms(static_transforms)
        self.get_logger().info("Published all static TFs.")

        # Action server initialization
        action_group = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            self,
            MoveRelativeXY,
            "move_relative_xy",
            execute_callback=self.commander.move_relative_xy,
            callback_group=action_group,
        )

        if self.use_streaming_client:
            self.streamer = SpotStreamer(self, self._shutdown_event)
            self.streamer.start(self.robot_state_streaming_client)

    def publish_robot_state(self):
        """Periodic publish robot data (if connected)."""
        # ---------------------------------------------------------------------
        # 1. FIDUCIAL (APRILTAG) PROCESSING
        # ---------------------------------------------------------------------
        fiducials = self.world_object_client.list_world_objects(
            [world_object_pb2.WORLD_OBJECT_APRILTAG]
        ).world_objects

        target_frame_id = f"odom_{self.odom_choice}" if self.odom_choice != "lidar" else "odom_lidar"

        for fiducial in fiducials:
            raw_name = fiducial.name 
            tag_id_str = raw_name.split('_')[-1]
            tag_id = int(tag_id_str)
            fiducial_frame = f"filtered_fiducial_{tag_id}"

            final_se3_pose = None
            if self.odom_choice != "lidar":
                # get transform: Tag -> Odom (computed by Spot internally)
                final_se3_pose = get_a_tform_b(
                    fiducial.transforms_snapshot, 
                    self.odom_frame,
                    fiducial_frame
                )
            else:
                tform_body_fiducial = get_a_tform_b(
                    fiducial.transforms_snapshot, 
                    BODY_FRAME_NAME, 
                    fiducial_frame
                )

                try:
                    tf_odom_base = self.tf_buffer.lookup_transform(
                        "odom_lidar",
                        "base_link",
                        rclpy.time.Time(),
                    )

                    tform_odom_base = SE3Pose(
                        tf_odom_base.transform.translation.x,
                        tf_odom_base.transform.translation.y,
                        tf_odom_base.transform.translation.z,
                        Quat(
                            tf_odom_base.transform.rotation.w,
                            tf_odom_base.transform.rotation.x,
                            tf_odom_base.transform.rotation.y,
                            tf_odom_base.transform.rotation.z,
                        )
                    )

                    final_se3_pose = tform_odom_base * tform_body_fiducial
                except Exception as e:
                    self.get_logger().warn(f"Could not look up transform from 'odom_lidar' to 'base_link': {e}")

            if final_se3_pose:
                ros_pose = self._bosdyn_pose_to_ros_pose(final_se3_pose)
                self.fiducial_cache[tag_id] = ros_pose
                # Broadcast as static TF
                self.tf_publisher.publish_static_transform(final_se3_pose, target_frame_id, f"fiducial_{tag_id}")

        # Publish the cached fiducials
        if self.fiducial_cache:
            pose_array_msg = PoseArray()
            pose_array_msg.header.stamp = self.get_clock().now().to_msg()
            pose_array_msg.header.frame_id = target_frame_id
            
            for tag_id in sorted(self.fiducial_cache.keys()):
                pose_array_msg.poses.append(self.fiducial_cache[tag_id])

            self.fiducial_pose_array_publisher.publish(pose_array_msg)

        # ---------------------------------------------------------------------
        # 2. IMAGE PUBLISHING
        # ---------------------------------------------------------------------
        self.image_component.publish_image_and_info(self.image_client)


        # ---------------------------------------------------------------------
        # 3. Odom PUBLISHING
        # ---------------------------------------------------------------------
        if self.odom_choice != "lidar":
            robot_state: RobotState = self.robot_state_client.get_robot_state()
            odom_tfrom_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot, self.odom_frame, BODY_FRAME_NAME)
            odom_vel_of_body = robot_state.kinematic_state.velocity_of_body_in_odom
            self.tf_publisher.publish_transform(odom_tfrom_body, f"odom_{self.odom_choice}", "base_link")
            self.publish_odometry(odom_tfrom_body, odom_vel_of_body, f"odom_{self.odom_choice}", "base_link")

    def publish_odometry(self, odom_tfrom_body: SE3Pose, odom_vel_of_body: SE3Velocity, header: str, child: str):
        """Publish the odometry data."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = header
        odom_msg.child_frame_id = child

        odom_msg.pose.pose.position.x = odom_tfrom_body.position.x
        odom_msg.pose.pose.position.y = odom_tfrom_body.position.y
        odom_msg.pose.pose.position.z = odom_tfrom_body.position.z
        odom_msg.pose.pose.orientation.x = odom_tfrom_body.rotation.x
        odom_msg.pose.pose.orientation.y = odom_tfrom_body.rotation.y
        odom_msg.pose.pose.orientation.z = odom_tfrom_body.rotation.z
        odom_msg.pose.pose.orientation.w = odom_tfrom_body.rotation.w

        self.odom_publisher.publish(odom_msg)

    def _bosdyn_pose_to_ros_pose(self, bosdyn_pose: SE3Pose) -> Pose:
        """Helper to convert Bosdyn SE3Pose to ROS geometry_msgs/Pose."""
        ros_pose = Pose()
        ros_pose.position.x = bosdyn_pose.position.x
        ros_pose.position.y = bosdyn_pose.position.y
        ros_pose.position.z = bosdyn_pose.position.z
        ros_pose.orientation.x = bosdyn_pose.rotation.x
        ros_pose.orientation.y = bosdyn_pose.rotation.y
        ros_pose.orientation.z = bosdyn_pose.rotation.z
        ros_pose.orientation.w = bosdyn_pose.rotation.w

        return ros_pose

    def stop_thread(self):
        self._shutdown_event.set()

    def shutdown_robot(self):
        """Shutdown the driver and release resources."""
        print("Shutting down the robot...")
        if self.robot and self.robot.is_powered_on():
            self.robot.power_off(cut_immediately=False, timeout_sec=20)
            print("Robot powered off.")
        if self.estop_keep_alive:
            self.estop_keep_alive.shutdown()
            print("E-Stop released.")
        if self.lease_keep_alive:
            self.lease_keep_alive.shutdown()
            print("Lease released.")


def main(args=None):
    """Initialize and run the Spot ROS 2 driver node."""
    rclpy.init(args=args)
    spot_driver_node = None
    executor = None
    try:
        spot_driver_node = SpotROS2Driver()
        executor = MultiThreadedExecutor()
        executor.add_node(spot_driver_node)
        executor.spin()
    except (KeyboardInterrupt, RpcError, ResponseError, LeaseError) as e:
        if isinstance(e, KeyboardInterrupt):
            print("Shutting down the Robot due to KeyboardInterrupt.")
        else:
            print(f"Shutting down the Robot due to Spot-SDK error: {e}")
    finally:
        spot_driver_node.stop_thread()
        spot_driver_node.shutdown_robot()
        spot_driver_node.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
