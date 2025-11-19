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
import bosdyn.client.util
import rclpy
from bosdyn.api import image_pb2
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
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.lease import Error as LeaseError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.math_helpers import SE2Pose, SE3Pose, SE3Velocity, Quat
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient, RobotStateStreamingClient
from bosdyn.client.world_object import WorldObjectClient, world_object_pb2
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.timer import Timer
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener

from spot_action.action import MoveRelativeXY
from spot_driver.spot_image import SpotImagePublisher
from spot_driver.spot_streams import SpotStreamer
from spot_driver.spot_tf import SpotTFPublisher
# from spot_srvs.srv import GetTransform


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
            # TODO: need to register the lidar odometry frame to the spot TF tree
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

            # NOTE: Must have both BOSDYN_CLIENT_USERNAME and BOSDYN_CLIENT_PASSWORD environment variables set
            # bosdyn.client.util.authenticate(self.robot)
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

            time.sleep(3.0)

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

        self.cmd_vel_subscriber = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        self.image_component = SpotImagePublisher(self)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.fiducial_pose_publisher = self.create_publisher(PoseStamped, "fiducial_pose", 10)

        # NOTE: DDS can only suport timer periond 0.5s or higher, need to use Zenoh 
        # middleware to achieve 0.1s
        robot_state_pub_group = MutuallyExclusiveCallbackGroup()
        self.robot_state_publisher = self.create_timer(
            0.1, self.publish_robot_state, callback_group=robot_state_pub_group
        )

        # broadcast camera frame as static TF
        request = build_image_request(
            "frontleft_fisheye_image",
            pixel_format=image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8,
            image_format=image_pb2.Image.FORMAT_RAW,
        )
        image_response = self.image_client.get_image([request])
        cam_tform_body = get_a_tform_b(
            image_response[0].shot.transforms_snapshot, BODY_FRAME_NAME, image_response[0].shot.frame_name_image_sensor
        )
        self.tf_publisher.publish_static_transform(cam_tform_body, "base_link", "frontleft_fisheye")
        self.get_logger().info(f"Published {image_response[0].shot.frame_name_image_sensor} TF.")

        # Action server initialization
        action_group = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            self,
            MoveRelativeXY,
            "move_relative_xy",
            execute_callback=self.move_relative_xy,
            callback_group=action_group,
        )

        if self.use_streaming_client:
            self.streamer = SpotStreamer(self, self._shutdown_event)
            self.streamer.start(self.robot_state_streaming_client)

    def move_relative_xy(self, goal_handle: ServerGoalHandle):
        """Execute the move to relative [x, y, yaw] action."""
        goal = goal_handle.request
        self.get_logger().info(f"Executing goal: x={goal.x}, y={goal.y}, theta={goal.yaw}")

        distance = math.sqrt(goal.x**2 + goal.y**2)
        max_vel = 1.0  # https://github.com/boston-dynamics/spot-sdk/blob/master/protos/bosdyn/api/spot/robot_command.proto#L66
        estimated_time = (distance / max_vel) + 5.0  # Add 5 second for safety margin

        try:
            transforms = self.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

            # convert the goal pose from robot body frame to odom frame
            body_tform_goal = SE2Pose(x=goal.x, y=goal.y, angle=goal.yaw)
            odom_tform_body = get_se2_a_tform_b(transforms, self.odom_frame, BODY_FRAME_NAME)
            odom_tfrom_goal = odom_tform_body * body_tform_goal

            command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=odom_tfrom_goal.x,
                goal_y=odom_tfrom_goal.y,
                goal_heading=odom_tfrom_goal.angle,
                frame_name=self.odom_frame,
            )

            cmd_id = self.command_client.robot_command(command, end_time_secs=time.time() + estimated_time)

            # feedback_msg = MoveRelativeXY.Feedback()
            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Goal canceled.")
                    self.command_client.robot_command(RobotCommandBuilder.stop_command())
                    return MoveRelativeXY.Result(success=False)

                feedback = self.command_client.robot_command_feedback(cmd_id)
                mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback

                if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
                    self.get_logger().error("Failed to reach the goal.")
                    goal_handle.abort()
                    return MoveRelativeXY.Result(success=False)

                # TODO: Add feedback publishing

                traj_feedback = mobility_feedback.se2_trajectory_feedback
                if (
                    traj_feedback.status == traj_feedback.STATUS_AT_GOAL
                    and traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED
                ):
                    self.get_logger().info("Arrived at the goal.")
                    goal_handle.succeed()
                    return MoveRelativeXY.Result(success=True)

                time.sleep(0.1)  # Check status at 10 Hz

        except (RpcError, ResponseError) as e:
            self.get_logger().error(f"Error during action execution: {e}")
            goal_handle.abort()
            return MoveRelativeXY.Result(success=False)

    def publish_robot_state(self):
        """Periodic publish robot data (if connected)."""
        # detect and publish fiducial transform
        fiducials = self.world_object_client.list_world_objects([world_object_pb2.WORLD_OBJECT_APRILTAG]).world_objects
        if fiducials:
            fiducial = fiducials[0]
            if self.odom_choice != "lidar":
                tform_fiducial_odom = get_a_tform_b(fiducial.transforms_snapshot, "filtered_fiducial_200", self.odom_frame)
                self.tf_publisher.publish_static_transform(tform_fiducial_odom, "filtered_fiducial_200", f"odom_{self.odom_choice}")
            else:
                tform_base_fiducial = get_a_tform_b(fiducial.transforms_snapshot, BODY_FRAME_NAME, "filtered_fiducial_200")
                # Listen to the TF broadcaster for: odom_lidar -> base_link
                try:
                    tf_odom_base = self.tf_buffer.lookup_transform(
                        "odom_lidar",
                        "base_link",
                        rclpy.time.Time(),
                    )
                    quat_tmp = Quat(
                        tf_odom_base.transform.rotation.w,
                        tf_odom_base.transform.rotation.x,
                        tf_odom_base.transform.rotation.y,
                        tf_odom_base.transform.rotation.z,
                    )
                    tform_odom_base = SE3Pose(
                        tf_odom_base.transform.translation.x,
                        tf_odom_base.transform.translation.y,
                        tf_odom_base.transform.translation.z,
                        quat_tmp
                    )
                    tform_odom_fiducial = tform_odom_base * tform_base_fiducial
                    # The transform is no longer inversed
                    # tform_fiducial_odom = tform_odom_fiducial.inverse()

                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "odom_lidar" # The pose is of fiducial in the odom_lidar frame
                    pose_msg.pose.position.x = tform_odom_fiducial.x
                    pose_msg.pose.position.y = tform_odom_fiducial.y
                    pose_msg.pose.position.z = tform_odom_fiducial.z
                    pose_msg.pose.orientation.x = tform_odom_fiducial.rotation.x
                    pose_msg.pose.orientation.y = tform_odom_fiducial.rotation.y
                    pose_msg.pose.orientation.z = tform_odom_fiducial.rotation.z
                    pose_msg.pose.orientation.w = tform_odom_fiducial.rotation.w
                    self.fiducial_pose_publisher.publish(pose_msg)
                except Exception as e:
                    self.get_logger().warn(f"Could not look up transform from 'odom_lidar' to 'base_link': {e}")

        # publish camera images
        self.image_component.publish_image_and_info(self.image_client)

        # TODO: if user select LiDAR odometry (which has to be provided externally), register the this coordinate frame to the spot TF tree
        # other publish odom from the robot internal system
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

    def cmd_vel_callback(self, msg: Twist):
        """Convert a Twist message to a robot velocity command and send it."""
        v_x, v_y, v_rot = msg.linear.x, msg.linear.y, msg.angular.z
        command = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot)
        try:
            # Send the command to the robot
            self.command_client.robot_command(command, end_time_secs=time.time() + 0.5)
            self.get_logger().debug(f"Sent velocity command: v_x={v_x}, v_y={v_y}, v_rot={v_rot}")
        except (RpcError, ResponseError) as e:
            self.get_logger().error(f"Failed to send velocity command: {e}")

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
