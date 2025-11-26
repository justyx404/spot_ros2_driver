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

import math
import time

from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b, get_se2_a_tform_b
from bosdyn.client.math_helpers import SE2Pose, SE3Pose, Quat, quat_to_eulerZYX
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from geometry_msgs.msg import Twist
import rclpy
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from tf2_ros.buffer import Buffer

from spot_action.action import MoveRelativeXY


class SpotCommander:
    """
    This class is responsible for handling all robot movement commands.
    """

    def __init__(self, node: Node, command_client: RobotCommandClient, robot_state_client: RobotStateClient, 
                 odom_frame: str, tf_buffer: Buffer):
        self._node = node
        self._command_client = command_client
        self._robot_state_client = robot_state_client
        self._odom_frame = odom_frame
        self._tf_buffer = tf_buffer
    
    def move_relative_xy(self, goal_handle: ServerGoalHandle):
        """Execute the move to relative [x, y, yaw] action."""
        goal = goal_handle.request
        self._node.get_logger().info(f"Executing goal: x={goal.x}, y={goal.y}, theta={goal.yaw}")

        distance = math.sqrt(goal.x**2 + goal.y**2)
        max_vel = 1.0  # https://github.com/boston-dynamics/spot-sdk/blob/master/protos/bosdyn/api/spot/robot_command.proto#L66
        estimated_time = (distance / max_vel) + 5.0  # Add 5 second for safety margin

        try:
            transforms = self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
            
            if self._odom_frame == "lidar":
                # 1. Get the goal relative to the body (Input)
                body_tform_goal = SE2Pose(x=goal.x, y=goal.y, angle=goal.yaw)

                # 2. Get the current Lidar -> Body transform from ROS TF
                try:
                    tf_stamped = self._tf_buffer.lookup_transform(
                        "odom_lidar", "base_link", rclpy.time.Time()
                    )
                    lidar_tform_body = self._ros_tf_to_se2(tf_stamped)
                except Exception as e:
                    self._node.get_logger().error(f"Failed to lookup odom_lidar -> base_link: {e}")
                    goal_handle.abort()
                    return MoveRelativeXY.Result(success=False)

                # 3. Get the current Vision -> Body transform from Spot SDK
                vision_tform_body = get_se2_a_tform_b(transforms, VISION_FRAME_NAME, BODY_FRAME_NAME)

                # 4. Calculate Vision -> Lidar (Bridging the gap)
                #    vision_T_lidar = vision_T_body * (lidar_T_body)^-1
                body_tform_lidar = lidar_tform_body.inverse()
                vision_tform_lidar = vision_tform_body * body_tform_lidar

                # 5. Calculate Lidar -> Goal (Goal relative to the lidar frame)
                #    lidar_T_goal = lidar_T_body * body_T_goal
                lidar_tform_goal = lidar_tform_body * body_tform_goal

                # 6. Calculate Vision -> Goal (Final command pose)
                #    vision_T_goal = vision_T_lidar * lidar_T_goal
                final_pose_in_cmd_frame = vision_tform_lidar * lidar_tform_goal
                
                # We command in the VISION frame
                cmd_frame_name = VISION_FRAME_NAME

                self._node.get_logger().info(f"Lidar Mode: Goal in Vision Frame calculated as {final_pose_in_cmd_frame}")

            else:
                # Standard Logic: odom_frame is known by Spot (odom or vision)
                body_tform_goal = SE2Pose(x=goal.x, y=goal.y, angle=goal.yaw)
                odom_tform_body = get_se2_a_tform_b(transforms, self._odom_frame, BODY_FRAME_NAME)
                final_pose_in_cmd_frame = odom_tform_body * body_tform_goal
                cmd_frame_name = self._odom_frame

            command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=final_pose_in_cmd_frame.x,
                goal_y=final_pose_in_cmd_frame.y,
                goal_heading=final_pose_in_cmd_frame.angle,
                frame_name=cmd_frame_name,
            )

            cmd_id = self._command_client.robot_command(command, end_time_secs=time.time() + estimated_time)

            # feedback_msg = MoveRelativeXY.Feedback()
            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._node.get_logger().info("Goal canceled.")
                    self._command_client.robot_command(RobotCommandBuilder.stop_command())
                    return MoveRelativeXY.Result(success=False)

                feedback = self._command_client.robot_command_feedback(cmd_id)
                mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback

                if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
                    self._node.get_logger().error("Failed to reach the goal.")
                    goal_handle.abort()
                    return MoveRelativeXY.Result(success=False)

                # TODO: Add feedback publishing

                traj_feedback = mobility_feedback.se2_trajectory_feedback
                if (
                    traj_feedback.status == traj_feedback.STATUS_AT_GOAL
                    and traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED
                ):
                    self._node.get_logger().info("Arrived at the goal.")
                    goal_handle.succeed()
                    return MoveRelativeXY.Result(success=True)

                time.sleep(0.1)  # Check status at 10 Hz

        except (RpcError, ResponseError) as e:
            self._node.get_logger().error(f"Error during action execution: {e}")
            goal_handle.abort()
            return MoveRelativeXY.Result(success=False)

    def cmd_vel_callback(self, msg: Twist):
        """Convert a Twist message to a robot velocity command and send it."""
        v_x, v_y, v_rot = msg.linear.x, msg.linear.y, msg.angular.z
        command = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot)
        try:
            # Send the command to the robot
            self._command_client.robot_command(command, end_time_secs=time.time() + 0.5)
            self._node.get_logger().debug(f"Sent velocity command: v_x={v_x}, v_y={v_y}, v_rot={v_rot}")
        except (RpcError, ResponseError) as e:
            self._node.get_logger().error(f"Failed to send velocity command: {e}")

    def _ros_tf_to_se2(self, tf_stamped) -> SE2Pose:
        """Helper to convert a ROS TransformStamped to a Bosdyn SE3Pose using API objects."""
        q = Quat(
            w=tf_stamped.transform.rotation.w,
            x=tf_stamped.transform.rotation.x,
            y=tf_stamped.transform.rotation.y,
            z=tf_stamped.transform.rotation.z,
        )
        yaw, pitch, roll = quat_to_eulerZYX(q)

        return SE2Pose(
            x=tf_stamped.transform.translation.x,
            y=tf_stamped.transform.translation.y,
            angle=yaw
        )
