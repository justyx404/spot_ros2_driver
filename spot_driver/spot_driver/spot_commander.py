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
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, get_se2_a_tform_b
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from geometry_msgs.msg import Twist
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from spot_action.action import MoveRelativeXY


class SpotCommander:
    """
    This class is responsible for handling all robot movement commands.
    """

    def __init__(self, node: Node, command_client: RobotCommandClient, robot_state_client: RobotStateClient, odom_frame: str):
        self._node = node
        self._command_client = command_client
        self._robot_state_client = robot_state_client
        self._odom_frame = odom_frame
    
    def move_relative_xy(self, goal_handle: ServerGoalHandle):
        """Execute the move to relative [x, y, yaw] action."""
        goal = goal_handle.request
        self._node.get_logger().info(f"Executing goal: x={goal.x}, y={goal.y}, theta={goal.yaw}")

        distance = math.sqrt(goal.x**2 + goal.y**2)
        max_vel = 1.0  # https://github.com/boston-dynamics/spot-sdk/blob/master/protos/bosdyn/api/spot/robot_command.proto#L66
        estimated_time = (distance / max_vel) + 5.0  # Add 5 second for safety margin

        try:
            transforms = self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

            # convert the goal pose from robot body frame to odom frame
            body_tform_goal = SE2Pose(x=goal.x, y=goal.y, angle=goal.yaw)
            odom_tform_body = get_se2_a_tform_b(transforms, self._odom_frame, BODY_FRAME_NAME)
            odom_tfrom_goal = odom_tform_body * body_tform_goal

            command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=odom_tfrom_goal.x,
                goal_y=odom_tfrom_goal.y,
                goal_heading=odom_tfrom_goal.angle,
                frame_name=self._odom_frame,
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
