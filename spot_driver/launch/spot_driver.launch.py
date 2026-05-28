"""Launch file for the Spot ROS2 Driver node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for the Spot ROS2 driver."""
    # Declare launch arguments
    declare_hostname_arg = DeclareLaunchArgument(
        "hostname", default_value="192.168.80.3", description="IP address of the Spot robot"
    )

    declare_username_arg = DeclareLaunchArgument(
        "username", default_value="user", description="Username for the Spot robot"
    )

    declare_password_arg = DeclareLaunchArgument(
        "password", default_value="password", description="Password for the Spot robot"
    )

    declare_odomframe_arg = DeclareLaunchArgument(
        "odometry_frame", default_value="kinematic", description="Odometry frame to use (kinematic or vision)"
    )

    declare_streaming_client_arg = DeclareLaunchArgument(
        "use_streaming_client",
        default_value="false",
        description="Whether to use the streaming client (requires license)",
    )

    declare_cmd_vel_duration_arg = DeclareLaunchArgument(
        "cmd_vel_command_duration",
        default_value="1.0",
        description="Seconds each /cmd_vel command remains valid on the robot",
    )

    declare_gripper_camera_arg = DeclareLaunchArgument(
        "gripper_camera",
        default_value="false",
        description="Whether to publish the hand_color_image gripper camera when Spot has an arm/gripper",
    )

    declare_front_camera_rate_arg = DeclareLaunchArgument(
        "front_camera_rate",
        default_value="10.0",
        description="Front fisheye camera publish rate in Hz. Set <= 0 to disable.",
    )

    declare_gripper_camera_rate_arg = DeclareLaunchArgument(
        "gripper_camera_rate",
        default_value="10.0",
        description="Gripper camera publish rate in Hz. Set <= 0 to disable.",
    )

    declare_jpeg_quality_arg = DeclareLaunchArgument(
        "jpeg_quality",
        default_value="75",
        description="JPEG quality percent for Spot image requests",
    )

    declare_gripper_camera_resolution_arg = DeclareLaunchArgument(
        "gripper_camera_resolution",
        default_value="1920x1080",
        description="Gripper camera resolution set through the Spot SDK gripper camera param service",
    )

    declare_rviz_arg = DeclareLaunchArgument("rviz", default_value="false", description="Whether to start RViz")

    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", default_value="spot.rviz", description="RViz configuration file name"
    )

    # Get launch configuration values
    hostname = LaunchConfiguration("hostname")
    username = LaunchConfiguration("username")
    password = LaunchConfiguration("password")

    odometry_frame = LaunchConfiguration("odometry_frame")
    use_streaming_client = LaunchConfiguration("use_streaming_client")
    cmd_vel_command_duration = LaunchConfiguration("cmd_vel_command_duration")
    gripper_camera = LaunchConfiguration("gripper_camera")
    front_camera_rate = LaunchConfiguration("front_camera_rate")
    gripper_camera_rate = LaunchConfiguration("gripper_camera_rate")
    jpeg_quality = LaunchConfiguration("jpeg_quality")
    gripper_camera_resolution = LaunchConfiguration("gripper_camera_resolution")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # Spot driver node
    spot_driver_node = Node(
        package="spot_driver",
        executable="spot_driver_node",
        name="spot_driver_node",
        output="screen",
        parameters=[
            {
                "hostname": hostname,
                "username": username,
                "password": password,
                "odometry_frame": odometry_frame,
                "use_streaming_client": use_streaming_client,
                "cmd_vel_command_duration": cmd_vel_command_duration,
                "gripper_camera": gripper_camera,
                "front_camera_rate": front_camera_rate,
                "gripper_camera_rate": gripper_camera_rate,
                "jpeg_quality": jpeg_quality,
                "gripper_camera_resolution": gripper_camera_resolution,
            }
        ],
        sigterm_timeout=LaunchConfiguration("sigterm_timeout", default="30"),
        sigkill_timeout=LaunchConfiguration("sigkill_timeout", default="30"),
    )

    # RViz node with conditional launch
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        arguments=["-d", [FindPackageShare("_driver"), "/config/", rviz_config]],
    )

    # Group all nodes
    nodes_group = GroupAction(
        [
            spot_driver_node,
            rviz_node,
        ]
    )

    return LaunchDescription(
        [
            declare_hostname_arg,
            declare_username_arg,
            declare_password_arg,
            declare_odomframe_arg,
            declare_streaming_client_arg,
            declare_cmd_vel_duration_arg,
            declare_gripper_camera_arg,
            declare_front_camera_rate_arg,
            declare_gripper_camera_rate_arg,
            declare_jpeg_quality_arg,
            declare_gripper_camera_resolution_arg,
            declare_rviz_arg,
            declare_rviz_config_arg,
            nodes_group,
        ]
    )
