from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fixed_grasp_share = Path(get_package_share_directory("rm75_fixed_grasp"))
    realsense_share = Path(get_package_share_directory("realsense2_camera"))

    serial_no = LaunchConfiguration("serial_no")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    start_rviz = LaunchConfiguration("start_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_no",
                default_value="_344422071193",
                description="RealSense serial number (prefix numeric serials with _)",
            ),
            DeclareLaunchArgument(
                "image_topic", default_value="/camera/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(realsense_share / "launch" / "rs_launch.py")
                ),
                launch_arguments={
                    "camera_namespace": "camera",
                    "camera_name": "camera",
                    # Keep USB load low on the RK board's shared controller.
                    # AprilTag only needs the color stream.
                    "align_depth.enable": "false",
                    "enable_color": "true",
                    "rgb_camera.color_profile": "640x480x15",
                    "enable_depth": "false",
                    "enable_infra": "false",
                    "enable_infra1": "false",
                    "enable_infra2": "false",
                    "enable_gyro": "false",
                    "enable_accel": "false",
                    "enable_sync": "false",
                    "pointcloud.enable": "false",
                    "initial_reset": "false",
                    "serial_no": serial_no,
                }.items(),
            ),
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag",
                output="screen",
                parameters=[str(fixed_grasp_share / "config" / "apriltag.yaml")],
                remappings=[
                    ("image_rect", image_topic),
                    ("camera_info", camera_info_topic),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="tag_camera_rviz",
                arguments=["-d", str(fixed_grasp_share / "config" / "tag_camera.rviz")],
                condition=IfCondition(start_rviz),
                output="screen",
            ),
        ]
    )
