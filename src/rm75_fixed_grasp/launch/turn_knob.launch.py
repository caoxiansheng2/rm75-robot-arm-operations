from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rm_bringup_share = Path(get_package_share_directory("rm_bringup"))
    package_share = Path(get_package_share_directory("rm75_fixed_grasp"))
    realsense_share = Path(get_package_share_directory("realsense2_camera"))
    easy_handeye_share = Path(get_package_share_directory("easy_handeye2"))
    knob_config = package_share / "config" / "knob.yaml"

    serial_no = LaunchConfiguration("serial_no")
    allow_motion = LaunchConfiguration("allow_motion")
    dry_run = LaunchConfiguration("dry_run")

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(rm_bringup_share / "launch" / "rm_75_bringup.launch.py")
        )
    )

    # Start one low-bandwidth color stream. No button AprilTag detector is
    # included in this launch.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(realsense_share / "launch" / "rs_launch.py")
        ),
        launch_arguments={
            "camera_namespace": "camera",
            "camera_name": "camera",
            "serial_no": serial_no,
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
        }.items(),
    )

    # knob.yaml maps AprilTag numeric ID 1 (printed label 01) to knob_tag.
    knob_detector = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_knob",
        output="screen",
        parameters=[str(knob_config)],
        remappings=[
            ("image_rect", "/camera/camera/color/image_raw"),
            ("camera_info", "/camera/camera/color/camera_info"),
        ],
    )

    handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(easy_handeye_share / "launch" / "publish.launch.py")
        ),
        launch_arguments={"name": "rm75_right_handeye"}.items(),
    )

    knob_motion = Node(
        package="rm75_fixed_grasp",
        executable="turn_knob_node",
        name="rm75_turn_knob",
        output="screen",
        parameters=[
            str(knob_config),
            {
                "allow_motion": allow_motion,
                "dry_run": dry_run,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_no",
                default_value="_344422071193",
                description="Right-arm RealSense serial number",
            ),
            DeclareLaunchArgument("allow_motion", default_value="true"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            robot_bringup,
            camera,
            knob_detector,
            handeye,
            knob_motion,
        ]
    )
