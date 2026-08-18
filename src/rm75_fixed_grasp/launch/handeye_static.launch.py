from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("parent_frame", default_value="Link7"),
        DeclareLaunchArgument("child_frame", default_value="camera_link"),
        DeclareLaunchArgument("x"),
        DeclareLaunchArgument("y"),
        DeclareLaunchArgument("z"),
        DeclareLaunchArgument("roll"),
        DeclareLaunchArgument("pitch"),
        DeclareLaunchArgument("yaw"),
    ]

    publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="handeye_static_transform",
        arguments=[
            "--x", LaunchConfiguration("x"),
            "--y", LaunchConfiguration("y"),
            "--z", LaunchConfiguration("z"),
            "--roll", LaunchConfiguration("roll"),
            "--pitch", LaunchConfiguration("pitch"),
            "--yaw", LaunchConfiguration("yaw"),
            "--frame-id", LaunchConfiguration("parent_frame"),
            "--child-frame-id", LaunchConfiguration("child_frame"),
        ],
        output="screen",
    )
    return LaunchDescription(arguments + [publisher])
