from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rm_bringup_share = Path(get_package_share_directory("rm_bringup"))
    fixed_grasp_share = Path(get_package_share_directory("rm75_fixed_grasp"))
    easy_handeye_share = Path(get_package_share_directory("easy_handeye2"))
    serial_no = LaunchConfiguration("serial_no")
    start_auto_grasp = LaunchConfiguration("start_auto_grasp")
    allow_motion = LaunchConfiguration("allow_motion")
    dry_run = LaunchConfiguration("dry_run")
    execute_contact = LaunchConfiguration("execute_contact")
    auto_execute = LaunchConfiguration("auto_execute")

    serial_argument = DeclareLaunchArgument(
        "serial_no",
        default_value="_344422071193",
        description="RealSense serial used by the right-arm vision pipeline",
    )
    start_auto_grasp_argument = DeclareLaunchArgument(
        "start_auto_grasp", default_value="true"
    )
    allow_motion_argument = DeclareLaunchArgument(
        "allow_motion", default_value="true"
    )
    dry_run_argument = DeclareLaunchArgument(
        "dry_run", default_value="false"
    )
    execute_contact_argument = DeclareLaunchArgument(
        "execute_contact", default_value="true"
    )
    auto_execute_argument = DeclareLaunchArgument(
        "auto_execute",
        default_value="true",
        description="Automatically execute once after grab_tag is fresh and stable",
    )
    # Starts the RM75 driver, robot state publisher, rm_control, MoveIt and the
    # single RViz instance configured by rm_75_config/config/moveit.rviz.
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(rm_bringup_share / "launch" / "rm_75_bringup.launch.py")
        )
    )

    # Starts the selected D435 and AprilTag detector.  Its own lightweight
    # camera RViz is disabled so the image appears in the MoveIt RViz instead.
    camera_and_tag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fixed_grasp_share / "launch" / "camera_tag.launch.py")
        ),
        launch_arguments={
            "serial_no": serial_no,
            "start_rviz": "false",
        }.items(),
    )

    handeye_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(easy_handeye_share / "launch" / "publish.launch.py")
        ),
        launch_arguments={"name": "rm75_right_handeye"}.items(),
    )

    automatic_fixed_grasp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fixed_grasp_share / "launch" / "fixed_grasp.launch.py")
        ),
        launch_arguments={
            "allow_motion": allow_motion,
            "dry_run": dry_run,
            "execute_contact": execute_contact,
            "auto_execute": auto_execute,
        }.items(),
        condition=IfCondition(start_auto_grasp),
    )

    return LaunchDescription([
        serial_argument,
        start_auto_grasp_argument,
        allow_motion_argument,
        dry_run_argument,
        execute_contact_argument,
        auto_execute_argument,
        robot_bringup,
        camera_and_tag,
        handeye_publisher,
        automatic_fixed_grasp,
    ])
