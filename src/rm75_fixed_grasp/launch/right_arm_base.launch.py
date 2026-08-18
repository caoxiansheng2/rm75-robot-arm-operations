from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(package_name, launch_name, arguments=None):
    package_share = Path(get_package_share_directory(package_name))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / "launch" / launch_name)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    # Common right-arm infrastructure only.  Camera, AprilTag, RViz and task
    # nodes are intentionally started on demand by the individual task script.
    return LaunchDescription(
        [
            include("rm_driver", "rm_75_driver.launch.py"),
            include("rm_description", "rm_75_display.launch.py"),
            include("rm_control", "rm_75_control.launch.py"),
            include("rm_75_config", "move_group.launch.py"),
            include(
                "easy_handeye2",
                "publish.launch.py",
                {"name": "rm75_right_handeye"},
            ),
        ]
    )
