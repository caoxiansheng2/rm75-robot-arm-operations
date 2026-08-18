from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = Path(get_package_share_directory("rm75_fixed_grasp"))
    config_file = package_share / "config" / "grasp.yaml"
    allow_motion = LaunchConfiguration("allow_motion")
    dry_run = LaunchConfiguration("dry_run")
    execute_contact = LaunchConfiguration("execute_contact")
    auto_execute = LaunchConfiguration("auto_execute")

    moveit_config = (
        MoveItConfigsBuilder("rm_75_description", package_name="rm_75_config")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("allow_motion", default_value="false"),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("execute_contact", default_value="true"),
            DeclareLaunchArgument("auto_execute", default_value="false"),
            Node(
                package="rm75_fixed_grasp",
                executable="fixed_grasp_node",
                name="rm75_fixed_grasp",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    str(config_file),
                    {
                        "allow_motion": allow_motion,
                        "dry_run": dry_run,
                        "execute_contact": execute_contact,
                        "auto_execute": auto_execute,
                    },
                ],
            ),
        ]
    )
