import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    base_config = os.path.join(
        get_package_share_directory("rm_driver"),
        "config",
        "rm_75_config.yaml",
    )

    with open(base_config, "r", encoding="utf-8") as file:
        params = yaml.safe_load(file)["rm_driver"]["ros__parameters"]

    params.update({
        "arm_ip": "192.168.3.19",
        "tcp_port": 8080,
        "arm_type": "RM_75",
        "arm_dof": 7,

        "udp_ip": "192.168.3.210",
        "udp_port": 8090,
        "udp_cycle": 5,
    })

    return LaunchDescription([
        Node(
            package="rm_driver",
            executable="rm_driver",
            parameters=[params],
            output="screen",
        )
    ])
