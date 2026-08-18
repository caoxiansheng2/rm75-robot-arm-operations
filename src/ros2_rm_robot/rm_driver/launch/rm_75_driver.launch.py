import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    arm_config = os.path.join(
        get_package_share_directory("rm_driver"),
        "config",
        "rm_75_config.yaml",
    )

    right_arm_parameters = {
        "arm_ip": "192.168.3.19",
        "tcp_port": 8080,
        "arm_type": "RM_75",
        "arm_dof": 7,
        "udp_ip": "192.168.3.210",
        "udp_port": 8090,
        "udp_cycle": 5,
    }

    rm_driver_node = Node(
        package="rm_driver",
        executable="rm_driver",
        # Keep one RM75 driver process per RK board. flock releases the lock
        # automatically when the process exits, so stale lock files are safe.
        prefix=[
            "flock --nonblock --exclusive --no-fork /tmp/rm75_rm_driver.lock",
        ],
        on_exit=Shutdown(
            reason="RM75 driver exited or another RM75 driver already owns the lock"
        ),
        parameters=[
            arm_config,
            right_arm_parameters,
        ],
        output="screen",
    )

    return LaunchDescription([
        rm_driver_node,
    ])
