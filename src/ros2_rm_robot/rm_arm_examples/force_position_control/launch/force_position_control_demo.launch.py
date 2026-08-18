from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    force_position_control_node = Node(
    package='force_position_control', #节点所在的功能包
    executable='rm_force_position_control', #表示要运行的可执行文件名或脚本名字.py
    output='screen', #用于将话题信息打印到屏幕
    )

    ld.add_action(force_position_control_node)
    return ld
