from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    get_arm_state_node = Node(
    package='get_arm_state', #节点所在的功能包
    executable='rm_get_arm_state_demo', #表示要运行的可执行文件名或脚本名字.py
    output='screen', #用于将话题信息打印到屏幕
    )

    ld.add_action(get_arm_state_node)
    return ld


