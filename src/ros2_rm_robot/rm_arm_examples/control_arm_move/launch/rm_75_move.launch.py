from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    move_node = Node(
    package='control_arm_move', #节点所在的功能包
    executable='move_demo', #表示要运行的可执行文件名或脚本名字.py
    parameters= [
                    {'arm_dof': 7}
                ],             #接入自由度参数
    output='screen', #用于将话题信息打印到屏幕
    )

    ld.add_action(move_node)
    return ld


