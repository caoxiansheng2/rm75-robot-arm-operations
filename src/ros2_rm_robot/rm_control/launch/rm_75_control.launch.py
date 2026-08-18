from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    control_node = Node(
    package='rm_control', #节点所在的功能包
    executable='rm_control', #表示要运行的可执行文件名或脚本名字.py
    parameters= [
                    # RM75 high-follow CANFD requires a transmission period no
                    # greater than 10 ms. rm_control now interpolates and
                    # publishes every 5 ms.
                    {'follow': True},
                    {'arm_type': 75}
                ],             #接入参数文件
    output='screen', #用于将话题信息打印到屏幕
    )

    ld.add_action(control_node)
    return ld

