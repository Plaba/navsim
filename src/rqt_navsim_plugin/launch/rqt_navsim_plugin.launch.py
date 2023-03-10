from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_navsim_plugin',
            node_executable='rqt_navsim_plugin',
            node_name='rqt_navsim_plugin',
            output='screen'
        )
    ])