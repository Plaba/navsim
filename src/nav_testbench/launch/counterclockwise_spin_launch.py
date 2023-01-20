from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    sensor_bringup_path = os.path.join(
        get_package_share_directory('nav_testbench'),
        'launch',
        'fake_sensor_bringup.py'
    )

    pub = Node(
            executable='ros2',
            arguments = [
                    'topic', 
                    'pub',
                    '-r4',
                    'cmd_vel',
                    'geometry_msgs/Twist',
                    "linear:\n  x: 0.5\n  y: 0.0\n  z: 0.0\nangular:\n  x: 0.0\n  y: 0.0\n  z: 0.1"
                ],
        )
    for sub_list in pub.cmd:
        for sub in sub_list:
            if "--ros-args" in str(sub.describe()):
                pub.cmd.remove(sub_list)

    return LaunchDescription([
        pub,
        IncludeLaunchDescription(
            launch_description_source = sensor_bringup_path
        )
    ])
