from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    noah_launch =  os.path.join(
        get_package_share_directory('nav_testbench'),
        'launch',
        'fake_sensor_bringup.py'
    )

    nav2_launch = os.path.join(
        get_package_share_directory('navigation_launch'),
        'launch',
        'nav2_bringup.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            launch_description_source = noah_launch
        ),
        IncludeLaunchDescription(
            launch_description_source = nav2_launch
        )
    ])
