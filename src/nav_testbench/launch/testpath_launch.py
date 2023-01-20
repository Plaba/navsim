from multiprocessing import Process
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


import subprocess
import yaml

PATH = [
    (
        3,
        {
            "linear": {"x": 1, "y": 1, "z": 0}, 
            "angular": {"x": 0, "y": 0, "z": 0}, 
        } 
    ),
    (
        1000,
        {
            "linear": {"x": 1, "y": 1, "z": 0}, 
            "angular": {"x": 0, "y": 0, "z": 1}, 
        } 
    )
]

def runpath():
    for num_seconds, cmd_vel in PATH:
        subprocess.run([
            'ros2',
            'topic', 
            'pub',
            '-r4',
            '-p0',
            f'-t{4*num_seconds}',
            'cmd_vel',
            'geometry_msgs/Twist',
            yaml.dump(cmd_vel)
        ])

def generate_launch_description():
    sensor_bringup_path = os.path.join(
        get_package_share_directory('nav_testbench'),
        'launch',
        'fake_sensor_bringup.py'
    )

    description = LaunchDescription(
        [IncludeLaunchDescription(
            launch_description_source = sensor_bringup_path
        )]
    )
    p = Process(target=runpath, )
    p.start()
    return description
