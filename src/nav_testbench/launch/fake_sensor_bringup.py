from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import sys
import os
sys.path.append(os.path.join(get_package_share_directory('nav_testbench'), 'launch'))

import fake_sensor_nodes

def get_static_transforms():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0.109", "0.994", "base_link", "left_lanes"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "-0.109", "0.994", "base_link", "right_lanes"]
        )
    ])

def generate_launch_description():
    right_camera, left_camera = fake_sensor_nodes.get_fake_cameras()

    return LaunchDescription([
        get_static_transforms(),
        fake_sensor_nodes.get_fake_odom(),
        fake_sensor_nodes.get_fake_lidar(),
        right_camera,
        left_camera
    ])

def generate_launch_description_with_config(map_config, odom_config):
    right_camera, left_camera = fake_sensor_nodes.get_fake_cameras(config=map_config)

    return LaunchDescription([
        get_static_transforms(),
        fake_sensor_nodes.get_fake_odom(config=odom_config),
        fake_sensor_nodes.get_fake_lidar(config=map_config),
        right_camera,
        left_camera
    ])
