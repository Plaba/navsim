from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    odom_config = os.path.join(
        get_package_share_directory('nav_testbench'),
        'config',
        'fake_odom.yaml'
        )
    
    lidar_config = os.path.join(
        get_package_share_directory('nav_testbench'),
        'config',
        'fake_lidar.yaml'
    )
    
    fake_camera_config = os.path.join(
        get_package_share_directory('nav_testbench'),
        'config',
        'fake_lane_camera.yaml'
    )

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
        ),
        Node(
            package="nav_testbench",
            executable="fake_robot_odom",
            parameters=[odom_config]
        ),
        Node(
            package="nav_testbench",
            executable="fake_lidar",
            parameters=[lidar_config]
        ),
        Node(
            package="nav_testbench",
            executable="fake_lane_camera",
            parameters=[fake_camera_config, {"frame": "left_lanes"}],
            remappings=[("lane_cloud", "left_lane_cloud"), ("camera_view", "left_camera_view")]
        ),
        Node(
            package="nav_testbench",
            executable="fake_lane_camera",
            parameters=[fake_camera_config, {"frame": "right_lanes"}],
            remappings=[("lane_cloud", "right_lane_cloud"), ("camera_view", "right_camera_view")]
        ),
    ])
