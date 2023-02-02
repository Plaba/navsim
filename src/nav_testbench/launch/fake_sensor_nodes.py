from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

MAP_CONFIG = {
    "map":{
        "path": os.path.join(
            get_package_share_directory('nav_testbench'),
            'config',
            'igvc-map-fixed.png'
        ),
        "width": 36.576,
        "height": 42.672,
    }
}

def get_fake_lidar():
    lidar_config = os.path.join(
        get_package_share_directory('nav_testbench'),
        'config',
        'fake_lidar.yaml'
    )

    return Node(
        package="nav_testbench",
        executable="fake_lidar",
        parameters=[lidar_config, MAP_CONFIG]
    )


def get_fake_cameras():
    fake_camera_config = os.path.join(
        get_package_share_directory('nav_testbench'),
        'config',
        'fake_lane_camera.yaml'
    )

    return (
        Node(
            package="nav_testbench",
            executable="fake_lane_camera",
            parameters=[fake_camera_config, {"frame": "left_lanes"}, MAP_CONFIG],
            remappings=[("lane_cloud", "left_lane_cloud"),
                        ("camera_view", "left_camera_view")]
        ),
        Node(
            package="nav_testbench",
            executable="fake_lane_camera",
            parameters=[fake_camera_config, {"frame": "right_lanes"}, MAP_CONFIG],
            remappings=[("lane_cloud", "right_lane_cloud"),
                        ("camera_view", "right_camera_view")]
        )
    )


def get_fake_odom():
    odom_config = os.path.join(
        get_package_share_directory('nav_testbench'),
        'config',
        'fake_odom.yaml'
    )

    return Node(
        package="nav_testbench",
        executable="fake_robot_odom",
        parameters=[odom_config]
    )
