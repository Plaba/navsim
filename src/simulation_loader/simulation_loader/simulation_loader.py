#!/usr/bin/env python3

import numpy as np
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros2launch.api.api import get_package_share_directory

from .robot_navigator import BasicNavigator, NavigationResult
from multiprocessing import Process

from yaml import load, Loader

from geometry_msgs.msg import PoseStamped, Quaternion


from simulation_loader_msgs.action import LoadSimulation

sys.path.append(os.path.join(get_package_share_directory('nav_testbench'), 'launch')) 

import fake_sensor_bringup

def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return (qx, qy, qz, qw)

def make_pose(x,y,theta):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    tuple_quat = euler_to_quaternion(0, 0, theta)
    quaternion = Quaternion()
    quaternion.w = tuple_quat[3]
    quaternion.x = tuple_quat[0]
    quaternion.y = tuple_quat[1]
    quaternion.z = tuple_quat[2]

    pose.pose.orientation = quaternion
 
    return pose

class SimulationLoader(Node):

    def __init__(self):
        super().__init__('simulation_loader')
        self.get_logger().info('Simulation loader started')
        self._launch_handle = None
        self._launch_service = LaunchService()
        self._action_server = ActionServer(
            self,
            LoadSimulation,
            'load_simulation',
            self.execute_callback)
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        path = goal_handle.request.path

        self.get_logger().info('Path: {}'.format(path))

        with open(path, 'r') as file:
            loaded_yaml = load(file, Loader=Loader)

            map = os.path.join(os.path.dirname(os.path.abspath(path)),loaded_yaml["map"]["path"])
            start = loaded_yaml["start"]
            goals = loaded_yaml["goals"]

        self.bringup_simulation({"map":map}, start) 
        self.run_goals(goals)
 
        return LoadSimulation.Result()
    
    def bringup_simulation(self, map, start):
        self.get_logger().info('Bringing up simulation...')
        self.get_logger().info('Map: {}'.format(map))
        self.get_logger().info('Start: {}'.format(start))

        launch_path = get_package_share_directory('nav_testbench') + '/launch/nav_bringup.launch.py'

        self._launch_service.include_launch_description(
            LaunchDescription(
                [IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path))]))
        self._launch_service.include_launch_description(fake_sensor_bringup.generate_launch_description_with_config(map, start))

        #self._launch_handle = self._launch_service.run()
        self._launch_handle = Process(target=self._launch_service.run)
        self._launch_handle.start()

        self.get_logger().info('Simulation brought up!')
    
    def run_goals(self, goals):
        navigator = BasicNavigator()
        navigator.waitUntilNav2Active()
        for goal in goals:
            goal_msg = make_pose(goal["x"], goal["y"], goal["theta"])
            self.get_logger().info('Running goal: {}'.format(goal_msg))

            navigator.goToPose(goal_msg)
        
            while not navigator.isNavComplete():
                rclpy.spin_once(navigator, timeout_sec=0.1)
            
            result = navigator.getResult()
            
            if result == NavigationResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == NavigationResult.CANCELED:
                print('Goal was canceled!')
            elif result == NavigationResult.FAILED:
                print('Goal failed!')
            else:
                print(f'Goal has an invalid return status: {result}')
    

def main(args=None):
    rclpy.init(args=args)

    simulation_loader = SimulationLoader()

    rclpy.spin(simulation_loader)

    simulation_loader.destroy_node()
    rclpy.shutdown()
