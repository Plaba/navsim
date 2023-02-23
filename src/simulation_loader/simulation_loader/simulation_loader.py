#!/usr/bin/env python3

import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from launch import LaunchService
from ros2launch.api.api import get_package_share_directory

from yaml import load, Loader

from geometry_msgs.msg import PoseStamped
import tf2_py 

from simulation_loader_msgs.action import LoadSimulation

sys.path.append(os.path.join(get_package_share_directory('nav_testbench'), 'launch')) 

import fake_sensor_bringup

def make_pose(x,y,theta):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    quaternion = tf2_py.toMsg(tf2_py.Quaternion.from_euler(0, 0, theta))
    pose.pose.orientation = quaternion
 
    return pose

class SimulationLoader(Node):

    def __init__(self):
        super().__init__('simulation_loader')
        self.get_logger().info('Simulation loader started')
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

            map = loaded_yaml["map"]
            start = loaded_yaml["start"]
            goals = loaded_yaml["goals"]

        self.bringup_simulation(map, start) 
        self.run_goals(goals)
 
        return LoadSimulation.Result()
    
    def bringup_simulation(self, map, start, goals):
        self.get_logger().info('Bringing up simulation...')
        self.get_logger().info('Map: {}'.format(map))
        self.get_logger().info('Start: {}'.format(start))

        launch_path = get_package_share_directory('nav_testbench') + '/launch/nav_bringup.launch.py'
        launch_service = LaunchService()
        launch_service.include_launch_description(launch_path)
        launch_service.include_launch_description(fake_sensor_bringup.generate_launch_description_with_config(map, start))

        launch_service.run()
    
    def run_goals(self, goals):
        navigator = BasicNavigator(self)
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
