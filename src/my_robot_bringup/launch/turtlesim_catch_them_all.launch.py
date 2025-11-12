#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    
    # Ruta al archivo de parámetros
    config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'turtlesim_catch_them_all.yaml'
    )
    
    # Nodo de turtlesim
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )
    
    # Nodo turtle_spawner
    turtle_spawner_node = Node(
        package='turtlesim_catch_them_all',
        executable='turtle_spawner',
        name='turtle_spawner',
        parameters=[config]
    )
    
    # Nodo turtle_controller
    turtle_controller_node = Node(
        package='turtlesim_catch_them_all',
        executable='turtle_controller',
        name='turtle_controller',
        parameters=[config]
    )
    
    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)
    
    return ld
