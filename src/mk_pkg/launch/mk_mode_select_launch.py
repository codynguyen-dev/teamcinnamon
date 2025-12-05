#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        remappings=[('/turtle1/cmd_vel', '/cmd_vel_mux')],
    )

    moveturtle = Node(
        package='mk_pkg',
        executable='moveturtle',  
        name='turtle_square',
        parameters=[{'rate_hz': 10.0, 'speed': 1.5, 'turn_speed': 1.2, 'side_len': 3.0}],
    )

    fsm = Node(
        package='mk_pkg',
        executable='mk_fsm_mode', 
        name='mode_mux',
        parameters=[{'rate_hz': 20.0}],
    )

    return LaunchDescription([turtlesim, moveturtle, fsm])
