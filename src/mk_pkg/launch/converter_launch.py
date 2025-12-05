from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        Node(
            package='mk_pkg',
            executable='moveturtle',
            name='moveturtle'
        ),

        Node(
            package='mk_pkg',
            executable='distance',
            name='distance'
        ),

        ExecuteProcess(
            cmd=['python3', '-m', 'mk_pkg.lab1_4', '--ros-args', '-p', 'output_unit:=smoots'],
            shell=False
        ),

        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/distance_converted'],
            shell=False
        ),
    ])
