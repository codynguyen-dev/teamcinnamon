
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controls_helper',
            executable='vehicle_dynamics',
            name='vehicle_dynamics',
            output='screen',
        ),
        Node(
            package='controls_helper',
            executable='controls_graph',
            name='controls_graph',
            output='screen',
        ),
        Node(
            package='mk_pid_control',
            executable='pid_control',
            name='pid_control',
            output='screen',
        ),
    ])