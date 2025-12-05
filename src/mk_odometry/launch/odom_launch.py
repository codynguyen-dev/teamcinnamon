from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    helper = '/class_ws/src/class_src/odometry_helper/launch/odom_helper_launch.py'

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(helper)),
        Node(
            package='mk_odometry',
            executable='mk_odom',
            name='odometer',
            output='screen',
            parameters=[{'L': 0.05}],
        ),
    ])