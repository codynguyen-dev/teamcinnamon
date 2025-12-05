from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = 'mk_pkg'
    lab12_launch = 'mvturtle_launch.py'
    output_topic = '/turtlesim1/distance'

    return LaunchDescription([
        # Include your Lab 1.2 launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare(pkg), lab12_launch])
            )
        ),

        # Start the new subpubturtle node
        Node(
            package=pkg,
            namespace='turtlesim1',
            executable='distance',  # matches filename / entry point
            name='distance'             # runtime node name
        ),

        # Echo the distance_traveled topic
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', output_topic, 'std_msgs/Float32'],
            shell=True,
            output='screen'
        )
    ])