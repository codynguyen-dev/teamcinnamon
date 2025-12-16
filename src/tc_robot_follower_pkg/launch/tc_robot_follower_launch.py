from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='tc_robot_follower_pkg',
            executable='image_processor_node',
            name='image_processor_node',
            namespace='abot',
            output='screen',
            remappings=[
                # camera driver publishes here
                ('camera/image_raw', '/abot/camera/image_raw'),
            ],
        ),

        Node(
            package='tc_robot_follower_pkg',
            executable='dual_pid_controller_node',
            name='dual_pid_controller_node',
            namespace='abot',
            output='screen',
        ),

        Node(
            package='tc_robot_follower_pkg',
            executable='fsm_node',
            name='fsm_node',
            namespace='abot',
            output='screen',
        ),

        Node(
            package='tc_robot_follower_pkg',
            executable='keyboard_mode_switch',
            name='keyboard_mode_switch',
            namespace='abot',
            output='screen',
        ),
    ])
