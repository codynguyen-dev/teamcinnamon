from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('tc_robot_follower_pkg')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    return LaunchDescription([
        # Image Processor
        Node(
            package='tc_robot_follower_pkg',
            executable='image_processor_node',
            name='image_processor_node',
            parameters=[params_file]
        ),
        
        # PID Controller
        Node(
            package='tc_robot_follower_pkg',
            executable='dual_pid_controller_node',
            name='dual_pid_controller_node',
            parameters=[params_file]
        ),
        
        # FSM
        Node(
            package='tc_robot_follower_pkg',
            executable='fsm_node',
            name='fsm_node'
        ),
        
        # Keyboard Mode Switch
        Node(
            package='tc_robot_follower_pkg',
            executable='keyboard_mode_switch',
            name='keyboard_mode_switch',
            output='screen',
        ),
    ])
