from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mk_color_filter',             
            executable='color_filter_node',      
            name='color_filter_node',
            output='screen',
            parameters=[{
                'target_color': 'blue',           
                'kernel_size': 5,
                'iterations': 2,
                'in_image_topic': '/image_raw',
                'out_image_topic': '/image_filtered',
                'publish_mask_debug': True,
            }]
        ),
        Node(
            package='image_view',
            executable='image_view',
            name='viewer',
            output='screen',
            remappings=[('image', '/image_filtered')]
        ),
         Node(
            package='mk_color_filter',
            executable='measurement_data',
            name='measurement_data',
            output='screen',
            parameters=[{
                'mask_topic': '/image_filtered_mask'
            }]
        )
    ])

