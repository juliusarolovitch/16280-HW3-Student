from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_processing_pkg',
            executable='image_processor',
            name='image_processor',
            output='screen'
        ),
    ])
