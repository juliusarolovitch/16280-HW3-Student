from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_processing_pkg',
            executable='image_node',
            name='image_node', 
            output='screen'
        ),
        Node(
            package='image_processing_pkg',
            executable='planner_node',
            name='planner_node', 
            output='screen'
        ),
    ])
