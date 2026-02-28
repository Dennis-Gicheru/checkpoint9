from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='attach_shelf',
            executable='approach_service_server_node',
            name='approach_service_server_node',
            output='screen'
        ),
        Node(
            package='attach_shelf',
            executable='pre_approach_v2_node',
            name='pre_approach_v2_node',
            output='screen',
            parameters=[
                {'obstacle': 0.3},
                {'degrees': -90},
                {'final_approach': True}
            ]
        )
    ])