import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('attach_shelf'),
        'rviz',
        'config.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('obstacle', default_value='0.3'),
        DeclareLaunchArgument('degrees', default_value='-90'),
        DeclareLaunchArgument('final_approach', default_value='true'),

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
            parameters=[{
                'obstacle': LaunchConfiguration('obstacle'),
                'degrees': LaunchConfiguration('degrees'),
                'final_approach': LaunchConfiguration('final_approach')
            }]
        ),
        
        # RViz2 Node with the config file argument
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])