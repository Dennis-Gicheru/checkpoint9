import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate a launch description to run the Preapproach component."""
   pkg_share = get_package_share_directory('my_components') 
   rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    # 1. Define the C'ontainer
    container = ComposableNodeContainer(
        name='pre_approach_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 2. Define the Composable Node (The Plugin)
            ComposableNode(
                package='my_components',
                plugin='my_components::PreApproach',
                name='pre_approach_node'
            ),
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([container])