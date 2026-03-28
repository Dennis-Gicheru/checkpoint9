import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# Note: We import Node specifically from launch_ros.actions
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate a launch description to run the PreApproach component and RViz."""
    
    # 1. Dynamically find the path to your config file
    pkg_share = get_package_share_directory('my_components') 
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    # 2. Define the Component Container
    container = ComposableNodeContainer(
        name='pre_approach_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_components',
                plugin='my_components::PreApproach',
                name='pre_approach_node'
            ),
        ],
        output='screen',
    )

    # 3. Define the RViz Node
    # Using the 'Node' we imported above
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # 4. Return the list containing BOTH actions
    return LaunchDescription([
        container,
        rviz_node
    ])