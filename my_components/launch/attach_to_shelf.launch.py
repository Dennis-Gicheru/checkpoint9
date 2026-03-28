import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    # 1. Path to RViz config
    pkg_share = get_package_share_directory('my_components') 
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    # 2. Define the Container and Load PreApproach + AttachServer
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_components',
                plugin='my_components::PreApproach',
                name='pre_approach'
            ),
            ComposableNode(
                package='my_components',
                plugin='my_components::AttachServer',
                name='attach_server'
            )
        ],
        output='screen',
    )

    # 3. RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        container,
        rviz_node
    ])