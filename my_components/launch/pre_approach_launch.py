from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate a launch description to run the Preapproach component."""
    
    # 1. Define the Container
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

    return LaunchDescription([container])