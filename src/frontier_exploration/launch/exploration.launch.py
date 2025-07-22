from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    explorer = Node(
        package='frontier_exploration',
        executable='explorer',
        name='frontier_explorer_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    navigation = Node(
        package='frontier_exploration',
        executable='navigation',
        name='frontier_client_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        explorer,
        navigation
    ])

