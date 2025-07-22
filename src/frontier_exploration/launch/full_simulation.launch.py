from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    el7009_diff_drive_dir = get_package_share_directory('el7009_diff_drive_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    frontier_exploration_dir = get_package_share_directory('frontier_exploration')

    nav2_params_file = os.path.join(
        frontier_exploration_dir,
        'config',
        'nav2_params.yaml'
    )
    slam_toolbox_params_file = os.path.join(
        frontier_exploration_dir,
        'config',
        'mapper_params_online_async.yaml'
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(el7009_diff_drive_dir, 'launch', 'robot.launch.py')
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true', 'slam_params_file': slam_toolbox_params_file}.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        #robot_launch,
        static_tf_base,
        slam_launch,
        nav2_launch,
    ])
