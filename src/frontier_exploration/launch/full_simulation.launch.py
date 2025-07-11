from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path al launch del robot
    robot_launch = os.path.join(
        get_package_share_directory('el7009_diff_drive_robot'),
        'launch',
        'robot.launch.py'
    )

    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )
    # Espera por clock
    wait_for_clock = ExecuteProcess(
        cmd=['ros2', 'run', 'frontier_exploration', 'wait_for_clock'],
        output='screen'
    )

    # Lanza SLAM Toolbox después de esperar clock
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    odom_tf_publisher = Node(
        package='frontier_exploration',
        executable='odom_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch)
        ),
        static_tf_map_odom,
        odom_tf_publisher,
        slam_toolbox_launch,
    ])
