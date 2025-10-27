"""
Launch file for testing the base placement pipeline

This launch file starts:
1. The base_placement_server node
2. The test_pipeline client node

Usage:
    ros2 launch base_placement_plugin test_pipeline.launch.py

    # With custom parameters:
    ros2 launch base_placement_plugin test_pipeline.launch.py \
        irm_path:=/path/to/IRM.h5 \
        rm_path:=/path/to/RM.h5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    irm_path_arg = DeclareLaunchArgument(
        'irm_path',
        default_value='/home/ros2_ws/src/CapaciNet/data_generation/data/IRM_0.1.h5',
        description='Path to the IRM (Inverse Reachability Map) HDF5 file'
    )

    rm_path_arg = DeclareLaunchArgument(
        'rm_path',
        default_value='/home/ros2_ws/src/CapaciNet/data_generation/data/master_ik_data0.1.h5',
        description='Path to the Master RM (Reachability Map) HDF5 file'
    )

    # Server node
    server_node = Node(
        package='base_placement_plugin',
        executable='base_placement_server',
        name='base_placement_server',
        output='screen',
        emulate_tty=True,
    )

    # Test client node (delayed to ensure server is ready)
    test_node = TimerAction(
        period=2.0,  # Wait 2 seconds for server to initialize
        actions=[
            Node(
                package='base_placement_plugin',
                executable='test_pipeline',
                name='test_pipeline',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'irm_path': LaunchConfiguration('irm_path'),
                    'rm_path': LaunchConfiguration('rm_path'),
                }]
            )
        ]
    )

    return LaunchDescription([
        irm_path_arg,
        rm_path_arg,
        server_node,
        test_node,
    ])
