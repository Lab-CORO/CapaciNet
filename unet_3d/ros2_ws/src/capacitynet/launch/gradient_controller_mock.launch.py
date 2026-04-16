#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the gradient controller mock node for testing."""

    # Declare launch arguments
    grid_spacing_arg = DeclareLaunchArgument(
        'grid_spacing',
        default_value='0.10',
        description='Grid spacing delta in meters'
    )

    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='1.0',
        description='Control loop frequency in Hz'
    )

    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='1.0',
        description='Proportional gain k for velocity command'
    )

    workspace_radius_arg = DeclareLaunchArgument(
        'workspace_radius',
        default_value='0.30',
        description='Workspace radius in meters'
    )

    voxel_resolution_arg = DeclareLaunchArgument(
        'voxel_resolution',
        default_value='0.02',
        description='Voxel resolution in meters'
    )

    use_static_obstacles_arg = DeclareLaunchArgument(
        'use_static_obstacles',
        default_value='false',
        description='Whether to load static obstacles from YAML'
    )

    static_obstacles_yaml_arg = DeclareLaunchArgument(
        'static_obstacles_yaml',
        default_value='/workspace/capacitynet/config/floor_world.yml',
        description='Path to static obstacles YAML file'
    )

    # Create node
    gradient_controller_mock_node = Node(
        package='capacitynet',
        executable='gradient_controller_mock',
        name='gradient_controller_mock',
        output='screen',
        parameters=[{
            'grid_spacing': LaunchConfiguration('grid_spacing'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'gain': LaunchConfiguration('gain'),
            'workspace_radius': LaunchConfiguration('workspace_radius'),
            'voxel_resolution': LaunchConfiguration('voxel_resolution'),
            'use_static_obstacles': LaunchConfiguration('use_static_obstacles'),
            'static_obstacles_yaml': LaunchConfiguration('static_obstacles_yaml'),
        }]
    )

    return LaunchDescription([
        grid_spacing_arg,
        control_frequency_arg,
        gain_arg,
        workspace_radius_arg,
        voxel_resolution_arg,
        use_static_obstacles_arg,
        static_obstacles_yaml_arg,
        gradient_controller_mock_node
    ])
