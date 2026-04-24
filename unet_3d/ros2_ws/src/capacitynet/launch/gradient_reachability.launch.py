#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the reachability node with gradient-based control."""

    # Declare launch arguments
    enable_gradient_control_arg = DeclareLaunchArgument(
        'enable_gradient_control',
        default_value='true',
        description='Enable gradient-based mobile base control'
    )

    grid_spacing_arg = DeclareLaunchArgument(
        'grid_spacing',
        default_value='0.10',
        description='Grid spacing delta in meters'
    )

    control_gain_arg = DeclareLaunchArgument(
        'control_gain',
        default_value='1.0',
        description='Proportional gain k for velocity command'
    )

    max_linear_velocity_arg = DeclareLaunchArgument(
        'max_linear_velocity',
        default_value='0.10',
        description='Maximum linear velocity in m/s'
    )

    workspace_radius_arg = DeclareLaunchArgument(
        'workspace_radius',
        default_value='0.30',
        description='Workspace radius in meters'
    )

    use_static_obstacles_arg = DeclareLaunchArgument(
        'use_static_obstacles',
        default_value='false',
        description='Whether to load static obstacles from YAML'
    )

    static_obstacles_yaml_arg = DeclareLaunchArgument(
        'static_obstacles_yaml',
        default_value='/home/ros2_ws/src/capacitynet/config/floor_world.yml',
        description='Path to static obstacles YAML file'
    )

    workspace_center_topic_arg = DeclareLaunchArgument(
        'workspace_center_topic',
        default_value='/workspace_center',
        description='Topic name for workspace center position (PointStamped message)'
    )

    log_control_timing_arg = DeclareLaunchArgument(
        'log_control_timing',
        default_value='true',
        description='Log detailed timing information'
    )

    log_quality_scores_arg = DeclareLaunchArgument(
        'log_quality_scores',
        default_value='false',
        description='Log quality scores for all 9 positions'
    )

    # Create node
    reachability_node = Node(
        package='capacitynet',
        executable='reachability_node',
        name='reachability_node',
        output='screen',
        parameters=[{
            'enable_gradient_control': LaunchConfiguration('enable_gradient_control'),
            'grid_spacing': LaunchConfiguration('grid_spacing'),
            'control_gain': LaunchConfiguration('control_gain'),
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'workspace_radius': LaunchConfiguration('workspace_radius'),
            'use_static_obstacles': LaunchConfiguration('use_static_obstacles'),
            'static_obstacles_yaml': LaunchConfiguration('static_obstacles_yaml'),
            'workspace_center_topic': LaunchConfiguration('workspace_center_topic'),
            'log_control_timing': LaunchConfiguration('log_control_timing'),
            'log_quality_scores': LaunchConfiguration('log_quality_scores'),
        }]
    )

    return LaunchDescription([
        enable_gradient_control_arg,
        grid_spacing_arg,
        control_gain_arg,
        max_linear_velocity_arg,
        workspace_radius_arg,
        use_static_obstacles_arg,
        static_obstacles_yaml_arg,
        workspace_center_topic_arg,
        log_control_timing_arg,
        log_quality_scores_arg,
        reachability_node
    ])
