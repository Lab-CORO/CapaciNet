#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the workspace reachability evaluation node."""

    model_config_path_arg = DeclareLaunchArgument(
        'model_config_path',
        default_value='/home/ros2_ws/src/capacitynet/config/test_reach.yaml',
        description='Path to the reachability model YAML config'
    )

    workspace_radius_arg = DeclareLaunchArgument(
        'workspace_radius',
        default_value='0.30',
        description='Workspace sphere radius in meters'
    )

    path_max_samples_arg = DeclareLaunchArgument(
        'path_max_samples',
        default_value='16',
        description='Max number of path poses evaluated per trajectory message (0 = all)'
    )

    episode_timeout_arg = DeclareLaunchArgument(
        'episode_timeout',
        default_value='1.0',
        description='Seconds without a path message before an episode is closed'
    )

    output_csv_prefix_arg = DeclareLaunchArgument(
        'output_csv_prefix',
        default_value='',
        description='Prefix for the *_samples.csv and *_episodes.csv output files '
                    '(empty = timestamped default under capacitynet/results)'
    )

    log_each_sample_arg = DeclareLaunchArgument(
        'log_each_sample',
        default_value='true',
        description='Log every per-trajectory-message sample to the console'
    )

    generate_episode_plot_arg = DeclareLaunchArgument(
        'generate_episode_plot',
        default_value='true',
        description='Save a matplotlib PNG (Q_goal / Q_path over time) at the end of each episode'
    )

    state_topic_arg = DeclareLaunchArgument(
        'state_topic',
        default_value='/object_grasper/state',
        description='std_msgs/String topic carrying the grasp state machine state, '
                    'marked as vertical dashed lines on the episode plot'
    )

    workspace_reachability_node = Node(
        package='capacitynet',
        executable='workspace_reachability_node',
        name='workspace_reachability_node',
        output='screen',
        parameters=[{
            'model_config_path': LaunchConfiguration('model_config_path'),
            'workspace_radius': LaunchConfiguration('workspace_radius'),
            'path_max_samples': LaunchConfiguration('path_max_samples'),
            'episode_timeout': LaunchConfiguration('episode_timeout'),
            'output_csv_prefix': LaunchConfiguration('output_csv_prefix'),
            'log_each_sample': LaunchConfiguration('log_each_sample'),
            'generate_episode_plot': LaunchConfiguration('generate_episode_plot'),
            'state_topic': LaunchConfiguration('state_topic'),
        }]
    )

    return LaunchDescription([
        model_config_path_arg,
        workspace_radius_arg,
        path_max_samples_arg,
        episode_timeout_arg,
        output_csv_prefix_arg,
        log_each_sample_arg,
        generate_episode_plot_arg,
        state_topic_arg,
        workspace_reachability_node
    ])
