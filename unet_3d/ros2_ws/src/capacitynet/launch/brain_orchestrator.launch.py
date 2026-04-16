#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the brain orchestrator node."""

    return LaunchDescription([
        # AR Tag
        DeclareLaunchArgument(
            'ar_tag_topic',
            default_value='/ar_pose_marker',
            description='Topic name for AR tag pose detection (PoseStamped message)'
        ),
        DeclareLaunchArgument(
            'ar_tag_timeout',
            default_value='10.0',
            description='Timeout for AR tag detection in seconds'
        ),
        DeclareLaunchArgument(
            'ar_tag_id',
            default_value='-1',
            description='Specific AR tag ID to track (-1 for any tag)'
        ),

        # Trigger
        DeclareLaunchArgument(
            'trigger_topic',
            default_value='/brain/trigger',
            description='Topic name for triggering pick-and-place sequence (Bool message)'
        ),

        # Workspace center
        DeclareLaunchArgument(
            'workspace_center_topic',
            default_value='/workspace_center',
            description='Topic name for publishing workspace center (PointStamped message)'
        ),

        # Services
        DeclareLaunchArgument(
            'trajectory_service',
            default_value='/unified_planner/generate_trajectory',
            description='Service name for trajectory generation'
        ),
        DeclareLaunchArgument(
            'gripper_action',
            default_value='/robotiq_gripper_controller/gripper_cmd',
            description='Action name for gripper control'
        ),

        # Gripper parameters
        DeclareLaunchArgument(
            'gripper_open_position',
            default_value='0.0',
            description='Gripper position for open state (meters or radians)'
        ),
        DeclareLaunchArgument(
            'gripper_closed_position',
            default_value='0.08',
            description='Gripper position for closed state (meters or radians)'
        ),
        DeclareLaunchArgument(
            'gripper_max_effort',
            default_value='50.0',
            description='Maximum effort for gripper (Newtons)'
        ),

        # Pick position offsets
        DeclareLaunchArgument(
            'pick_approach_offset_z',
            default_value='0.15',
            description='Z offset for approach position above object (meters)'
        ),
        DeclareLaunchArgument(
            'pick_grasp_offset_z',
            default_value='0.05',
            description='Z offset for grasp position above object (meters)'
        ),

        # Place position
        DeclareLaunchArgument(
            'place_position_x',
            default_value='0.5',
            description='X coordinate for place position (meters)'
        ),
        DeclareLaunchArgument(
            'place_position_y',
            default_value='0.3',
            description='Y coordinate for place position (meters)'
        ),
        DeclareLaunchArgument(
            'place_position_z',
            default_value='0.2',
            description='Z coordinate for place position (meters)'
        ),
        DeclareLaunchArgument(
            'place_offset_z',
            default_value='0.10',
            description='Z offset above place position (meters)'
        ),

        # Home position
        DeclareLaunchArgument(
            'home_position_x',
            default_value='0.0',
            description='X coordinate for home position (meters)'
        ),
        DeclareLaunchArgument(
            'home_position_y',
            default_value='0.0',
            description='Y coordinate for home position (meters)'
        ),
        DeclareLaunchArgument(
            'home_position_z',
            default_value='0.5',
            description='Z coordinate for home position (meters)'
        ),

        # Timing
        DeclareLaunchArgument(
            'state_check_frequency',
            default_value='10.0',
            description='Frequency for state machine updates (Hz)'
        ),
        DeclareLaunchArgument(
            'trajectory_timeout',
            default_value='30.0',
            description='Timeout for trajectory generation service (seconds)'
        ),
        DeclareLaunchArgument(
            'gripper_timeout',
            default_value='5.0',
            description='Timeout for gripper action (seconds)'
        ),

        # Node
        Node(
            package='capacitynet',
            executable='brain_orchestrator',
            name='brain_orchestrator',
            output='screen',
            parameters=[{
                'ar_tag_topic': LaunchConfiguration('ar_tag_topic'),
                'ar_tag_timeout': LaunchConfiguration('ar_tag_timeout'),
                'ar_tag_id': LaunchConfiguration('ar_tag_id'),
                'trigger_topic': LaunchConfiguration('trigger_topic'),
                'workspace_center_topic': LaunchConfiguration('workspace_center_topic'),
                'trajectory_service': LaunchConfiguration('trajectory_service'),
                'gripper_action': LaunchConfiguration('gripper_action'),
                'gripper_open_position': LaunchConfiguration('gripper_open_position'),
                'gripper_closed_position': LaunchConfiguration('gripper_closed_position'),
                'gripper_max_effort': LaunchConfiguration('gripper_max_effort'),
                'pick_approach_offset_z': LaunchConfiguration('pick_approach_offset_z'),
                'pick_grasp_offset_z': LaunchConfiguration('pick_grasp_offset_z'),
                'place_position_x': LaunchConfiguration('place_position_x'),
                'place_position_y': LaunchConfiguration('place_position_y'),
                'place_position_z': LaunchConfiguration('place_position_z'),
                'place_offset_z': LaunchConfiguration('place_offset_z'),
                'home_position_x': LaunchConfiguration('home_position_x'),
                'home_position_y': LaunchConfiguration('home_position_y'),
                'home_position_z': LaunchConfiguration('home_position_z'),
                'state_check_frequency': LaunchConfiguration('state_check_frequency'),
                'trajectory_timeout': LaunchConfiguration('trajectory_timeout'),
                'gripper_timeout': LaunchConfiguration('gripper_timeout'),
            }]
        )
    ])
