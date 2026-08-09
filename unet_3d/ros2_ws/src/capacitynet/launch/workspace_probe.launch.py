#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Only the region and inference knobs — the probe has no motion parameters
# because it has no BaseCommander, and so no way to command the base.
ARGUMENTS = [
    ('voxel_grid_topic', '/curobo_trajectory_planner/voxel_grid_sparse',
     'curobo_msgs/SparseVoxelGrid topic driving the evaluation loop'),
    ('marker_topic', '/fiducial_markers',
     'FiducialMarkerArray topic providing the fallback workspace center'),
    ('goal_topic', '/curobo_trajectory_planner/mpc_goal',
     'geometry_msgs/Pose grasp target published by grasp.py; primary region center'),
    ('path_topic', '/mpc_predicted_path',
     'nav_msgs/Path whose tail is unioned with the goal sphere'),
    ('path_tail_samples', '4',
     'Number of path poses (from the end) added to the scored region; 0 = goal only'),
    ('planning_frame', 'dsr01/world',
     'Frame assumed for goal_topic, which carries no header'),
    ('target_marker_id', '-1',
     'Marker id used as workspace center; -1 = first marker in the array'),
    ('model_config_path', '/home/ros2_ws/src/capacitynet/config/test_reach.yaml',
     'Path to the reachability model YAML config'),
    ('fp16', 'false',
     'Run the PyTorch fallback in half precision (ignored when a TRT engine loads)'),

    ('workspace_radius', '0.30',
     'Goal sphere radius in meters (region center 0)'),
    ('path_radius', '0.30',
     'Path-tail sphere radius in meters (region centers 1+). Defaults to the '
     'same value as workspace_radius'),
    ('grid_spacing', '0.10',
     'Grid spacing delta in meters between the 9 candidate base positions'),
    ('gradient_method', 'least_squares',
     "Gradient stencil: 'least_squares' (all 9 maps) or 'central' (4 maps)"),

    ('use_static_obstacles', 'false',
     'Load world-fixed obstacles from YAML so they are not translated with the base'),
    ('static_obstacles_yaml', '/home/ros2_ws/src/capacitynet/config/floor_world.yml',
     'Path to the static obstacles YAML'),

    ('marker_timeout', '2.0',
     'Seconds without a marker detection before the marker region is dropped'),
    ('mpc_timeout', '2.0',
     'Seconds without an MPC goal/path before falling back to the marker'),
    ('static_workspace_center', '',
     'Fixed workspace center "x,y,z" in the voxel grid frame, overriding both the '
     'MPC goal and the marker. Required for bag replay, since no recorded bag '
     'contains /fiducial_markers or /mpc_predicted_path'),

    ('log_timing', 'true',
     'Log gradient, Q and cycle time each evaluation cycle'),
    ('log_quality_scores', 'false',
     'Log the 3x3 block of quality scores each cycle'),
    ('publish_debug_markers', 'true',
     'Publish the region spheres, the 9 candidate scores and the gradient arrow'),
]

# Topic remappings, not node parameters: WorkspaceRegionSource's TransformListener
# subscribes to the plain 'tf'/'tf_static' topics, which some robots publish under
# a namespace (e.g. /leeloo/tf) instead of the default /tf, /tf_static.
TF_REMAP_ARGUMENTS = [
    ('tf_topic', '/tf', 'Topic carrying tf2_msgs/TFMessage for dynamic transforms'),
    ('tf_static_topic', '/tf_static', 'Topic carrying tf2_msgs/TFMessage for static transforms'),
]


def generate_launch_description():
    """Launch the reachability pipeline without any base actuation."""

    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS + TF_REMAP_ARGUMENTS
    ]

    parameters = {name: LaunchConfiguration(name) for name, _, _ in ARGUMENTS}

    workspace_probe_node = Node(
        package='capacitynet',
        executable='workspace_probe',
        name='workspace_probe',
        output='screen',
        parameters=[parameters],
        remappings=[
            ('tf', LaunchConfiguration('tf_topic')),
            ('tf_static', LaunchConfiguration('tf_static_topic')),
        ],
    )

    return LaunchDescription(declared + [workspace_probe_node])
