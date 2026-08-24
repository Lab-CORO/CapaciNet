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
    ('cycle_period', '2.0',
     'Seconds between inference cycles; the voxel grid subscription only caches '
     'the latest message, a timer runs the cycle on it at this fixed cadence'),
    ('marker_topic', '/fiducial_markers',
     'FiducialMarkerArray topic providing the fallback workspace center'),
    ('goal_topic', '/curobo_trajectory_planner/mpc_goal',
     'geometry_msgs/Pose grasp target published by grasp.py; primary region center'),
    ('path_topic', '/mpc_predicted_path',
     'nav_msgs/Path whose horizon point is unioned with the goal sphere'),
    ('path_horizon_s', '2.0',
     'Seconds from now to look up in the path (nearest header.stamp); negative = goal only'),
    ('planning_frame', 'dsr01/base_link',
     'Frame assumed for goal_topic, which carries no header'),
    ('target_marker_id', '-1',
     'Marker id used as workspace center; -1 = first marker in the array'),
    ('model_config_path', '/home/ros2_ws/src/capacitynet/config/test_reach.yaml',
     'Path to the reachability model YAML config'),
    ('fp16', 'false',
     'Run the PyTorch fallback in half precision (ignored when a TRT engine loads)'),

    ('workspace_radius', '0.10',
     'Goal sphere radius in meters (region center 0)'),
    ('path_radius', '0.10',
     'Path-tail sphere radius in meters (region centers 1+). Defaults to the '
     'same value as workspace_radius'),
    ('grid_spacing', '0.10',
     'Grid spacing delta in meters between candidate base positions'),
    ('grid_size', '3',
     'Number of candidate base positions per grid side (must be odd, >= 3); '
     'total candidates = grid_size**2 (3 -> 9, 5 -> 25, 7 -> 49). Cost scales '
     'linearly with candidate count — see OPTIMIZATIONS.md'),
    ('gradient_method', 'least_squares',
     "Gradient stencil: 'least_squares' (all grid_size**2 maps) or 'central' "
     '(4 nearest neighbors of center, regardless of grid_size)'),

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
     'Log the grid_size x grid_size block of quality scores each cycle'),
    ('publish_debug_markers', 'true',
     'Publish the region spheres, the grid_size**2 candidate scores and the '
     'gradient arrow'),
]

# grasp.py's own params, passed through to the grasp node started alongside
# the probe so it actually drives mpc_goal/mpc_predicted_path.
GRASP_ARGUMENTS = [
    ('grasp_distance', '-0.15',
     'grasp.py: distance (m) along the marker Z at the final grasp pose'),
    ('approach_distance', '-0.23',
     'grasp.py: distance (m) along the marker Z at the pregrasp pose'),
]

# Topic remappings, not node parameters: WorkspaceRegionSource's TransformListener
# subscribes to the plain 'tf'/'tf_static' topics. This robot (leeloo) publishes
# its TF tree under the /leeloo namespace rather than the default /tf, /tf_static.
TF_REMAP_ARGUMENTS = [
    ('tf_topic', '/leeloo/tf', 'Topic carrying tf2_msgs/TFMessage for dynamic transforms'),
    ('tf_static_topic', '/leeloo/tf_static', 'Topic carrying tf2_msgs/TFMessage for static transforms'),
]


def generate_launch_description():
    """Launch the reachability pipeline without any base actuation."""

    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS + GRASP_ARGUMENTS + TF_REMAP_ARGUMENTS
    ]

    parameters = {name: LaunchConfiguration(name) for name, _, _ in ARGUMENTS}
    grasp_parameters = {name: LaunchConfiguration(name) for name, _, _ in GRASP_ARGUMENTS}

    tf_remappings = [
        ('tf', LaunchConfiguration('tf_topic')),
        ('tf_static', LaunchConfiguration('tf_static_topic')),
    ]

    workspace_probe_node = Node(
        package='capacitynet',
        executable='workspace_probe',
        name='workspace_probe',
        output='screen',
        parameters=[parameters],
        remappings=tf_remappings,
    )

    # Drives mpc_goal/mpc_predicted_path so the probe has something live to
    # score, instead of only ever working off a bag or static_workspace_center.
    grasp_node = Node(
        package='capacitynet',
        executable='grasp',
        name='object_grasper',
        output='screen',
        parameters=[grasp_parameters],
        remappings=tf_remappings,
    )

    return LaunchDescription(declared + [workspace_probe_node, grasp_node])
