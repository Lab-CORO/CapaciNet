#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (name, default, description) — kept as a table because the node has many knobs
# and declaring each one by hand triples the length of this file.
ARGUMENTS = [
    ('debug_csv_path', "/home/ros2_ws/src/control_debug.csv", 'csv for debug'),
    ('debug_gradient_html_path', '',
     'Interactive Plotly HTML of the explored (goal_x, goal_y, Q_center) trace, '
     'one point per cycle. Empty disables accumulation; when set, call the '
     '~/save_gradient_exploration service (std_srvs/Trigger) to write it'),
    ('voxel_grid_topic', '/curobo_trajectory_planner/voxel_grid_sparse',
     'curobo_msgs/SparseVoxelGrid topic driving the control loop'),
    ('cycle_period', '2.0',
     'Seconds between inference cycles; the voxel grid subscription only caches '
     'the latest message, a timer runs the cycle on it at this fixed cadence'),
    ('marker_topic', '/fiducial_markers',
     'FiducialMarkerArray topic providing the fallback workspace center'),
    ('goal_topic', '/curobo_trajectory_planner/mpc_goal',
     'geometry_msgs/Pose grasp target published by grasp.py; primary region center'),
    ('path_topic', '/mpc_predicted_path',
     'nav_msgs/Path whose tail is unioned with the goal sphere'),
    ('path_tail_samples', '4',
     'Number of path poses (from the end) added to the scored region; 0 = goal only'),
    ('planning_frame', 'dsr01/base_link',
     'Frame assumed for goal_topic, which carries no header'),
    ('cmd_vel_topic', '/cmd_vel',
     'geometry_msgs/Twist topic commanding the mobile base'),
    ('target_marker_id', '-1',
     'Marker id used as workspace center; -1 = first marker in the array'),
    ('model_config_path', '/home/ros2_ws/src/test.yaml',
     'Path to the reachability model YAML config'),
    ('fp16', 'false',
     'Run the PyTorch fallback in half precision (ignored when a TRT engine loads)'),

    ('workspace_radius', '0.10',
     'Goal sphere radius in meters (region center 0)'),
    ('path_radius', '0.10',
     'Path-tail sphere radius in meters (region centers 1+). Defaults to the '
     'same value as workspace_radius'),
    ('ee_frame', 'dsr01/link_6',
     'TF frame rigidly attached to the arm TCP/end-effector; base stops once it '
     'enters the workspace region. Empty string disables the interlock'),
    ('arm_stop_margin', '0.05',
     'Hysteresis band in meters added to workspace_radius before the '
     'arm-in-workspace interlock releases'),
    ('grid_spacing', '0.1',
     'Grid spacing delta in meters; also bounds travel between gradient updates'),
    ('grid_size', '3',
     'Number of candidate base positions per grid side (must be odd, >= 3); '
     'total candidates = grid_size**2 (3 -> 9, 5 -> 25, 7 -> 49). Cost scales '
     'linearly with candidate count — see OPTIMIZATIONS.md'),

    ('base_speed', '0.02',
     'Commanded speed ceiling in m/s (hardware-validated maximum)'),
    ('step_fraction', '0.5',
     'Fraction of delta the base may travel per control cycle; sets the adaptive cap'),
    ('min_speed', '0.005',
     'Below this commanded speed the node declares convergence and publishes zero'),
    ('control_gain', '1.0',
     'Proportional gain k in v = k * grad Q'),
    ('gradient_taper_ref', '0.0',
     'Reference |grad Q| for the quadratic taper near the optimum; 0 disables it'),
    ('gradient_method', 'least_squares',
     "Gradient stencil: 'least_squares' (all grid_size**2 maps) or 'central' "
     '(4 nearest neighbors of center, regardless of grid_size)'),

    ('use_static_obstacles', 'false',
     'Load world-fixed obstacles from YAML so they are not translated with the base'),
    ('static_obstacles_yaml', '/home/ros2_ws/src/capacitynet/config/floor_world.yml',
     'Path to the static obstacles YAML'),

    ('publish_rate', '10.0',
     'Rate in Hz at which /cmd_vel is (re)published by the watchdog'),
    ('command_timeout', '2.10',
     'Seconds without a reachability update before the base is stopped'),
    ('marker_timeout', '2.0',
     'Seconds without a marker detection before the base is stopped'),
    ('mpc_timeout', '2.0',
     'Seconds without an MPC goal/path before falling back to the marker'),

    ('start_enabled', 'false',
     'Start commanding immediately instead of waiting for ~/enable'),
    ('grasper_state_topic', '/object_grasper/state',
     'Grasp state topic; any non-idle state blocks motion. Empty string disables the interlock'),
    ('grasper_allowed_states', 'idle,act_move_pregrasp',
     'Comma-separated grasper states that permit base motion. Stops at '
     'act_move_pregrasp because grasp.py only actively corrects for a moving '
     'target up to that state; from act_open_gripper onward the arm holds its '
     'last pose uncorrected'),
    ('static_workspace_center', '',
     'Fixed workspace center "x,y,z" in the voxel grid frame, overriding both the '
     'MPC goal and the marker. Required for bag replay, since no recorded bag '
     'contains /fiducial_markers or /mpc_predicted_path'),

    ('log_timing', 'true',
     'Log gradient, velocity and cycle time each control cycle'),
    ('log_quality_scores', 'false',
     'Log the grid_size x grid_size block of quality scores each control cycle'),
    ('publish_debug_markers', 'true',
     'Publish the grid_size**2 candidate scores and the gradient arrow as RViz markers'),
]

# Topic remappings, not node parameters: WorkspaceRegionSource's TransformListener
# subscribes to the plain 'tf'/'tf_static' topics, which some robots publish under
# a namespace (e.g. /leeloo/tf) instead of the default /tf, /tf_static.
TF_REMAP_ARGUMENTS = [
    ('tf_topic', '/tf', 'Topic carrying tf2_msgs/TFMessage for dynamic transforms'),
    ('tf_static_topic', '/tf_static', 'Topic carrying tf2_msgs/TFMessage for static transforms'),
]


def generate_launch_description():
    """Launch the reachability-gradient mobile base controller."""

    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS + TF_REMAP_ARGUMENTS
    ]

    parameters = {name: LaunchConfiguration(name) for name, _, _ in ARGUMENTS}

    gradient_base_controller_node = Node(
        package='capacitynet',
        executable='gradient_base_controller',
        name='gradient_base_controller',
        output='screen',
        parameters=[parameters],
        remappings=[
            ('tf', LaunchConfiguration('tf_topic')),
            ('tf_static', LaunchConfiguration('tf_static_topic')),
        ],
    )

    return LaunchDescription(declared + [gradient_base_controller_node])
