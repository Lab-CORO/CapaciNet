from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    voxel_size = LaunchConfiguration('voxel_size')
    init_batch_size = LaunchConfiguration('init_batch_size')
    dataset_size = LaunchConfiguration('dataset_size')
    batch_size = LaunchConfiguration('batch_size')
    reach_max = LaunchConfiguration('reach_max')
    obj_max = LaunchConfiguration('obj_max')
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value="0.02",  # Set the default value
    )
    init_batch_size_arg = DeclareLaunchArgument(
        'init_batch_size',
        default_value="1000",  # Set the default value
    )
    dataset_size_arg = DeclareLaunchArgument(
        'dataset_size',
        default_value="5",  # Set the default value
    )
    batch_size_arg = DeclareLaunchArgument(
        'batch_size',
        default_value="1000",  # Set the default value
    )
    reach_max_arg = DeclareLaunchArgument(
        'reach_max',
        default_value="1.3",  # Set the default value
    )
    obj_max_arg = DeclareLaunchArgument(
        'obj_max',
        default_value= "20",  # Set the default value
    )
    return LaunchDescription([
        init_batch_size_arg,
        voxel_size_arg,
        dataset_size_arg,
        batch_size_arg,
        reach_max_arg,
        obj_max_arg,
        # Launch the curobo_ik node
        Node(
            package='curobo_ros',
            executable='curobo_ik',
            name='curobo_ik',
            output='screen',
            parameters=[
                {"voxel_size": voxel_size,
                "init_batch_size":init_batch_size}
            ]
        ),
        # Launch the obstacle_adder node
        Node(
            package='data_generation',
            executable='obstacle_adder',
            name='obstacle_adder',
            output='screen',
        ),
        # Launch the generate_data node
        Node(
            package='data_generation',
            executable='generate_data',
            name='generate_data',
            output='screen',
            parameters=[
                {"voxel_size": voxel_size,
                "dataset_size": dataset_size,
                "batch_size": batch_size,
                "reach_max": reach_max,
                "obj_max": obj_max,
                }
            ]
        ),
        Node(
            package='data_generation',
            executable='scene_manager',
            name='scene_manager',
            output='screen',
            parameters=[
                {"voxel_size": voxel_size,
                "dataset_size": dataset_size,
                "batch_size": batch_size,
                "reach_max": reach_max,
                "obj_max": obj_max,
                }
            ]
        ),
    ])
