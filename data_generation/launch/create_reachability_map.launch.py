from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    voxel_size = LaunchConfiguration('voxel_size')
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value="0.08",  # Set the default value
    )


    # Node curobo_ik
    ik_node = Node(
        package='curobo_ros',
        executable='curobo_ik',
        name='curobo_ik',
        output='screen',
        # parameters=[
        #         {"voxel_size": voxel_size}
        #     ]
    )

    # Node data_gen create reachability map
    create_reachability_map = Node(
        package='data_generation',
        executable='create_reachability_map',
        name='create_reachability_map',
        output='screen',
        # parameters=[
        #         {"voxel_size": voxel_size,}
        # ],
        
    )

    return LaunchDescription([
        voxel_size_arg,
        # ik_node, 
        Node(
            package='curobo_ros',
            executable='curobo_ik',
            name='curobo_ik',
            output='screen',
            parameters=[
                    {"voxel_size": voxel_size}
                ]
        ),
        # create_reachability_map
        Node(
        package='data_generation',
        executable='create_reachability_map',
        name='create_reachability_map',
        output='screen',
        parameters=[
                {"voxel_size": voxel_size,}
        ])
        ])