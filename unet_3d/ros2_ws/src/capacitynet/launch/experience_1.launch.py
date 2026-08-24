#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# grasp.py and gradient_base_controller both talk TF over leeloo's namespaced
# topics rather than the default /tf, /tf_static.
TF_TOPIC = '/leeloo/tf'
TF_STATIC_TOPIC = '/leeloo/tf_static'


def generate_launch_description():
    grasp_node = Node(
        package='capacitynet',
        executable='grasp',
        name='grasp',
        output='screen',
        parameters=[{'grasp_distance': -0.15}],
        remappings=[
            ('/tf', TF_TOPIC),
            ('/tf_static', TF_STATIC_TOPIC),
        ],
    )

    gradient_base_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('capacitynet'),
                'launch', 'gradient_base_controller.launch.py',
            )
        ),
        launch_arguments={
            'tf_topic': TF_TOPIC,
            'tf_static_topic': TF_STATIC_TOPIC,
            'start_enabled': 'true',
            'grid_size': '3',
        }.items(),
    )

    return LaunchDescription([grasp_node, gradient_base_controller_launch])
