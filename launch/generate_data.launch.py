from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the curobo_ik node
        Node(
            package='curobo_ros',
            executable='curobo_ik',
            name='curobo_ik',
            output='screen'
        ),
        # Launch the obstacle_adder node
        Node(
            package='data_generation',
            executable='obstacle_adder',
            name='obstacle_adder',
            output='screen'
        ),
        # Launch the generate_data node
        Node(
            package='data_generation',
            executable='generate_data',
            name='generate_data',
            output='screen'
        ),
        Node(
            package='data_generation',
            executable='scene_manager',
            name='scene_manager',
            output='screen'
        ),
    ])
