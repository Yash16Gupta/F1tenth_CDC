import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_localization',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '--configuration_directory', '/home/nvidia/test_ws/src/ugv02_cartographer/config',
                '--configuration_basename', 'localization.lua',
                '--load_state_filename', '/home/nvidia/test_ws/final_test.pbstream'
            ],
            remappings=[('/imu', '/imu/data')],
        ),
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])
