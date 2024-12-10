from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adaptive_pure_pursuit', 
            executable='adaptive_pure_pursuit_node',  
            name='adaptive_pure_pursuit',
            output='screen',
            arguments=[
                '--configuration_directory','/home/nvidia/ros2_ws/src/adaptive_pure_pursuit/config/control_params.yaml' 
            ]
        )
    ])
