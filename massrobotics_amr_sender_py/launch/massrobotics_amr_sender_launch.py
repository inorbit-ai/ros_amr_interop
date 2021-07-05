from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='massrobotics_amr_sender',
            namespace='massrobotics_amr_sender',
            executable='massrobotics_amr_node',
            name='massrobotics_amr_sender',
            parameters=[
                {'config_file': 'sample_config.yaml'}
            ]
        )
    ])
