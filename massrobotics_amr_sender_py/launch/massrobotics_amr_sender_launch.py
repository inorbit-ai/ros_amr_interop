from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mass_config_file',
            default_value=['../sample_config.yaml'],
            description='massrobotics_amr_sender node configuration file'
        ),
        Node(
            package='massrobotics_amr_sender',
            namespace='massrobotics_amr_sender',
            executable='massrobotics_amr_node',
            name='massrobotics_amr_sender',
            parameters=[
                {'config_file': LaunchConfiguration('mass_config_file') }
            ]
        ),
    ])
