from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='massrobotics_amr_sender',
            namespace='massrobotics_amr_sender',
            executable='massrobotics_amr_node',
            name='massrobotics_amr_sender',
            parameters=[
                {'config_file': 'config.yaml'}
            ]
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', './rosbag/rosbag_demo.db3'],
            output='screen'
        )
    ])
