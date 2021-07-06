from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mass_config_file',
            default_value=['config.yaml'],
            description='massrobotics_amr_sender node configuration file'
        ),
        DeclareLaunchArgument(
            'bag_path',
            default_value=['./rosbag/rosbag_demo.db3'],
            description='Path for ROS 2 data bag'
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
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-l', LaunchConfiguration('bag_path')],
            output='screen',
            name='rosbag_demo'
        )
    ])
