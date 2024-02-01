import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = FindPackageShare(package='my_r2_robot').find('my_r2_robot')
    default_launch_dir = os.path.join(pkg_share, 'launch/my_r2.launch.py') 
    return LaunchDescription([  
        DeclareLaunchArgument(
            'target_frame', default_value='ball',
            description='Target frame name.'
        ), 
        Node(
            package='my_r2_tf2',
            executable='tf2_to_cmd_vel',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ),
        # Including another launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(default_launch_dir) 
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='my_static_transform_publisher',
            output='screen',
            arguments=['--x', '0', '--y', '4', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'r2', '--child-frame-id', 'ball']
        ),
        Node(
            package='my_r2_tf2',
            executable='get_cmd_vel',
            name='get_send_cmd_vel'
        )
    ])
