# import os
from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # pkg_share = FindPackageShare(package='my_r2_robot').find('my_r2_robot')
    # default_launch_dir = os.path.join(pkg_share, 'launch/my_r2.launch.py') 
    return LaunchDescription([  
        Node(
            package='my_r2_tf2',
            executable='get_cmd_vel',
            name='get_send_cmd_vel'
        )
    ])
