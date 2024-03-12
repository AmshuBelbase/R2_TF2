from launch import LaunchDescription 
from launch_ros.actions import Node 


def generate_launch_description(): 
    return LaunchDescription([  
        Node(
            package='my_r2_tf2',
            executable='get_cmd_vel',
            name='get_send_cmd_vel'
        )
    ])
