from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    spot_to_file_server = Node(
            package='spot_recorder',
            executable='server',
            name='server',
            output='screen'
        )
        
    return LaunchDescription([
        spot_to_file_server,
    ])