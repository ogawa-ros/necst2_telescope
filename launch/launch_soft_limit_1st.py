from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2pkg_ogameasure',
            node_executable='checker',
            parameters=[
                {'az_upper_1st_limit': '350'},
                {'az_lower_1st_limit': '10'}
                {'el_upper_1st_limit': '80'}
                {'el_lower_1st_limit': '15'}
            ]
        )],
       )
