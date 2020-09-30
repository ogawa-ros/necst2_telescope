from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2pkg_ogameasure',
            node_executable='motor_locker',
            parameters=[
                {'az_upper_2nd_limit': ''},
                {'az_lower_2nd_limit': ''}
                {'el_upper_2nd_limit': ''}
                {'el_lower_2nd_limit': ''}
            ]
        )],
       )
