from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2pkg_ogameasure',
            node_executable='limit_2nd',
            parameters=[
                {'az_upper_2nd_limit': 355},
                {'az_lower_2nd_limit': 5},
                {'el_upper_2nd_limit': 85},
                {'el_lower_2nd_limit': 10}
            ]
        )],
       )
