from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='necst2_telescope',
            node_executable='limit_2nd',
            parameters=[
                {'az_upper_2nd_limit': 355.0,
                'az_lower_2nd_limit': 5.0,
                'el_upper_2nd_limit': 85.0,
                'el_lower_2nd_limit': 10.0}
            ]
        )],
       )
