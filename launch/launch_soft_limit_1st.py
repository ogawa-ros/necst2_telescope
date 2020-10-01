from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='necst2_telescope',
            node_executable='limit_1st',
            parameters=[
                {'az_upper_1st_limit': 350.0},
                {'az_lower_1st_limit': 10.0},
                {'el_upper_1st_limit': 80.0},
                {'el_lower_1st_limit': 15.0}
            ]
        )],
       )
