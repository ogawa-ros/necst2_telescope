from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='necst2_telescope',
            node_executable='lcoordinate_calc',
            parameters=[
                {}
            ]
        )],
       )
