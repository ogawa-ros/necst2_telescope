from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="necst2_telescope",
                node_executable="coordinate_calc",
                parameters=[{}],
            )
        ],
    )
