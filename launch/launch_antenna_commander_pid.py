from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='necst2_telescope',
            node_executable='az_pid',
            parameters=[
                {'p_coeff': 1.5,
                'i_coeff': 0.0,
                'd_coeff': 0.0,
                'gear_ratio': 7200.0,
                'pulseper360deg': 65536.0,
                'pulse_a': 16384.0,
                'pulse_b': 2500.0,
                'MOTOR_MAXSTEP': 32767.5,
                'MOTOR_AZ_MAXSPEED': 320000.0},
            ]
        ), 
        Node(
            package='necst2_telescope',
            node_executable='el_pid',
            parameters=[
                {'p_coeff': 1.5,
                'i_coeff': 0.0,
                'd_coeff': 0.0,
                'gear_ratio': 7200.0,
                'pulseper360deg': 65536.0,
                'pulse_a': 16384.0,
                'pulse_b': 2500.0,
                'MOTOR_MAXSTEP': 32767.5,
                'MOTOR_AZ_MAXSPEED': 320000.0},
            ]
        )],
    )
