from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_node(
        Node(
            package = 'necst2_telescope',
            executable = 'exec_antenna_commander',
            remappings = [
                ('azel', 'az'),
                ('azel_xy', 'y'),
                ('azel_12', 'output_do1_cmd'),
            ],
            parameters = [{
                # PID
                'p_coeff': 1.5,
                'i_coeff': 0,
                'd_coeff': 0,
                'gear_ratio': 7200,
                'pulsepar360deg': 65536,
                'pulse_a': 16384,
                'pulse_b': 2500,
                'MOTORMAXSTEP': 32767.5,
                'MOTORMAXSPEED': 320000,
                # 1st limit
                'upper_1st_limit': 350,
                'lower_1st_limit': 10,
                # 2nd limit
                'upper_2nd_limit': 355,
                'lower_2nd_limit': 5,
                }],
            ))
    ld.add_node(
        Node(
            package = 'necst2_telescope',
            executable = 'exec_antenna_commander',
            remappings = [
                ('azel', 'el'),
                ('azel_xy', 'x'),
                ('azel_12', 'output_do2_cmd'),
            ],
            parameters = [{
                # PID
                'p_coeff': 1.5,
                'i_coeff': 0,
                'd_coeff': 0,
                'gear_ratio': 7200,
                'pulsepar360deg': 65536,
                'pulse_a': 16384,
                'pulse_b': 2500,
                'MOTORMAXSTEP': 32767.5,
                'MOTORMAXSPEED': 320000,
                # 1st limit
                'upper_1st_limit': 80,
                'lower_1st_limit': 15,
                # 2nd limit
                'upper_2nd_limit': 85,
                'lower_2nd_limit': 10,
                }],
            ))
    return ld
