#!/usr/bin/env python3

from .antenna_commander_pid import antenna_commander_pid
from .soft_limit_1st import checker
from .soft_limit_2nd import motor_locker

import rclpy
from rclpy.executors import SingleThreadedExecutor


def main(args=None):
    rclpy.init(args=args)
    try:
        nodes = {
            "pid": antenna_commander_pid(),
            "soft1st": checker(),
            "soft2nd": motor_locker(),
        }

        executor = SingleThreadedExecutor()

        [executor.add_node(node) for node in nodes.values()]

        try:
            executor.spin()
        finally:
            executor.shutdown()
            [node.destroy_node() for node in nodes.values()]
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
