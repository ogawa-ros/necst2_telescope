#!/usr/bin/env python3

node_name = 'motor_locker'

import time
import rclpy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int64MultiArray


class motor_locker(object):

    def __init__(self):
        self.node = rclpy.create_node(node_name)

        self.node.declare_parameter("az_upper_2nd_limit")
        self.node.declare_parameter("az_lower_2nd_limit")
        self.node.declare_parameter("el_upper_2nd_limit")
        self.node.declare_parameter("el_lower_2nd_limit")

        self.az_upper_2nd_limit = self.node.get_parameter("az_upper_2nd_limit").get_parameter_value().string_value
        self.az_lower_2nd_limit = self.node.get_parameter("az_lower_2nd_limit").get_parameter_value().string_value
        self.el_upper_2nd_limit = self.node.get_parameter("el_upper_2nd_limit").get_parameter_value().string_value
        self.el_lower_2nd_limit = self.node.get_parameter("el_lower_2nd_limit").get_parameter_value().string_value

        topic_name = '/opu1p85m/'

        self.pub_az_lock = rclpy.node.create_publisher(Bool, topic_name+'az_lock_cmd', 1)
        self.pub_el_lock = rclpy.node.create_publisher(Bool, topic_name+'el_lock_cmd', 1)

        self.node.create_subscription(Float64, topic_name+'az', self.recieve_az, 1)
        self.node.create_subscription(Float64, topic_name+'el', self.recieve_el, 1)


    def recieve_az(self, q):
        self.az = q.data
        if self.az > float(self.az_upper_2nd_limit) or self.az < float(self.az_lower_2nd_limit):
            msg = Bool()
            msg.data = True
            self.pub_az_lock.publish(msg)
        else:
            pass
        return

    def recieve_el(self, q):
        self.el = q.data
        if self.el > float(self.el_upper_2nd_limit) or self.el < float(self.el_lower_2nd_limit):
            msg = Bool()
            msg2.data = True
            self.pub_el_lock.publish(msg)
        else:
            pass
        return

def main(args = None):
    rclpy.init(args = args)
    lock = motor_locker()
    rclpy.spin(lock.node)

    lock.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
