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

        self.az_upper_2nd_limit = self.node.get_parameter("az_upper_2nd_limit").double_value
        self.az_lower_2nd_limit = self.node.get_parameter("az_lower_2nd_limit").get_parameter_value().double_value
        self.el_upper_2nd_limit = self.node.get_parameter("el_upper_2nd_limit").get_parameter_value().double_value
        self.el_lower_2nd_limit = self.node.get_parameter("el_lower_2nd_limit").get_parameter_value().double_value

        topic_name = '/opu1p85m/'

        self.pub_az_lock = self.node.create_publisher(Bool, topic_name+'az_lock_cmd', 1)
        self.pub_el_lock = self.node.create_publisher(Bool, topic_name+'el_lock_cmd', 1)
        self.pub_west_river = self.node.create_publisher(Float64, "west", 1)

        self.node.create_subscription(Float64, '/dev/HEIDENHAIN/ND287/azz', self.recieve_az, 1)
        self.node.create_subscription(Float64, '/dev/HEIDENHAIN/ND287/ell', self.recieve_el, 1)


    def recieve_az(self, q):
        self.az = q.data
        if self.az > self.az_upper_2nd_limit or self.az < self.az_lower_2nd_limit:
            
            msg = Bool()
            msg.data = True
            self.pub_az_lock.publish(msg)
            print(self.az_upper_2nd_limit)
            print(self.az_lower_2nd_limit)
        else:
            msg = Bool()
            msg.data = False
            self.pub_az_lock.publish(msg)
            print(self.az_upper_2nd_limit)
            print(self.az_lower_2nd_limit)
            pass
        return

    def recieve_el(self, q):
        self.el = q.data
        if self.el > self.el_upper_2nd_limit or self.el < self.el_lower_2nd_limit:
            msg2 = Float64()
            msg2.data = self.el_upper_2nd_limit
            self.pub_west_river.publish(msg2)
            msg = Bool()
            msg.data = True
            self.pub_el_lock.publish(msg)
        else:
            msg = Bool()
            msg.data = False
            self.pub_el_lock.publish(msg)
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
