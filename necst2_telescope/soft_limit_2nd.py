#!/usr/bin/env python3

node_name = 'motor_locker'

import time
import rclpy
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int64MultiArray


class motor_locker(object):

    def __init__(self):
        self.node = rclpy.create_node(node_name)

        self.upper_2nd_limit = double(self.node.declare_parameter("upper_2nd_limit").value)
        self.lower_2nd_limit = double(self.node.declare_parameter("lower_2nd_limit").value)

        #topic_name = '/opu1p85m/'

        #self.pub_az_lock = self.node.create_publisher(Int64, topic_name+'az_lock_cmd', 1)
        #self.pub_el_lock = self.node.create_publisher(Int64, topic_name+'el_lock_cmd', 1)
        self.output_do = self.node.create_publisher(Int64, "/pyinterface/pci7415/rsw0/azel_12",1)




        self.node.create_subscription(Float64, '/dev/HEIDENHAIN/ND287/azel', self.recieve, 1)


    def recieve(self, q):
        self.azel = q.data
        if self.azel > self.upper_2nd_limit or self.azel < self.lower_2nd_limit:
            
            msg = Int64()
            msg.data = 0
            self.output_do1.publish(msg)
            print(self.upper_2nd_limit)
            print(self.lower_2nd_limit)
        else:
            print(self.upper_2nd_limit)
            print(self.lower_2nd_limit)
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
