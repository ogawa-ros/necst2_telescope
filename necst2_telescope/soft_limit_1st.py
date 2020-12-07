#!/usr/bin/env python3

node_name = 'checker'

import rclpy
from std_msgs.msg import Bool
from std_msgs.msg import Float64

class checker(object):

    def __init__(self):
        self.node = rclpy.create_node(node_name)

        self.upper_1st_limit = double(self.node.declare_parameter("upper_1st_limit").value)
        self.lower_1st_limit = double(self.node.declare_parameter("lower_1st_limit").value)


        topic_name = '/opu1p85m'

        self.pub_flag = self.node.create_publisher(Bool, topic_name+'/azel/soft_limit', 1)
        self.pub_cmd2 = self.node.create_publisher(Float64, topic_name+'/azel/cmd2', 1)

        self.node.create_subscription(Float64, topic_name+'/azel/cmd', self.check_az, 1)


    def check(self, q):
        self.azel = q.data
        if self.azel > self.upper_1st_limit:
            msg = Float64()
            msg.data = self.upper_1st_limit
            msg2 = Bool()
            msg2.data = True
            self.pub_cmd2.publish(msg)
            self.pub_flag.publish(msg2)

        elif self.azel < self.lower_1st_limit:
            msg = Float64()
            msg.data = self.lower_1st_limit
            msg2 = Bool()
            msg2.data = True
            self.pub_cmd2.publish(msg)
            self.pub_flag.publish(msg2)

        else:
            msg = Float64()
            msg.data = self.azel
            msg2 = Bool()
            msg2.data = False
            self.pub_cmd2.publish(msg)
            self.pub_flag.publish(msg2)
            pass

        return

def main(args = None):
    rclpy.init(args = args)
    check = checker()
    rclpy.spin(check.node)

    check.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
