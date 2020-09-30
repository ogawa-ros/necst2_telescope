#!/usr/bin/env python3

node_name = 'checker'

import rclpy
from std_msgs.msg import Bool
from std_msgs.msg import Float64

class checker(object):

    def __init__(self):
        self.node = rclpy.create_node(node_name)

        self.node.declare_parameter("az_upper_1st_limit")
        self.node.declare_parameter("az_lower_1st_limit")
        self.node.declare_parameter("el_upper_1st_limit")
        self.node.declare_parameter("el_lower_1st_limit")

        self.az_upper_1st_limit = self.node.get_parameter("az_upper_1st_limit").get_parameter_value().string_value
        self.az_lower_1st_limit = self.node.get_parameter("az_lower_1st_limit").get_parameter_value().string_value
        self.el_upper_1st_limit = self.node.get_parameter("el_upper_1st_limit").get_parameter_value().string_value
        self.el_lower_1st_limit = self.node.get_parameter("el_lower_1st_limit").get_parameter_value().string_value


        topic_name = '/1p85m/'

        self.pub_az_flag = rclpy.node.create_publisher(Bool, topic_name+'az_soft_limit', 1)
        self.pub_el_flag = rclpy.node.create_publisher(Bool, topic_name+'el_soft_limit', 1)
        self.pub_az_cmd2 = rclpy.node.create_publisher(Float64, topic_name+'az_cmd2', 1)
        self.pub_el_cmd2 = rclpy.node.create_publisher(Float64, topic_name+'el_cmd2', 1)

        self.node.create_subscription(Float64, topic_name+'az_cmd', self.check_az, 1)
        self.node.create_subscription(Float64, topic_name+'el_cmd', self.check_el, 1)


    def check_az(self, q):
        self.az = q.data
        if self.az > float(self.az_upper_1st_limit):
            self.pub_az_cmd2.publish(float(self.az_upper_1st_limit))
            self.pub_az_flag.publish(True)

        elif self.az < float(self.az_lower_1st_limit):
            self.pub_az_cmd2.publish(float(self.az_lower_1st_limit))
            self.pub_az_flag.publish(True)

        else:
            self.pub_az_cmd2.publish(self.az)
            self.pub_az_flag.publish(False)
            pass

        return

    def check_el(self, q):
        self.el = q.data
        if self.el > float(self.el_upper_1st_limit):
            self.pub_el_cmd2.publish(float(self.el_upper_1st_limit))
            self.pub_el_flag.publish(True)

        elif self.el < float(self.el_lower_1st_limit):
                self.pub_el_cmd2.publish(float(self.el_lower_1st_limit))
                self.pub_el_flag.publish(True)

        else:
            self.pub_el_cmd2.publish(self.el)
            self.pub_el_flag.publish(False)
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
