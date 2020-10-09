import rclpy
from std_msgs.msg import Float64
import time

class AzEl_Publisher(object):

    def __init__(self):
        self.node = rclpy.create_node('AzEl_publisher')
        self.az_pub = self.node.create_publisher(Float64, '/opu1p85m/az_cmd', 1)
        self.el_pub = self.node.create_publisher(Float64, '/opu1p85m/el_cmd', 1)
        
        timer_period = 0.1  # seconds
        self.node.create_timer(timer_period, self.clock)

    def clock(self):
        az = Float64()
        el = Float64()
        az.data = 50.5  #Az parameter
        el.data = 45.0   #El parameter
        self.az_pub.publish(az)
        self.el_pub.publish(el)


def main(args=None):
    rclpy.init(args=args)
    azel = AzEl_Publisher()
    rclpy.spin(azel.node)

    azel.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

