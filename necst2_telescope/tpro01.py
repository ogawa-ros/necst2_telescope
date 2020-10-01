import rclpy
from std_msgs.msg import Float64
import time

class AzEl_Publisher(object):

    def __init__(self):
        self.node = rclpy.create_node('AzEl_publisher')
        self.az_pub = self.node.create_publisher(Float64, '/1p85m/az', 1)
        self.el_pub = self.node.create_publisher(Float64, '/1p85m/el', 1)
        
        timer_period = 1  # seconds
        self.node.create_timer(timer_period, self.clock)

    def clock(self):
        az = Float64()
        el = Float64
        az.data = 180  #Az parameter
        el.data = 45   #El parameter
        self.az_pub.publish(az)
        self.el_pub.publish(el)


def main(args=None):
    rclpy.init(args=args)
    AzEl_publisher = AzEl_Publisher()
    rclpy.spin(AzEl_publisher.node)

    AzEl_publisher.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

