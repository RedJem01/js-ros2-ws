import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist 


class CirclePublisher(Node):

    def __init__(self):
        super().__init__('circlePublisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 3.0
        self.publisher_.publish(twist)
        self.get_logger().info('Publishing: "%s"' % twist)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    circlePublisher = CirclePublisher()

    rclpy.spin(circlePublisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    circlePublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()