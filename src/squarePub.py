from time import sleep
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist 
# from nav_msgs.msg import Odometry

# class OdometrySubscriber(Node):
#     def __init__(self):
#         """ Initialise the Node. """
#         super().__init__('OdemetrySubscriber')  
        
#         self.create_subscription(Odometry, '/odom', self.get_rotation, 1)

#     def get_rotation (msg):
#         return msg

class SquarePublisher(Node):

    def __init__(self):
        super().__init__('squarePublisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        twistMove = Twist()
        twistTurn = Twist()
        
        twistMove.linear.x = 0.5
        twistTurn.angular.z = -1.57
        
        for i in range (0, 4):
            self.publisher_.publish(twistTurn)
            self.get_logger().info('Publishing: "%s"' % twistTurn)
            sleep(0.25)
        self.publisher_.publish(twistMove)
        self.get_logger().info('Publishing: "%s"' % twistMove)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    squarePublisher = SquarePublisher()
    # odometrySubscriber = OdometrySubscriber()

    # rclpy.spin(odometrySubscriber)
    rclpy.spin(squarePublisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    squarePublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()