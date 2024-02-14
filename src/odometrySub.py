import rclpy
from rclpy.node import Node



roll = pitch = yaw = 0.0



def main(args=None):
    # always run "init()" first
    rclpy.init()

    # let's catch some exceptions should they happen
    try:
        node = OdometryReceiver()

        # tell ROS to run this node until stopped (by [ctrl-c])
        rclpy.spin(node)

        # once stopped, tidy up
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print('Node interrupted')

    finally:
        # always print when the node has terminated
        print("Node terminated")


if __name__ == '__main__':
    main()