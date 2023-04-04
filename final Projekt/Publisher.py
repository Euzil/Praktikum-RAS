import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from tictactoe import *


class TicTacToe(Node):

    def __init__(self):
        super().__init__('tictactoe_publisher')
        self.publisher_ = self.create_publisher(int, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)



    def timer_callback(self):
        msg = int()
        msg.data = TicTacToe.compMove().bestMove
        self.publisher_.publish(msg)
        self.get_logger().info('Move: "%s"' % msg.data)
        self.i += 1



def main(args=None):
    rclpy.init(args=args)

    TicTacToe_publisher = TicTacToePublisher()

    rclpy.spin(TicTacToeT_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    TicTacToe_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
