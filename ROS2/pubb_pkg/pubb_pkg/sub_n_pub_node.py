import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SubNpub(Node):

    def __init__(self):
        super().__init__('sub_pub_node')
        self.subscription = self.create_subscription(String,'print_topic',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.Msg_sub="q"
    
        self.publisher_ = self.create_publisher(String, 'sub_n_pub_topic', 10)
    
    def listener_callback(self, msg):
        # self.get_logger().info('I hear: "%s"' % msg.data)
        self.Msg_sub= msg.data

        msg.data=msg.data+" pub again"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    Sub_pub = SubNpub()
    print("sub_N_pub_start")
    rclpy.spin(Sub_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Sub_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()