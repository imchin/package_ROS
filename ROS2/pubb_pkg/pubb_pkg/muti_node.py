from os import waitid
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from turtlesim.msg import Pose
import numpy as np
from std_srvs.srv import Empty
from long_interfaces.msg import Num
from long_interfaces.action import Posetogo

import time
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ThreadOne(Node):

    def __init__(self):
        super().__init__('ThreadOne')
        self.A ="3.14 "
        self.publisher_ = self.create_publisher(String, 'test_pub_muti', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'ThreadOne grab data :' + str(self.A)
        self.publisher_.publish(msg)



    
          
class ThreadTwo(Node):
    def __init__(self,tt):
        super().__init__('ThreadTwo')
        self.tt = tt
        self.subscription = self.create_subscription(String,'print_topic',self.listener_callback,10)

    def listener_callback(self, msg):
      
        self.tt.A =  msg.data

        






def main(args=None):
    rclpy.init(args=args)
    try:
        threadOnee= ThreadOne()
        threadTwoo= ThreadTwo(tt=threadOnee)
    
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(threadOnee)
        executor.add_node(threadTwoo)
        try:
            executor.spin()
        finally:
            executor.shutdown()
          
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()