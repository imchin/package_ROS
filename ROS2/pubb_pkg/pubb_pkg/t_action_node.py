from enum import Flag
from os import waitid
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from turtlesim.msg import Pose
import numpy as np
from long_interfaces.action import Posetogo

import time
import math

from geometry_msgs.msg import Twist


class TurtlesimControlActionServer(Node):

    def __init__(self):
        super().__init__('turtlesim_control_action_server')
        self.action_server = ActionServer(self,Posetogo,'pose_to_go',self.execute_callback)
        
        self.Flag = 0

        self.current_position=np.array([0,0])
        self.current_orientation = 0
        self.goal_position = np.array([0,0])

        self.ENAcontrol = 0 
    def execute_callback(self, goal_handle):
        init_time = time.time()
        self.get_logger().info('Executing action...')


        self.goal_position[0]=goal_handle.request.x
        self.goal_position[1]=goal_handle.request.y

        self.get_logger().info('turtle going to X: ' + str(self.goal_position[0]) +" Y : "+ str(self.goal_position[1]) )

    
        feedback_msg = Posetogo.Feedback()
        
        self.ENAcontrol = 1
        self.Flag =0
        self.get_logger().info("wait 0.2 sec for control loop")
        time.sleep(0.2)
        self.get_logger().info("wait Done")
        if(self.Flag==0):
            self.get_logger().info("Going to goal")
        else:
            self.ENAcontrol = 0
            self.get_logger().info("On goal")

        while(self.Flag != 1):
            dp = self.goal_position-np.array([self.current_position[0],self.current_position[1]])
            dist = np.linalg.norm(dp)
            feedback_msg.distance = dist
            # self.get_logger().info('Feedback: {0}'.format(feedback_msg.distance))
            goal_handle.publish_feedback(feedback_msg)
            # self.get_logger().info(str(self.current_position[0]))
            time.sleep(0.5)

        self.ENAcontrol = 0
           

        goal_handle.succeed()
        result = Posetogo.Result()
        result.total_time =  time.time()  - init_time
        self.get_logger().info("Done task.")
        return result


    
          
class TurtlesimWatchdog(Node):
    def __init__(self,actionServer):
        super().__init__('turtlesim_Watchdog')
        self.Acs = actionServer
        self.subscription = self.create_subscription(Pose,'/turtle1/pose',self.callback_current_pose,10)


        # routine 0.1 sec
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # end routine 0.1 sec

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

       
    def callback_current_pose(self,msg):	# callback for subscriber

        # self.get_logger().info(str(msg.x))

        self.Acs.current_position[0]= msg.x
        self.Acs.current_position[1]= msg.y
        self.Acs.current_orientation= msg.theta 


    def timer_callback(self):
        v,w = self.control()
        tw=Twist()   # obj twist
        tw.linear.x = float(v)
        tw.angular.z = float(w)
        if(self.Acs.ENAcontrol):
            self.publisher_.publish(tw)

    def control(self):
        dp = self.Acs.goal_position-self.Acs.current_position
        v = 1
        if np.linalg.norm(dp)<0.1:
            v = 0
            self.Acs.Flag=1  # done task
            w=0
        else:    
            e = math.atan2(dp[1],dp[0])-self.Acs.current_orientation
            K = 10
            w = K*math.atan2(math.sin(e),math.cos(e))
        return v,w


def main(args=None):
    rclpy.init(args=args)
    try:
        turtlesim_control_action_server = TurtlesimControlActionServer()
        turtlesim_Watchdog = TurtlesimWatchdog(actionServer=turtlesim_control_action_server)
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(turtlesim_control_action_server)
        executor.add_node(turtlesim_Watchdog)
        try:
            executor.spin()
        finally:
            executor.shutdown()
           
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()