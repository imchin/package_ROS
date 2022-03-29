#!/usr/bin/env python3
from curses import use_env
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,UInt16
from turtlesim.msg import Pose as tPose
from std_srvs.srv import Empty, EmptyResponse

class Node :				# for storing and passing variables
    def __init__(self):
        self.goal_position = np.array([0,0])
        self.current_position = np.array([0,0])
        self.current_orientation = 0
        self.cmd_vel_msg = Twist()
    def callback_current_pose(self,msg):	# callback for subscriber
        self.current_position  = np.array([msg.x,msg.y])
        self.current_orientation = msg.theta
    def control(self):
        dp = self.goal_position-self.current_position
        v = 1
        if np.linalg.norm(dp)<0.1:
            v = 0
        e = math.atan2(dp[1],dp[0])-self.current_orientation
        K = 10
        w = K*math.atan2(math.sin(e),math.cos(e))
        return v,w
    def publish_cmd_vel(self):			# publish
        dp = self.goal_position-self.current_position 
        v,w = self.control()
        self.cmd_vel_msg.linear.x = v
        self.cmd_vel_msg.angular.z = w
        pub_cmd.publish(self.cmd_vel_msg)
        rate.sleep()    

    def service_rand_goal(self,req):		# service method
        self.goal_position = np.random.rand(2)*10
        print(self.goal_position)
        return EmptyResponse()    

if __name__=='__main__':
    rospy.init_node('rand_tao1')
    node = Node()
    srv_rand_goal = rospy.Service('/rand_goal',Empty,node.service_rand_goal)
    rospy.Subscriber('/turtle1/pose',tPose,node.callback_current_pose)
    pub_cmd = rospy.Publisher('turtle1/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.publish_cmd_vel()