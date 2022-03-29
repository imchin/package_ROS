#!/usr/bin/env python3
from curses import use_env
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,UInt16
from turtlesim.msg import Pose as tPose
from std_srvs.srv import Empty, EmptyResponse

class Node :
    def __init__(self):
        self.goal_position = np.array([0,0])

        self.current_position_tao1 = np.array([0,0])
        self.current_orientation_tao1 = 0

        self.current_position_tao2= np.array([0,0])
        self.current_orientation_tao2 = 0

        self.cmd_vel_msg = Twist()

        self.timer = None
        
        self.speed=0.5

        self.B1state=0
        self.B2state=0
        self.B3state=0
        self.flagpause=1
        self.flagrand=-1
        self.goal_position_rand=np.array([0,0])

    # get currunt pose
    def callback_current_pose_tao1(self,msg):
        self.current_position_tao1  = np.array([msg.x,msg.y])
        self.current_orientation_tao1 = msg.theta

    def callback_current_pose_tao2(self,msg):
        self.current_position_tao2  = np.array([msg.x,msg.y])
        self.current_orientation_tao2= msg.theta

    def callback_poten(self,msg):
        if(msg.data<=450):
            usee=450
        elif(msg.data>=400):
            usee=500
        else:
            usee=msg.data
        self.speed=(usee-400+1)/50

    def callback_B1(self,msg):
        if(self.B1state!=msg.data):
            self.B1state=msg.data
            rospy.ServiceProxy('clear', Empty)()


    def callback_B2(self,msg):
        if(self.B2state!=msg.data):
            self.B2state=msg.data
            if(self.flagrand>0):
               self.flagpause=self.flagpause*-1
               self.flagrand=self.flagrand*-1
            else:
                self.flagpause=self.flagpause*-1

    def callback_B3(self,msg):
        if(self.B3state!=msg.data):
            self.B3state=msg.data
            if(self.flagpause<0):
                self.flagpause=1
                self.rand_goal()
                self.flagrand=1
            else:    
                self.rand_goal()
                self.flagrand=self.flagrand*-1
           

    def control(self):
        if(self.flagrand>0):
            dp = self.goal_position-self.current_position_tao2
            v =self.speed
        
            if np.linalg.norm(dp)<0.1:
                v = 0
            e = math.atan2(dp[1],dp[0])-self.current_orientation_tao2
            K = 10
            w = K*math.atan2(math.sin(e),math.cos(e))
        else:
            dp = self.goal_position-self.current_position_tao2- np.array([2,2])*np.array([math.cos(self.current_orientation_tao1),math.sin(self.current_orientation_tao1)])
            v =self.speed
        
            if np.linalg.norm(dp)<0.1:
                v = 0
            e = math.atan2(dp[1],dp[0])-self.current_orientation_tao2
            K = 10
            w = K*math.atan2(math.sin(e),math.cos(e))
        return v,w

    def target_follower(self,event):
       
        rospy.Subscriber('/turtle1/pose',tPose,self.callback_current_pose_tao1)
        rospy.Subscriber('/turtle2/pose',tPose,self.callback_current_pose_tao2)

        rospy.Subscriber('/Arduino/poten',UInt16,self.callback_poten)
        rospy.Subscriber('/Arduino/button1',UInt16,self.callback_B1)
        rospy.Subscriber('/Arduino/button2',UInt16,self.callback_B2)
        rospy.Subscriber('/Arduino/button3',UInt16,self.callback_B3)
 

        if(self.flagpause>0):
            if(self.flagrand>0):
                self.goal_position=self.goal_position_rand
            else:
                self.goal_position = np.array([self.current_position_tao1[0],self.current_position_tao1[1]])
            dp = self.goal_position-self.current_position_tao2
            v,w = self.control()
            self.cmd_vel_msg.linear.x = v
            self.cmd_vel_msg.angular.z = w
            pub_cmd.publish(self.cmd_vel_msg)

    def callback_shutDownTimer(self):
        self.timer.shutdown()

    def rand_goal(self):		# service method
        self.goal_position_rand = np.random.rand(2)*10
        # print(self.goal_position)
        # return EmptyResponse()    

        

    
if __name__=='__main__':
    rospy.init_node('follow_timer')
    node = Node()
    
    
    
    pub_cmd = rospy.Publisher('/turtle2/cmd_vel',Twist,queue_size=10)
    node.timer = rospy.Timer(rospy.Duration(1/10), node.target_follower)

    
    # rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()





