#!/usr/bin/env python3
import math
from xml.etree.ElementTree import QName
import numpy as np
import rospy

 
from std_msgs.msg import UInt8
from turtlesim.msg import Pose as tPose
from std_srvs.srv import Empty, EmptyResponse
class Node :
    def __init__(self):
        self.goal_position = np.array([0,0])

        self.current_position_tao1 = np.array([0,0])
        self.current_orientation_tao1 = 0

        self.current_position_tao2= np.array([0,0])
        self.current_orientation_tao2 = 0
 
        self.cmd_vel_msg = int()

        self.timer = None

    def callback_current_pose_tao1(self,msg):
        self.current_position_tao1  = np.array([msg.x,msg.y])
        self.current_orientation_tao1 = msg.theta

    def arr(self,event):
        rospy.Subscriber('/turtle1/pose',tPose, self.callback_current_pose_tao1)
        q=(self.current_orientation_tao1 *180/3.14)/2

        pub_cmd.publish(int(q%180))

    def callback_shutDownTimer(self):
        self.timer.shutdown()


        

    
if __name__=='__main__':
    rospy.init_node('m_Arduino')
    node = Node()
    
    
    pub_cmd = rospy.Publisher('Arduino/cmd_sv',UInt8,queue_size=10)

    node.timer = rospy.Timer(rospy.Duration(1/10), node.arr)

    
    # rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()




