#!/usr/bin/env python3
from curses import use_env
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist,Vector3Stamped
from std_msgs.msg import Bool,UInt16,Float32

from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import Imu
class Node :
    def __init__(self):
   

        self.timer = None
        self.yaw=0
        self.pitch=0
        self.roll=0

       
    # get currunt pose
    def callbacky(self,msg):
        self.yaw=msg.vector.z
        self.pitch=msg.vector.y
        self.roll=msg.vector.x
           
    def target(self,event):
       
        rospy.Subscriber('/imu/rpy/filtered',Vector3Stamped,self.callbacky)


        pub_cmd_yaw.publish(self.yaw)
        pub_cmd_pitch.publish(self.pitch)
        pub_cmd_roll.publish(self.roll)

    def callback_shutDownTimer(self):
        self.timer.shutdown()

    

        

    
if __name__=='__main__':
    rospy.init_node('yyimu')
    node = Node()
    
    
    pub_cmd_yaw = rospy.Publisher('/yaw',Float32,queue_size=10)
    pub_cmd_pitch = rospy.Publisher('/pitch',Float32,queue_size=10)
    pub_cmd_roll = rospy.Publisher('/roll',Float32,queue_size=10)

    node.timer = rospy.Timer(rospy.Duration(1/10), node.target)

    
    # rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()





