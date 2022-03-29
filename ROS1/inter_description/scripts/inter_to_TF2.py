#!/usr/bin/env python3
import rospy
import tf2_ros as tf
import tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped

from std_msgs.msg import Header
from sensor_msgs.msg import Imu
import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Bool,UInt16,Float32

class Node :
    def __init__(self) :
        self.yaw=0
        self.pitch=0
        self.roll=0
        rospy.init_node('tf_broadcaster')
        
        self.timer = rospy.Timer(rospy.Duration(1.0 / 10), self.tfBroadcasting)


        rospy.on_shutdown(self.callback_shutDownTimer)  


    def callback_current_roll(self,msg):
        self.roll = (msg.data*-1)-3.14

    def callback_current_pitch(self,msg):
        self.pitch = msg.data
    def callback_current_yaw(self,msg):
        self.yaw = msg.data

    def tfBroadcasting(self,event):

        rospy.Subscriber('/roll',Float32,self.callback_current_roll)
        rospy.Subscriber('/pitch',Float32,self.callback_current_pitch)
        rospy.Subscriber('/yaw',Float32,self.callback_current_yaw)
        br = tf.TransformBroadcaster()
        t = TransformStamped()
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "world"
        t.header = h
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = 4
        t.transform.translation.y = 4
        t.transform.translation.z = 0
        rotation = tf_conversions.transformations.quaternion_from_euler(self.roll,self.pitch,0)
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]


        
        br.sendTransform(t)

    def callback_shutDownTimer(self):
        self.timer.shutdown()
if __name__=='__main__':
    node = Node()
    rospy.spin()