#!/usr/bin/env python

import datetime
import rospy
import sys
import copy
import math

from servo.msg import Joints_publish
from std_msgs.msg import Float32MultiArray

def callback(data):
    pub = rospy.Publisher('arduino_listener',Float32MultiArray,queue_size=10)
   # rate = rospy.Rate(10)   
    a = round(data.Joints[0])
    if a == 0:
        b = data.Joints[1]
        c = data.Joints[2]
        d = data.Joints[3]
    else:
        b = round(data.Joints[1]*3696/360) 
        c = round(data.Joints[2]*3696/360)
        d = data.Joints[3]
    arrray = [a,b,c,d]
    ang = Float32MultiArray(data=arrray)
    pub.publish(ang)


def listener():
    rospy.init_node('RTA', anonymous=True)
    rospy.Subscriber("unity_chatter", Joints_publish, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
