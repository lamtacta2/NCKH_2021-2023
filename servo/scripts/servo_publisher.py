#!/usr/bin/env python

import time
import rospy
import csv
from servo.msg import Joints_sub, Joints_publish
from std_msgs.msg import Float32MultiArray

def callback(data):
    pub = rospy.Publisher('unity_listener', Joints_sub,queue_size=10)
    
    dataa = [data.data[1]/3696*360, data.data[0]/3696*360, time.time()]
    # dataa = [a, data.data[0]/3696*360, haha]
    pub.publish(Joints_sub(dataa))


def listener():
    rospy.init_node('ATU', anonymous=True)
    rospy.Subscriber("arduino_chatter", Float32MultiArray, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
