#!/usr/bin/env python

# Library
import time
from datetime import datetime
import rospy
import csv
import numpy as np
import math
import os
from servo.msg import Joints_sub, Joints_publish
from std_msgs.msg import Float32MultiArray


def pub_(data):
    pub = rospy.Publisher('arduino_listener', Float32MultiArray, queue_size=10)   
    datas = Float32MultiArray(data=data)
    pub.publish(datas)


while_draw = False
draws = None
run = False
number = 0
para_begin = [0.0,0.0,0.0,0.0,0.0,0.0]
times = time.time()
traject = None

def callback_(data):
    global data_arduino, data_unity, simulator, run
    data_arduino = [data.data[0]/1848*360,data.data[1]/1848*360,data.data[2]]

def main():
    rospy.init_node('ATU', anonymous=True)
    rospy.Subscriber("arduino_chatter", Float32MultiArray, callback_)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass