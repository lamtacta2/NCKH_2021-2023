#!/usr/bin/env python

# Library
import time
from datetime import datetime
import rospy
import csv
import numpy as np
import pandas as pd
import math
import os
import matplotlib.pyplot as plt

from servo.msg import Joints_publish, Main, Status, Virtual


class Kinematic:
    def forward_kinematics(self, theta1, theta2, link1_length, link2_length):
        # Chuyển đổi góc sang radian
        theta1 = round(theta1,5)
        theta2 = round(theta2,5)
        theta1 = math.radians(theta1)
        theta2 = math.radians(theta2)

        # Tính toán tọa độ (x, y) của công cụ
        x = link1_length * math.cos(theta1) + link2_length * math.cos(theta1 + theta2)
        y = link1_length * math.sin(theta1) + link2_length * math.sin(theta1 + theta2)

        return [x, y]

    # Hàm tính toán góc quay cho robot arm 2 bậc tự do
    def inverse_kinematics(self, x, y, l1, l2):
        x = round(x,5)
        y = round(y,5)
        r = np.sqrt(x**2 + y**2)
        cos_q2 = round((r**2 - l1**2 - l2**2) / (2 * l1 * l2),4)
        print(np.abs(cos_q2))
        if np.abs(cos_q2) <= 1:
            q2_1 = np.arccos(cos_q2)
            q2_2 = -np.arccos(cos_q2)
            q1_1 = np.arctan2(y, x) - np.arctan2((l2 * np.sin(q2_1)), (l1 + l2 * np.cos(q2_1)))
            q1_2 = np.arctan2(y, x) - np.arctan2((l2 * np.sin(q2_2)), (l1 + l2 * np.cos(q2_2)))
            a = self.forward_kinematics(np.degrees(q1_1),np.degrees(q2_1),l1,l2)
            b = self.forward_kinematics(np.degrees(q1_2),np.degrees(q2_2),l1,l2)
            if abs(a[0]-x) >= 1 or abs(a[1]-y) >= 1 or abs(q1_1) > 90 or abs(q1_1) < -90:
                q1_1 = None
                q2_1 = None
            if abs(b[0]-x) >= 1 or abs(b[1]-y) >= 1 or abs(q1_2) > 90 or abs(q1_2) < -90:
                q1_2 = None
                q2_2 = None
            return [np.degrees(q1_1), np.degrees(q2_1), np.degrees(q1_2), np.degrees(q2_2)]
        else:
            return [None, None, None, None]

def pub(data):
    pub = rospy.Publisher('main_', Main, queue_size=10)   
    datas = Main(data=data)
    pub.publish(datas)
    # rospy.loginfo(datas)

def callback(data):
    global data_
    # rospy.loginfo(data)
    if data.Joints[0] == 0.0:
        if data.Joints[1] == 0.0:
            pub([0.0,data.Joints[2],data.Joints[3],0.0,0.0])
        elif data.Joints[1] == 1.0:
            A = kinematic.inverse_kinematics(data.Joints[2],data.Joints[3],140,80)
            if A[0] == None and A[2] == None:
                pass
            elif A[0] == None:
                pub([0.0,A[2],A[3],0.0,0.0])
            else:
                pub([0.0,A[0],A[1],0.0,0.0])               
    elif data.Joints[1] == 2.0:
        pub([2.0,data.Joints[1],data.Joints[2],data.Joints[3],data.Joints[4]])
    else:
        if data.Joints[1] == 0.0:
            pub([1.0,data.Joints[2],data.Joints[3],data_[0],data_[1]])
        elif data.Joints[2] == 1.0:
            A = kinematic.inverse_kinematics(data.Joints[2],data.Joints[3],140,80)
            if A[0] == None and A[2] == None:
                pass
            elif A[0] == None:
                pub([1.0,A[2],A[3],data_[0],data_[1]])
            else:
                pub([1.0,A[0],A[1],data_[0],data_[1]])  

kinematic = Kinematic()
data_ = [0.0, 0.0]

def main():
    rospy.init_node('CENTER', anonymous=True)
    rospy.Subscriber("unity_chatter", Joints_publish, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
