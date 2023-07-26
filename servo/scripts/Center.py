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

from servo.msg import Joints_sub, Main, Virtual, Status
from std_msgs.msg import Float32MultiArray

# Trajector
class Trajector:
    def __init__(self) -> None:
        pass

    def draw_circle(self, center_x, center_y, radius, num_points):
        # Tính toán các góc tương ứng với các điểm trên hình tròn
        angles = np.linspace(0, 2*np.pi, int(num_points)).tolist()
        # Tạo các điểm trên hình tròn
        points = []
        for i in range(int(num_points)):
            # Tính toán các tọa độ (x, y) của các điểm trên hình tròn
            x = center_x + radius * math.cos(angles[i])
            y = center_y + radius * math.sin(angles[i])                          
            points.append([x,y])
        return points
    
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

class PIDController:
    def __init__(self):
        self.last_error = [0,0]
        self.integral = [0,0]
        self.dt = 0
        self.done1 = False
        self.done2 = False
        self.time1 = time.time()
        self.time2 = time.time()
        self.pub_time = time.time()
        self.pwm1 = 0.0
        self.pwm2 = 0.0

    def pid(self, para, setpoint, measured_value, delta):
        self.dt = delta
        error = setpoint[0] - measured_value[0]
        error1 = setpoint[1] - measured_value[1]

        # Proportional term
        P = para[0] * error
        P1 = para[3] * error1

        # Integral term
        self.integral[0] += error * self.dt
        self.integral[1] += error1 * self.dt
        I = para[1] * self.integral[0]
        I1 = para[4] * self.integral[1]

        # Derivative term
        D = para[2] * (error - self.last_error[0])/self.dt
        D1 = para[5] * (error1 - self.last_error[1])/self.dt
 
        self.pwm1 = min(max((P + I + D), -255), 255)
        self.pwm2 = min(max((P1 + I1 + D1), -255), 255)

       # Update the last error
        self.last_error[0] = error
        self.last_error[1] = error1

        return [self.pwm1,self.pwm2]
    
def callback(data):
    global controller, setpoint_unity
    controller = data.data
    setpoint_unity = [data.data[1],data.data[2]]

def callback_(data):
    global controller, data_, setpoint_unity
    data_ = [data.data[0]/1848*360,data.data[1]/1848*360,data.data[2]]
    if controller[0] == 0.0:
        setpoint = [controller[1],controller[2]] 
        real(setpoint,data_)
    elif controller[0] == 1.0:
        setpoints =  pid_(setpoint_unity)
        real(setpoints,data_)    
        rospy.loginfo(setpoints)

    
def real(setpoint,data):
    global para_pid, controller, times, time_unity, collider
    if time.time() - times >= 0.0143:
        if collider == True:
            pub_controller([0,0])
        else:
            pwm = pid.pid(para_pid,setpoint,data,time.time() - times)
            pub_controller(pwm)
        times = time.time()
        # rospy.loginfo(pwm)
    if time.time() - time_unity >= 0.05:
        if collider == True:
            a = [data[0],data[1],500.0,500.0]
        else:  
            a = [data[0],data[1],setpoint[0],setpoint[1]]
        pub_unity(a)
        time_unity = time.time()
        rospy.loginfo(a)
    rospy.loginfo(collider)
        
def callback__(data):
    global mesua
    mesua = data.joints

def pid_(setpoint):
    global mesua, collider
    if collider == True:
        a = mesua[0]
        b = mesua[1]
    else:
        a = mesua[0] + (setpoint[0] - mesua[0])*0.1 
        b = mesua[1] + (setpoint[1] - mesua[1])*0.1
    return [a,b]

def pub_controller(data):
    pub = rospy.Publisher('arduino_listener', Float32MultiArray, queue_size=10)   
    datas = Float32MultiArray(data=data)
    pub.publish(datas)

def pub_unity(data):
    pub1 = rospy.Publisher('unity_listener', Joints_sub, queue_size=10) 
    a = Joints_sub(data)
    pub1.publish(a)

def callback_c(data):
    global collider
    if data.Joints1 == 1.0:
        collider = True
    else:
        collider = False  
    rospy.loginfo(collider)

kinematic = Kinematic()
pid = PIDController()
draw = Trajector()
para_pid = [5.0*4.1,2.0*1.5,1.0*0.8,5.0*4.1,2.0*1.77,1.0*0.8]
controller = [10.0,10.0,0.0,0.0,0.0]
data_ = [0.0,0.0]
times = time.time()
time_unity = time.time()
collider = False
setpoint_unity = []
mesua = [0.0,0.0]

def main():
    rospy.init_node('REAL', anonymous=True)
    rospy.Subscriber("main_", Main, callback)
    rospy.Subscriber("arduino_chatter", Float32MultiArray, callback_)
    rospy.Subscriber("Status", Status, callback_c)
    rospy.Subscriber("data_virtual", Virtual, callback__)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
