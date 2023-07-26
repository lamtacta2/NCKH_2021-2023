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

from servo.msg import Joints_sub, Joints_publish, trajctory, Trajectory
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

# File data csv
class Csv_data:
    def __init__(self) -> None:
        pass

    def check_file_exists(self,file_path):
        return os.path.exists(file_path)
    
    def create(self, file_path, headers):
        with open(file_path, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(headers)

    def update(self, file_path, data):
        with open(file_path, 'a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(data)

    def read(self,file_path):
        with open(file_path, 'r') as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
                print(row)
        return reader

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

        rospy.loginfo(setpoint)

        return [self.pwm1,self.pwm2]
        
def callback(data):
    global data_unity, run, trajct, para_begin, pwm_, mode, kinematic
    if data.Joints == 100.0:
        # Real
        data_unity = [data.Joints[1],data.Joints[2],data.Joints[3],data.Joints[4]] 
    if data.Joints[0] == 4.0:
        para_begin = [data.Joints[1], data.Joints[2], data.Joints[3], para_begin[3], para_begin[4], para_begin[5]]
    elif data.Joints[0] == 6.0:
        para_begin = [para_begin[0], para_begin[1], para_begin[2], data.Joints[1], data.Joints[2], data.Joints[3]]    
    elif data.Joints[0] == 11.0:
        mode = True
        pwm_ = [data.Joints[1],data.Joints[2]]
    elif data.Joints[0] == 12.0:
        mode = False
    else:   
        data_unity = data.Joints
    if data_unity[0] == 2.0:
        draws = draw.draw_circle(data_unity[1],data_unity[2],data_unity[3],data_unity[4])
        trajct = draw_(draws)
        for i in range(trajct):
            if trajct[i][1] < 0:
                trajct[i][1] = 360 + trajct
        rospy.loginfo(trajct) 
    if data_unity[0] == 20.0:
        J = kinematic.inverse_kinematics(data_unity[1],data_unity[2],140,80)
        data_unity = [1.0,J[0],J[1],0.0,0.0,0.0,0.0]
        rospy.loginfo(data_unity)
    if data_unity[0] == 3.0:
        pub_([0.0,0,0])
    # rospy.loginfo(data.Joints)

def callback_(data):
    global data_arduino, data_unity, mode
    data_arduino = [data.data[0]/1848*360,data.data[1]/1848*360,data.data[2]]
    if data_unity[0] != 50.0:
        if mode == False:
            processer(data_arduino)
        else:
            control_pwm(data_arduino)
    

def control_pwm(data):
    global pwm_, data_unity, pre_time
    pre_time = time.time()
    pub_a(data)
    if round(data_unity[0]) == 1.0:
        if abs(data[0] - data_unity[1]) > 1.0:
            if data[0] - data_unity[1] < 0.0:
                pub_([pwm_[0],0.0]) 
            elif data[0] - data_unity[1] < 0.0:
                pub_([-pwm_[0],0.0])
        else:
            pub_([0.0,0.0])
        if abs(data[1] - data_unity[2]) > 1.0:
            if data[1] - data_unity[2] < 0.0:
                pub_([0.0,pwm_[1]])  
            elif data[1] - data_unity[2] > 0.0:
                ss = pwm_[1]*-1.0
                pub_([0.0,ss])  
        else:
            pub_([0.0,0.0])  
    rospy.loginfo(data)
    rospy.loginfo(time.time()-pre_time)

def pub_(data):
    pub = rospy.Publisher('arduino_listener', Float32MultiArray, queue_size=10)   
    datas = Float32MultiArray(data=data)
    pub.publish(datas)

def pub_a(data):
    pub1 = rospy.Publisher('unity_listener', Joints_sub, queue_size=10) 
    a = Joints_sub([round(data[0],2),round(data[1],2)])
    pub1.publish(a)

def processer(data):
    # rospy.loginfo(data[2])
    global pre, k1, k, id, run, point_number, para_begin, times, counter, begin_time, number, pre_data, base,  check0, check1, data_unity, pub_time
    pub1 = rospy.Publisher('unity_listener', Joints_sub, queue_size=10) 
    if round(data_unity[0]) != 10.0 and round(data_unity[0]) != 9.0 and round(data_unity[0]) != 8.0 and time.time() - pub_time >= 0.05: 
        # if data_unity[1] - data_arduino[0] >= 10:
        #     k += 1
        # else: 
        #     k = 0
        # if k >= 100:
        #     pub1.publish([0.0,0.0])
        #     rospy.loginfo("EC: 003. Motor 1 Fail")
        #     rospy.on_shutdown()
        # if data_unity[2] - data_arduino[1] >= 10:
        #     k1 += 1
        # else:
        #     k1 = 0
        # if k1 >= 100:
        #     pub1.publish([0.0,0.0])
        #     rospy.loginfo("EC: 004. Motor 2 Fail")
        #     rospy.on_shutdown()
        # if k1 >= 100 and k1 >= 100:
        #     pub1.publish([0.0,0.0])
        #     rospy.loginfo("EC: 005. Motor Fail")
        #     rospy.on_shutdown()
        pub_time = time.time()
        a = Joints_sub([round(data_arduino[0],2),round(data_arduino[1],2)])
        # rospy.loginfo(a)
        pub1.publish(a)

    if round(data_unity[0]) == 9.0:
        pub1.publish(a)
    if round(data_unity[0]) != 2:
        id = 0
    if round(data_unity[0]) == 2.0:
        if run == True:
            if distance_between_points([trajct[id][0],trajct[id][1]],[data[0],data[1]]) <= 2:
                id += 1
            if id < data_unity[4]:
                if time.time() - times >= 0.02:
                    counter += 1
                    point_number += 1
                    if id >= 2 and trajct[id-1][1] > 0.0 and trajct[id-2][1] > 0.0 and trajct[id][1] < 0.0:
                        trajct[id][1] = trajct[id][1] + 360
                    if id >= 2 and trajct[id-1][1] > 240 and trajct[id][1] < 10:
                        trajct[id][1] = 360 + trajct[id][1]
                    pwm = pid.pid(para_begin,[trajct[id][0],trajct[id][1]],[data[0],data[1]], time.time() - times)
                    pub_(pwm)
                    rospy.loginfo(pwm)
                    times = time.time()
            else:
                if distance_between_points([trajct[id-1][0],trajct[id-1][1]],[data[0],data[1]]) <= 2:
                    run = False
                    id = 0
                    pwm = [0,0]
                else:
                    pwm = pid.pid(para_begin,[trajct[id-1][0],trajct[id-1][1]],[data[0],data[1]], time.time() - times)
                    pub_(pwm)
        else:
            if distance_between_points([trajct[int(data_unity[4])-1][0],trajct[int(data_unity[4])-1][1]],[data[0],data[1]]) <= 2:
                    pwm = [0,0]
            else:
                pid.pid(para_begin,[trajct[int(data_unity[4])-1][0],trajct[int(data_unity[4])-1][1]],[data[0],data[1]], time.time() - times)
    if round(data_unity[0]) == 1.0:
        if time.time() - times >= 0.0143:
            pwm = pid.pid(para_begin,[data_unity[1],data_unity[2]],[data[0],data[1]], time.time() - times)
            pub_(pwm)
            rospy.loginfo(pwm)
            times = time.time()


def SU(data):
    global  draws, draw_e, traject, data_unity
    pub1 = rospy.Publisher('unity_listener', Joints_sub, queue_size=10) 
    rospy.loginfo("Joints: %s", draw_e)
    if draw_e == True:
        draws = draw.draw_circle(data[1],data[2],data[3],data[4])
        traject = draw_(draws)
        draw_e = False
        if traject[0][0] == 360:
            data[0] = 0
            draw_e = True

    if traject != None:
        for i in range(len(traject)):
            pub1.publish(traject[i])
            rospy.loginfo("Joints: %s", traject[i])
            rospy.sleep(0.1)

def draw_(draws):
    global while_draw, data_unity, draw_e, run
    solution = []
    solution1 = [] 
    s = False
    s1 = False
    for i in range(len(draws)):
        a = kinematic.inverse_kinematics(draws[i][0],draws[i][1],140,80)
        if i == 0 and a[0] != None:
            solution = [[a[0],a[1]]]
            s = True
        if i == 0 and a[2] != None:
            solution1= [[a[2],a[3]]]
            s1 = True
        if i > 0 and a[0] == None and a[2] != None:
            if s == True:
                solution.append([a[2],a[3]])
            if s1 == True:    
                solution1.append([a[2],a[3]]) 
        elif i > 0 and a[0] != None and a[2] == None:
            if s == True:  
                solution.append([a[0],a[1]])
            if s1 == True:      
                solution1.append([a[0],a[1]]) 
        elif a[0] == None and a[2] == None:
            rospy.loginfo("EC: 001. No solution")
            break
        elif i > 0 and a[0] != None and a[2] != None:
            if abs(solution[i-1][0] - a[0]) <= abs(solution[i-1][0]-a[2]) and s == True:
                solution.append([a[0],a[1]])
            elif abs(solution[i-1][0] - a[0]) > abs(solution[i-1][0]-a[2]) and s == True:
                solution.append([a[2],a[3]])
            if abs(solution1[i-1][0] - a[0]) <= abs(solution1[i-1][0]-a[2])and s1 == True:
                solution1.append([a[0],a[1]])
            elif abs(solution1[i-1][0] - a[0]) > abs(solution1[i-1][0]-a[2])and s1 == True:
                solution1.append([a[2],a[3]])
        if (s == False and s1 == False) or (len(solution) < i+1 and len(solution1) < i+1):
            rospy.loginfo("EC: 001. No solution")
            break
        if s == False:
            solution.append(solution[0])
        if s1 == False:
            solution1.append(solution1[0])
     
    if s == True and s1 == True:
        a = 0
        b = 0
        solution.append(solution[0])
        solution1.append(solution1[0])
        for i in range(len(solution)-1):
            a += distance_between_points(solution[i],solution[i+1])
            b += distance_between_points(solution1[i],solution1[i+1])
        if a <= b:
            run = True
            return solution
        else:
            run = True
            return solution1
    elif s == True and s1 == False:
        solution.append(solution[0])
        run = True
        return solution
    elif s == False and s1 == True:
        solution1.append(solution1[0])
        run = True
        return solution1
    else:
        rospy.loginfo("EC: 001. No solution")
        return [[360,360]]

# Kinematics
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

def distance_between_points(p, p1):
    return math.sqrt((p1[0] - p[0])**2 + (p1[1] - p[1])**2)

def emti(p,pre_p,time,pre_time,pre_v):
    v = (p-pre_p)/(time-pre_time)*1000
    a = (v-pre_v)/(time-pre_time)*1000
    s = p + v*(time-pre_time) + 0.5*a*((time-pre_time)**2)
    rospy.loginfo(p-pre_p)
    # rospy.loginfo(v)
    # rospy.loginfo(pre_v)
    # rospy.loginfo(s)
    return s, p, v, time

def main():
    rospy.init_node('REAL', anonymous=True)
    rospy.Subscriber("unity_chatter", Joints_publish, callback)
    rospy.Subscriber("arduino_chatter", Float32MultiArray, callback_)
    rospy.spin()

pre_v2 = 0.0
pre_v1 = 0.0
pre_time2 = 0.0
pre_time1 = 0.0
pre_p1 = 0.0
pre_p2 = 0.0
pub_time = time.time()
pre_time = 0
stt_pwm = 0
pre = []
k = 0
k1 = 0
kinematic = Kinematic()
check = [0,0]
draw = Trajector()
while_draw = False
draws = None
run = False
number = 0
times = time.time()
traject = [0,0]
id = 0
run = False
trajct = [0,0]
check0 = 0
check1 = 0
pre_data = [0,0]
counter = 0
number = 0
data_unity = [0.0,0.0,0.0]
data_arduino = None
stt = 0
para_begin = [5.0*3.5,2.0*1.5,1.0*0.8,5.0*4.1,2.0*1.77,1.0*0.8]
pid = PIDController()
point_number = 0
traject = [0,0]
simulator = False
times = time.time()
begin_time = time.time()
base = [0,0]
pwm_ = [0.0,0.0]
mode = False
csv_datas = Csv_data()
header = ['Stt','Time (s)','setpoint','data','pwm'] 
header1 = ['Stt','Time (s)','setpoint','data','pwm','setpoint1','data1','pwm1'] 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
