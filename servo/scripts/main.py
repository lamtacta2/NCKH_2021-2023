#!/usr/bin/env python

import time
from datetime import datetime
import rospy
import csv
import numpy as np
import math
import os
import pandas as pd

from servo.msg import Joints_sub, Joints_publish, trajctory, Status
from std_msgs.msg import Float32MultiArray
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
        data = pd.read_csv(file_path)
        # Chuyển đổi dữ liệu thành kiểu float
        data['Time (s)'] = data['Time (s)'].astype(float)
        data['data'] = data['data'].astype(float)
        data['setpoint'] = data['setpoint'].astype(float)
        return data['data']
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
    
    def draw_circle_1(self, center_x, center_y, radius, num_points):
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
    
    def pid_si(self, setpoint, measured_value, delta):

        self.pwm1 = (setpoint[0] - measured_value[0] )/ 10 + measured_value[0]
        self.pwm2 = (setpoint[1] - measured_value[1] )/ 10 + measured_value[1]

        return [self.pwm1,self.pwm2]

def pub_(data):
    pub = rospy.Publisher('trajectory', trajctory, queue_size=10)   
    datas = trajctory(data=data)
    pub.publish(datas)

pid = PIDController()
Csv_data = Csv_data()
kinematic = Kinematic()
check = [0,0]
draw = Trajector()
while_draw = False
draws = None
run = False
number = 0
para_begin = [7.0,2.2,1.0,6.0,2.2,1.0]
times = time.time()
traject = None
timsample = 0.0
j1 = 0.0
j2 = 0.0
stt_unity = 0
file_name = "/home/tta/DT/src/servo/data_unity.csv"
header = ['STT,Timesample,J1,J2']
if Csv_data.check_file_exists(file_name):
    pass
else:
    Csv_data.create(file_name,header)

def callback(data):
    global data_unity, draw_e, simulator, run
    data_unity = data.Joints
    if round(data_unity[0]) == 10.0:
        draw_e = True
        simulator = True
        SU(data_unity) 
        
    if round(data_unity[0]) == 8.0:
        a = Csv_data.read("/home/tta/DT/src/servo/data.csv")
        b = Csv_data.read("/home/tta/DT/src/servo/data1.csv")
        Demo_simu(a,b)  
    

def pub_(data):
    pub = rospy.Publisher('arduino_listener', Float32MultiArray, queue_size=10)   
    datas = Float32MultiArray(data=data)
    pub.publish(datas)

def pub_a(data):
    pub1 = rospy.Publisher('unity_listener', Joints_sub, queue_size=10) 
    a = Joints_sub([round(data[0],2),round(data[1],2)])
    pub1.publish(a)


def callback_(data):
    global pid, data_unity
    if data_unity[0] == 50.0 and data_unity[1] == 1.0:
        pwm = pid.pid_si([data_unity[2],data_unity[3]],[data.Joints[1],data.Joints[2]],time.time() - times)
        pub_a(pwm)
        rospy.loginfo(pwm)
    rospy.sleep(0.01)

def Demo_simu(data, data1):
    pub1 = rospy.Publisher('unity_listener', Joints_sub, queue_size=10) 
    for i in range(len(data)):
        a = Joints_sub([round(data[i],2),round(data1[i],2)])
        print("data: %s, data1: %s", round(data[i],2),round(data1[i],2))
        pub1.publish(a)
        rospy.sleep(0.01)

def Unity_Status(data):
    global timsample, j1, j2, file_name, stt_unity
    stt_unity += 1
    timsample = data.timestamp/1000.0
    j1 = data.Joints1
    j2 = data.Joints2
    Csv_data.update(file_name,[stt_unity,timsample,j1,j2])

def SA(data):
    global  draws, draw_e, traject, data_unity
    if draw_e == True:
        draws = draw.draw_circle(data[1],data[2],data[3],data[4])
        traject = draw_(draws)
        draw_e = False
        if traject[0][0] == 360:
            data[0] = 0
            draw_e = True

    if traject != None:
        pub_(traject)
        print(traject)

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
    global while_draw, data_unity, draw_e   
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
            return solution
        else:
            return solution1
    elif s == True and s1 == False:
        solution.append(solution[0])
        return solution
    elif s == False and s1 == True:
        solution1.append(solution1[0])
        return solution1
    else:
        rospy.loginfo("EC: 001. No solution")
        return [[360,360]]

def distance_between_points(p, p1):
    return math.sqrt((p1[0] - p[0])**2 + (p1[1] - p[1])**2)

def main():
    rospy.init_node('SIMU', anonymous=True)
    rospy.Subscriber("unity_chatter", Joints_publish, callback)
    rospy.Subscriber("Status", Status, Unity_Status)
    # rospy.Subscriber("unity_chatter", Joints_publish, callback_)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass