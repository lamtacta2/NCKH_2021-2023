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

        # PID control signal
        pwm1 = min(max((P + I + D), -255), 255)
        pwm2 = min(max((P1 + I1 + D1), -255), 255)

        # Update the last error
        self.last_error[0] = error
        self.last_error[1] = error1

        # rospy.loginfo("pwm1: %s, pwm2: %s || dataa: %s, datab: %s" ,pwm1, pwm2, measured_value[0], measured_value[1])
        return [pwm1, pwm2]

def callback_(data):
    global pre_time, pwm
    if time.time() - pre_time < 5.0:
        control_pwm(pwm)
    elif time.time() - pre_time > 5.0 and time.time() - pre_time < 20.0:
        control_pwm([0.0,0.0])
    elif time.time() - pre_time > 10.0:
        pre_time = time.time()
        pwm = [0.0,pwm[1]+1.0]
    print(time.time()-pre_time)
    print(pwm)

def control_pwm(data):
    pub_(data)


def pub_(data):
    pub = rospy.Publisher('arduino_listener', Float32MultiArray, queue_size=10)   
    datas = Float32MultiArray(data=data)
    pub.publish(datas)


def pub_a(data):
    pub1 = rospy.Publisher('unity_listener', Joints_sub, queue_size=10) 
    a = Joints_sub([round(data[0],2),round(data[1],2)])
    pub1.publish(a)

def main():
    rospy.init_node('Tesst', anonymous=True)
    rospy.Subscriber("arduino_chatter", Float32MultiArray, callback_)
    rospy.spin()

pre_time = time.time()
pwm = [0.0,0.0]

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass