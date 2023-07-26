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

from servo.msg import Virtual
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

def callback_(data):
    global stt,file_name1
    stt +=1
    a = data.data[0]/1848*360
    b = data.data[1]/1848*360
    csv_datas.update(file_name1,[stt,time.time(),a,b])

def callback_c(data):
    global stt1, file_name
    stt1 += 1
    csv_datas.update(file_name,[stt1,time.time(),data.joints[0],data.joints[1]])
    rospy.loginfo(data)
    
def main():
    rospy.init_node('SAVE', anonymous=True)
    rospy.Subscriber("arduino_chatter", Float32MultiArray, callback_)
    rospy.Subscriber("data_virtual", Virtual, callback_c)
    rospy.spin()

times = time.time()
stt = 0
stt1 = 0
csv_datas = Csv_data()
header = ['Stt','Time','J1','J2'] 
file_name = "/home/tta/DT/src/servo/data_unity.csv"
file_name1 = "/home/tta/DT/src/servo/data_robot.csv"

if csv_datas.check_file_exists(file_name1):
    pass
else:
    csv_datas.create(file_name1,header)

if csv_datas.check_file_exists(file_name):
    pass
else:
    csv_datas.create(file_name,header)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass