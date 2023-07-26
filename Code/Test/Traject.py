#!/usr/bin/env python

# Library
import time
from datetime import datetime
import csv
import numpy as np
import math
import os


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
    
# Trajector
class Trajector:
    def __init__(self) -> None:
        pass

    def draw_circle(self, center_x, center_y, radius, num_points):
        # Tính toán các góc tương ứng với các điểm trên hình tròn
        angles = np.linspace(0, 2*np.pi, num_points).tolist()
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
        """
        Tính toán động học nghịch cho robot 2 bậc tự do
        Input:
            x, y: Vị trí mong muốn của công cụ làm việc
            l1, l2: Độ dài các khớp của robot
        Output:
            q1, q2: Góc của các khớp để đạt được vị trí mong muốn
        """

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
            if abs(a[0]-x) >= 1 or abs(a[1]-y) >= 1:
                q1_1 = None
                q2_1 = None
            if abs(b[0]-x) >= 1 or abs(b[1]-y) >= 1:
                q1_2 = None
                q2_2 = None
            return [np.degrees(q1_1), np.degrees(q2_1), np.degrees(q1_2), np.degrees(q2_2)]
        else:
            return [None, None, None, None]

kinematic = Kinematic()
check = [0,0]
draw = Trajector()
while_draw = False
draws = None
i = 0
a = 1
draw_e = True
data_unity = None
data_arduino = None
stt = 0
para_begin = [0,0,0]
find = True
point_number = 0
traject = []
# # Create CSV
# csv_datas = Csv_data()
# current_date = datetime.now().strftime("%d-%m-%Y")  # Lấy ngày hiện tại và định dạng thành chuỗi "YYYY-MM-DD"
# file_name = f"/home/tta/data_find_pid_{current_date}.csv"  # Tạo tên file CSV từ ngày hiện tại
# header = ['Stt','Time','setpoint1','data1','setpoint2','data2','P','I','D','P1', 'I1', 'D1'] 

# if csv_datas.check_file_exists(file_name):
#     pass
# else:
#     csv_datas.create(file_name,header)



def SU(data):
    global point_number, draws, draw_e, para_begin, traject, data_unity, draw
    if draw_e == True:
        draws = draw.draw_circle(data[1],data[2],data[3],data[4])
        traject = draw_(draws)
        draw_e = False
        if traject[0][0] == 360:
            data[0] = 0
            draw_e = True

    if traject != None:
        for i in range(len(traject)):
            print(traject[i],i)

    data_unity[0] = 0


def distance_between_points(p, p1):
    return math.sqrt((p1[0] - p[0])**2 + (p1[1] - p[1])**2)


def main():
    global data_unity
    data_unity = [10, 140, 0, 80, 100]
    SU(data_unity)


def draw_(draws):
    global while_draw, data_unity, para_begin, draw_e   
    solution = []
    solution1 = [] 
    s = False
    s1 = False
    csv_datas = Csv_data()
    current_date = datetime.now().strftime("%d-%m-%Y")  # Lấy ngày hiện tại và định dạng thành chuỗi "YYYY-MM-DD"
    file_name = "a.csv"  # Tạo tên file CSV từ ngày hiện tại
    header = ['Stt','pos_x','pos_y','solution_x','solution_y','solution1_x','solution1_y'] 
    if csv_datas.check_file_exists(file_name):
        pass
    else:
        csv_datas.create(file_name,header)
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
        elif i >0 and a[0] != None and a[2] == None:
            if s == True:  
                solution.append([a[0],a[1]])
            if s1 == True:      
                solution1.append([a[0],a[1]]) 
        elif a[0] == None and a[2] == None:
            break
        elif i > 0 and a[0] != None and a[2] != None:
            if distance_between_points(solution[i-1],[a[0],a[1]]) <= distance_between_points(solution[i-1],[a[2],a[3]]) and s == True:
                solution.append([a[0],a[1]])
            elif distance_between_points(solution[i-1],[a[0],a[1]]) > distance_between_points(solution[i-1],[a[2],a[3]]) and s == True:
                solution.append([a[2],a[3]])
            if distance_between_points(solution1[i-1],[a[0],a[1]]) <= distance_between_points(solution1[i-1],[a[2],a[3]]) and s1 == True:
                solution1.append([a[0],a[1]])
            elif distance_between_points(solution1[i-1],[a[0],a[1]]) > distance_between_points(solution1[i-1],[a[2],a[3]]) and s1 == True:
                solution1.append([a[2],a[3]])
        if (s == False and s1 == False) or (len(solution) < i+1 and len(solution1) < i+1):
            break
        if s == False:
            solution.append(solution[0])
        if s1 == False:
            solution1.append(solution1[0])
        csv_datas.update(file_name,[i,draws[i][0],draws[i][1],solution[i][0],solution[i][1],solution1[i][0],solution1[i][1]])

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
        return [[360,360]]

while True:
    main()