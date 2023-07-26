import math
import numpy as np

def forward_kinematics(theta1, theta2, link1_length, link2_length):
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
def inverse_kinematics(x, y, l1, l2):
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
        a = forward_kinematics(np.degrees(q1_1),np.degrees(q2_1),l1,l2)
        b = forward_kinematics(np.degrees(q1_2),np.degrees(q2_2),l1,l2)
        if abs(a[0]-x) >= 1 or abs(a[1]-y) >= 1:
            q1_1 = None
            q2_1 = None
        if abs(b[0]-x) >= 1 or abs(b[1]-y) >= 1:
            q1_2 = None
            q2_2 = None
        return [np.degrees(q1_1), np.degrees(q1_2)], [np.degrees(q2_1), np.degrees(q2_2)]
    else:
        return None, None

# while True:

#     a = [-150,80]
#     a[0] = float(input("Nhập gốc 1: "))
#     a[1] = float(input("Nhập gốc 2: "))
#     if a[0] == 5 and a[1] == 5:
#         break
#     q1_values, q2_values = inverse_kinematics(a[0], a[1], 140, 80)
#     if q1_values is not None:
#         for i in range(len(q1_values)):
#             q1 = q1_values[i]
#             q2 = q2_values[i]
#             print("Góc q1:", q1)
#             print("Góc q2:", q2)
#             print("---------------------")
#     else:
#         print("Không có giải pháp với vị trí mong muốn.")


while True:

    a = [-150,80]
    a[0] = float(input("Nhập gốc 1: "))
    a[1] = float(input("Nhập gốc 2: "))
    if a[0] == 5 and a[1] == 5:
        break
    b = forward_kinematics(a[0],a[1],140,80)
    q1_values, q2_values = inverse_kinematics(b[0], b[1], 140, 80)
    print("IK | Vị trí (x, y):", b[0], b[1])
    if q1_values is not None:
        for i in range(len(q1_values)):
            q1 = q1_values[i]
            q2 = q2_values[i]
            print("Góc q1:", q1)
            print("Góc q2:", q2)
            print("---------------------")
    else:
        print("Không có giải pháp với vị trí mong muốn.")
