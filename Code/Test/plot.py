import pandas as pd
import matplotlib.pyplot as plt

while True:
    data = pd.read_csv('a.csv')

    # Chuyển đổi dữ liệu thành kiểu float
    data['solution_x'] = data['solution_x'].astype(float)
    data['solution_y'] = data['solution_y'].astype(float)
    data['solution1_x'] = data['solution1_x'].astype(float)
    data['solution1_y'] = data['solution1_y'].astype(float)
    data['Stt'] = data['Stt'].astype(float)
    # Vẽ biểu đồ
    plt.plot(data['Stt'], data['solution_x'], label='x1')
    plt.plot(data['Stt'], data['solution_y'], label='y1')
    # plt.plot(data['Stt'], data['solution1_x'], label='x2')
    # plt.plot(data['Stt'], data['solution1_y'], label='y2')
    plt.xlabel('X Label')
    plt.ylabel('Y Label')
    plt.show()
