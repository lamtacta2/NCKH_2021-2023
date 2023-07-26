from Database import database
import csv
import time
import pandas as pd

data_database = database('tta','231000')

data_database.connect()

data = pd.read_csv('/home/tta/DT/src/servo/data_robot.csv')
data1 = pd.read_csv('/home/tta/DT/src/servo/data_unity.csv')

data['Time'] = data['Time'].astype(float)
data['Stt'] = data['Stt'].astype(float)
data['J1'] = data['J1'].astype(float)
data['J2'] = data['J2'].astype(float)
data1['Time'] = data1['Time'].astype(float)
data1['Stt'] = data1['Stt'].astype(float)
data1['J1'] = data1['J1'].astype(float)
data1['J2'] = data1['J2'].astype(float)


for i in range(len(data['Time'])):
    
    new_data = {
        'Time': 0.0,
        'J1': 0.0,
        'J2': 0.0
    }
    new_data['Time'] = data['Time'][i]
    new_data['J1'] = data['J1'][i]
    new_data['J2'] = data['J2'][i]
    data_pub1 = data_database.pub_data(new_data, 'DT','History_real')
    time.sleep(0.1)

data_pub2 = data_database.pub_data(new_data, 'DT','Current_real')
data_pub3 = data_database.pub_data(new_data, 'DT','Future_real')

for i in range(len(data1['Time'])):

    new_data = {
        'Time': 0.0,
        'J1': 0.0,
        'J2': 0.0
    }

    new_data['Time'] = data1['Time'][i]
    new_data['J1'] = data1['J1'][i]
    new_data['J2'] = data1['J2'][i]
    data_pub4 = data_database.pub_data(new_data, 'DT','History_virtual')
    time.sleep(0.1)


data_pub5 = data_database.pub_data(new_data, 'DT','Current_virtual')
data_pub6 = data_database.pub_data(new_data, 'DT','Future_virtual')


# get_data = data.get_data('DT','Current_real')

# for document in get_data:
#     print(document)
#     a = document
# print(document)
# print(get_data)