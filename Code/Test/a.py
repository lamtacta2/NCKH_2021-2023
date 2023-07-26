feedback_values = [0,1,1,2,4,5,6]
time_axis = [0,2,3,4,5,6,7]
Kc = max(feedback_values) - min(feedback_values)
Tc = time_axis[feedback_values.index(max(feedback_values))] - time_axis[feedback_values.index(min(feedback_values))]
print(Kc)
print(Tc)