import numpy as np


# Get variables from file
values = np.loadtxt("sim/measures_raw.csv", dtype="float", delimiter=",")

distance = values[:, 4]
accelerometer = values[:, 3]

variance_distance = 1/distance.shape[0]*np.sum((np.average(distance)-distance)**2)
variance_accelerometer = 1/distance.shape[0]*np.sum((np.average(accelerometer)-accelerometer)**2)

print(variance_distance)
print(variance_accelerometer)
print(np.average(accelerometer))
