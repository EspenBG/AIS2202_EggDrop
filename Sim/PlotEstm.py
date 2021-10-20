import numpy as np
from matplotlib import pyplot as plt

# Get variables from file
sensors = np.loadtxt("measures_raw.csv", dtype="float", delimiter=",")
estimate = np.loadtxt("estimates.csv", dtype="float", delimiter=",")

time = np.zeros_like(sensors[:, 0])
time_array = sensors[:, 0]

for i in np.arange(sensors[:, 0].size):
    time[i] = np.sum(time_array[0:i])


plt.figure()
#plt.plot(time, sensors[:, 4], label="TOF")
plt.plot(sensors[:, 3], label="IMU")
#plt.plot(time, estimate[:, 0], label="pos")
#plt.plot(time, estimate[:, 1], label="speed")
#plt.plot(time, estimate[:, 2], label="accel")
plt.ylim([0, 1.5])
plt.grid()
plt.legend()
plt.show()

plt.figure()
plt.plot(time, sensors[:, 4], label="TOF")
#plt.plot(time, sensors[:, 3], label="IMU")
#plt.plot(time, estimate[:, 0], label="pos")
#plt.plot(time, estimate[:, 1], label="speed")
#plt.plot(time, estimate[:, 2], label="accel")
plt.ylim([0, 130])
plt.grid()
plt.legend()
plt.show()

