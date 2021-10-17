import numpy as np
from matplotlib import pyplot as plt

# Get variables from file
sensors = np.loadtxt("measures.csv", dtype="float", delimiter=",")
estimate = np.loadtxt("estimates.csv", dtype="float", delimiter=",")


plt.figure()
plt.plot(sensors[:, 4], label="TOF")
#plt.plot(sensors[:, 3], label="IMU")
plt.plot(estimate[:, 0], label="pos")
plt.plot(estimate[:, 1], label="speed")
plt.plot(estimate[:, 2], label="accel")
plt.grid()
plt.legend()
plt.show()

