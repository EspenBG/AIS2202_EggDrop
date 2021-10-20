import datetime
import time
from socket import *
import numpy as np

from SensorFusion import SensorFusion

udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)
udp_socket.settimeout(1)

np.set_printoptions(suppress=True)

arduino_ip = '192.168.10.240'
arduino_port = 8888

# En klasse som håndterer sensor fusion og bygger matrisene som brukes i Kalman filteret. KF er implementert som en egen klasse. Disse to må dere implementere selv

f = SensorFusion(0.212, .02, 2.765112489591674 / 1000, .160539894275893977)  # .160539894275893977 # 020539894275893977

reset = True
previous_time = datetime.datetime.now()

sensor_log_data = []
estimates_log_data = []


def estimate(measurements):
    global previous_time
    now = datetime.datetime.now()
    diff = now - previous_time
    previous_time = now

    delta_t = measurements[0]
    delta_t1 = measurements[0]*1
    delta_t2 = measurements[0]*1

    # print(delta_t)
    f.set_timestep(delta_t1)  # her oppdateres modellmatrisene med hensyn på tidssteget
    f.process_distance(measurements[4])  # her brukes avstandsmåling for å estimere tilstandene
    f.set_timestep(delta_t2)
    f.process_acceleration(measurements[3])  # her brukes akselerasjon i z-retning for å estimere tilstandene

    estimates = f.estimates()
    log_measurements_and_estimates(delta_t, estimates, measurements)

    return estimates


def log_measurements_and_estimates(delta_t, estimates, measurements):
    sensor_log_data.append([delta_t, measurements[1], measurements[2], measurements[3], measurements[4]])
    estimates_log_data.append([estimates.item(0, 0), estimates.item(1, 0), estimates.item(2, 0)])

    if len(sensor_log_data) > 1000:
        np.savetxt('measures.csv', sensor_log_data, delimiter=',')
        np.savetxt('estimates.csv', estimates_log_data, delimiter=',')
        sensor_log_data.clear()
        estimates_log_data.clear()


def arduino_has_been_reset():
    global reset
    if reset:
        print("Arduino is offline.. Resetting kalman filter")
        global f
        f = SensorFusion(0, .01, 2.765112489591674 / 1000, .00021203102776483 * 9.81)
        reset = False


mesurements = np.loadtxt("measures_raw.csv",  dtype="float", delimiter=",")


for mes in mesurements:
    #sensor_values = arduino_send_receive(f.estimates())
    sensor_values = mes
    if True:
        sensor_values[3] = (sensor_values[3]-1.0640459540459541)*9.81
        sensor_values[4] = sensor_values[4]/1000
    else:
        sensor_values[3] = sensor_values[3]
        sensor_values[4] = sensor_values[4]

    if sensor_values is not None:
        estimate(sensor_values)
    else:
        arduino_has_been_reset()
