import datetime
import time
from socket import *
import numpy as np

from SensorFusion import SensorFusion

udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)
udp_socket.settimeout(1)

only_measure = True

np.set_printoptions(suppress = True)

arduino_ip = '192.168.10.240'
arduino_port = 8888

# En klasse som håndterer sensor fusion og bygger matrisene som brukes i Kalman filteret. KF er implementert som en egen klasse. Disse to må dere implementere selv

f = SensorFusion(0.1, .01, 2.765112489591674/1000, .020539894275893977)

reset = True
previous_time = datetime.datetime.now()

sensor_log_data = []
estimates_log_data = []

live_plot = True


def arduino_send_receive(estimate):
    global reset
    global previous_time
    if not reset:
        previous_time = datetime.datetime.now()

    distance = estimate.item(0)
    udp_socket.sendto(str(distance).encode(), (arduino_ip, arduino_port))
    try:
        inbound_message, remote_address = udp_socket.recvfrom(24)
        # returns an array with the following values
        # [accel_x, accel_y, accel_z, range_sensor]
        parse = np.array(inbound_message.decode('ascii').split(',')).astype(float)
        reset = True
        if only_measure:
            ret = [parse.item(0), parse.item(1), (parse.item(2)), parse.item(3)]  # husk at verdiene har ulike enheter (m/s^2 og mm)
        else:
            ret = [parse.item(0), parse.item(1), (parse.item(2)-1.0640459540459541)*9.81, parse.item(3)/1000]  # husk at verdiene har ulike enheter (m/s^2 og mm)

        return ret
    except Exception as e:
        print(e)


def plot_measurements_and_estimates(delta_t, estimates, measurements):
    pass


def estimate(measurements):
    global previous_time
    now = datetime.datetime.now()
    diff = now - previous_time
    previous_time = now

    delta_t = diff.total_seconds()
    #print(delta_t)
    f.set_timestep(delta_t) # her oppdateres modellmatrisene med hensyn på tidssteget
    f.process_distance(measurements[3]) # her brukes avstandsmåling for å estimere tilstandene
    f.process_acceleration(measurements[2]) # her brukes akselerasjon i z-retning for å estimere tilstandene
    
    estimates = f.estimates()
    log_measurements_and_estimates(delta_t, estimates, measurements)
    plot_measurements_and_estimates(delta_t, estimates, measurements)
    
    return estimates


def log_measurements_and_estimates(delta_t, estimates, measurements):
    sensor_log_data.append([delta_t, measurements[0], measurements[1], measurements[2], measurements[3]])
    estimates_log_data.append([estimates.item(0), estimates.item(1), estimates.item(2)])

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
        f = SensorFusion(0, .01, 2.765112489591674/1000, .00021203102776483*9.81)
        reset = False


while True:
    sensor_values = arduino_send_receive(f.estimates())
    if sensor_values is not None:
        estimate(sensor_values)
    else:
        arduino_has_been_reset()