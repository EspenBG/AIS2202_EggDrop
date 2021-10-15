import time
from socket import *
import numpy as np

print("foobar")

udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

np.set_printoptions(suppress = True)

arduino_ip = '192.168.10.240'
arduino_port = 8888
sensor_log_data = []


def arduino_send_receive(motor_rpm_percent):
    udp_socket.sendto(str(motor_rpm_percent).encode(), (arduino_ip, arduino_port))
    try:
        inbound_message, remote_address = udp_socket.recvfrom(24)
        # returns an array with the following values
        # [accel_x, accel_y, accel_z, range_sensor]
        t = np.array(inbound_message.decode('ascii').split(',')).astype(float)
        print(t)
        return t
    except Exception as e:
        print(e)


def use_sensor_values_for_something(sensor_values):
    #print(sensor_values)
    pass


def arduino_has_been_reset():
    print("Arduino is offline.. Maybe prepare for new initial state and re-initialize kalman filter?")


def log_measurements_and_estimates(delta_t, estimates, measurements):
    #sensor_log_data.append([delta_t, measurements[0], measurements[1], measurements[2], measurements[3]])
    #estimates_log_data.append([estimates.item(0, 0), estimates.item(1, 0), estimates.item(2, 0)])

    if len(sensor_log_data) > 500:
        np.savetxt('measures.csv', sensor_log_data, delimiter=',')
        #np.savetxt('estimates.csv', estimates_log_data, delimiter=',')
        sensor_log_data.clear()
        #estimates_log_data.clear()


power = 0.0
i =0
while(True):
    power = power + 1
    if(power > 100.0):
        power = 0.0

    sensor_values = arduino_send_receive(power)

    log_measurements_and_estimates(0, 0, sensor_values)
    if(sensor_values is not None):
        use_sensor_values_for_something(sensor_values)
        if power % 10 == 1: print("-------------------------")
    else:
        arduino_has_been_reset()

