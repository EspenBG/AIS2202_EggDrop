import time
from socket import *
import numpy as np

power = 0.0             # Sets the initial value for power

# Initializing UDP socket and settings
udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

# Determines how numpy data are displayed
np.set_printoptions(suppress = True)

# Arduino communication settings
arduino_ip = '192.168.10.240'
arduino_port = 8888


reset = True           # NOT IN USE
previous_time = datetime.datetime.now()     # Variable that holds time when program is here

sensor_log_data = []                        # Array that holds the sensor readings
estimates_log_data = []                     # Array that holds the estimates


def arduino_send_receive(estimate):
    global reset
    global previous_time
    if not reset:
        previous_time = datetime.datetime.now()

    distance = estimate.item((0, 0))
    udp_socket.sendto(str(distance).encode(), (arduino_ip, arduino_port))
    try:
        inbound_message, remote_address = udp_socket.recvfrom(24)
        # returns an array with the following values
        # [accel_x, accel_y, accel_z, range_sensor]
        parse = np.array(inbound_message.decode('ascii').split(',')).astype(float)
        reset = True
        ret = [parse.item(0), parse.item(1), parse.item(2), parse.item(3)] # OBS: husk at verdiene har ulike enheter (m/s^2 og mm)
        return ret
    except Exception as e:
        print(e)

def use_sensor_values_for_something(sensor_values):
    print(sensor_values)
    pass


def arduino_has_been_reset():
    print("Arduino is offline.. Maybe prepare for new initial state and re-initialize kalman filter?")


#
while True:
    power = power + 1
    if power > 100.0:
        power = 0.0

    # Calls function to send new value and handle incoming values
    sensor_values = arduino_send_receive(power)

    # If sensor data is read,
    if sensor_values is not None:
        # Uses data to calculate covariance for
        use_sensor_values_for_something(sensor_values)
        if power % 10 == 1: print("-------------------------")
    else:
        arduino_has_been_reset()