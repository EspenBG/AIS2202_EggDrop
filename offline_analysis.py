import datetime
import time
from socket import *
import numpy as np
import statistics as stat

power = 0.0  # Sets the initial value for power

# Initializing UDP socket and settings
udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

# Determines how numpy data are displayed
np.set_printoptions(suppress=True)

# Arduino communication settings
arduino_ip = '192.168.10.240'
arduino_port = 8888

reset = True  # NOT IN USE
previous_time = datetime.datetime.now()  # Variable that holds time when program is here

sensor_log_data = []  # Array that holds the sensor readings


def arduino_receive():
    global reset
    global previous_time
    if not reset:
        previous_time = datetime.datetime.now()


    # udp_socket.sendto(str(distance).encode(), (arduino_ip, arduino_port))

    try:
        inbound_message, remote_address = udp_socket.recvfrom(24)
        # returns an array with the following values
        # [accel_x, accel_y, accel_z, range_sensor]
        parse = np.array(inbound_message.decode('ascii').split(',')).astype(float)
        reset = True
        # Only need item 2 and 3 (respectively acceleration in z direction and range sensor data)
        ret = [parse.item(0), parse.item(1), parse.item(2), parse.item(3)]  # OBS: husk at verdiene har ulike enheter (m/s^2 og mm)
        return ret
    except Exception as e:
        print(e)

def log_measurements (measurements):
    # Only need item 2 and 3 (respectively acceleration in z direction and range sensor data)
    sensor_log_data.append([measurements[2], measurements[3]])

    # If the measurement log exceeds 500 lines
    if len(sensor_log_data) > 500:
        np.savetxt('measures.csv', sensor_log_data, delimiter=',')      # Saves the measurement data to csv file
        sensor_log_data.clear()                                         # Removes all data from dictionary
        return True


# TODO - Use either library with built-in functions or create variance function on own
def calculate_variance(sensor_values):


        # TODO - Pass incoming data as array into argument of variance
        # Need to figure out how to structure the csv data into an array for each sensor
        print("Variance for accelerometer: " + stat.variance())
        print("Variance for distance sensor: " + stat.variance())

    pass


def arduino_has_been_reset():
    print("Arduino is offline.. Maybe prepare for new initial state and re-initialize kalman filter?")


while True:

    sensor_values = arduino_receive()

    # If sensor data is read,
    if sensor_values is not None:

        # Saves the data to a CSV file
        log_measurements(sensor_values)

        # When the VSC file has exceeded 500 lines variance for both sensors are calculated
        if log_measurements():
            calculate_variance()
    else:
        arduino_has_been_reset()
