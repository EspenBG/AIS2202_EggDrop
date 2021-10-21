##
# Class for using sensor fusion with a Kalman filter
##
import numpy as np
from KalmanFilter import KalmanFilter


class SensorFusion:
    """
    Class for Sensor Fusion, this class is made with the intention to use a accelerometer and a distance sensor to
    estimate the position, speed and acceleration. Some modification may be needed for it to be used for other purposes.
    """
    def __init__(self, start_height, eval_variance, distance_variance, acceleration_variance):
        """
        :param start_height: This is the start position for the object
        :param eval_variance: The variance for the jerk (noise)
        :param distance_variance: The variance for the distance sensor (white noise)
        :param acceleration_variance: The variance for the accelerometer (white noise)
        """
        self.kf = KalmanFilter(self.getA, self.getG, eval_variance, start_value=np.array([[start_height], [0], [0]]))
        self.estimate = np.array((0, 0))
        self.delta_t = 0
        self.distance_variance = distance_variance
        self.acceleration_variance = acceleration_variance
        self.evaluation_variance = eval_variance

        self.distance_H = np.array([[1, 0, 0]])
        self.acceleration_H = np.array([[0, 0, 1]])

    def getA(self, dt):
        """
        Function to generate the A matrix
        Given as a function to Kalman filter
        :param dt:
        """
        a = np.array(
            [[1, dt, 1/2*dt**2],
             [0, 1, dt],
             [0, 0, 1]])
        return a

    def getG(self, dt):
        """
        Function to generate the G matrix
        Given as a function to Kalman filter
        :param dt:
        :return:
        """
        return np.array([[1/6*dt**3],
                         [1/2*dt**2],
                         [dt]])

    def set_timestep(self, delta_t):
        self.delta_t = delta_t

    def process_distance(self, new_value):
        """
        Get the new estimate based on a new distance measurement
        :param new_value: New measurement value
        """
        self.kf.setR(self.distance_variance)
        self.kf.setH(self.distance_H)
        self.kf.getNewSensorValue(delta_t=self.delta_t, measurement=new_value)

    def process_acceleration(self, new_value):
        """
        Get the new estimate based on a new acceleration measurement
        :param new_value: New measurement value
        """
        self.kf.setR(self.acceleration_variance)
        self.kf.setH(self.acceleration_H)
        self.kf.getNewSensorValue(delta_t=self.delta_t, measurement=new_value)

    def estimates(self):
        x_est = self.kf.get_x()
        return x_est



