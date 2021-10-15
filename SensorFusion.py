import numpy as np

from KalmanFilter import KalmanFilter


class SensorFusion:

    def __init__(self, start_height, eval_variance, distance_variance, acceleration_variance):
        self.kf = KalmanFilter(self.getA, self.getG, eval_variance, start_value=np.array([start_height, 0, 0]))
        self.estimate = np.array((0, 0))
        self.delta_t = 0
        self.distance_variance = distance_variance
        self.acceleration_variance = acceleration_variance
        self.evaluation_variance = eval_variance

        self.distance_H = np.array([1, 0, 0])
        self.acceleration_H = np.array([0, 0, 1])

    def getNewValue(self, measurement, delta_t):
        pass

    def getA(self, dt):
        a = np.array(
            [[1, dt, 1/2*dt**2],
             [0, 1, dt],
             [0, 0, 1]])
        return a

    def getG(self, dt):
        return np.array([[1/6*dt**3], [1/2*dt**2], [dt]])

    def set_timestep(self, delta_t):
        self.delta_t = delta_t

    def process_distance(self, new_value):
        self.kf.setR(self.distance_variance)
        self.kf.setH(self.distance_H)
        self.kf.getNewSensorValue(delta_t=self.delta_t, measurement=new_value)

    def process_acceleration(self, new_value):
        self.kf.setR(self.acceleration_variance)
        self.kf.setH(self.acceleration_H)
        self.kf.getNewSensorValue(delta_t=self.delta_t, measurement=new_value)

    def estimates(self):
        x_est = self.kf.get_x()
        return x_est[0], x_est[2]


#f = SensorFusion(0, 1, 2.765112489591674*1000, .00021203102776483*9.81)


